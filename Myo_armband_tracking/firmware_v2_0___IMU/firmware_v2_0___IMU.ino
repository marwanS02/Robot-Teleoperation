/*
  Myo Armband BLE → UART Gateway
  ------------------------------
  Author: Mohamad Marwan Sidani
*/

#include <bluefruit.h>
#include <string.h>   // <-- NEW: for memcpy
#include <math.h>     // <-- NEW: for rounding float to int

// --- Myo base UUIDs (little-endian form used by Bluefruit) ---
const uint8_t UUID_MYO_CONTROL[16]   = {0x42,0x48,0x12,0x4A,0x7F,0x2C,0x48,0x47,0xB9,0xDE,0x04,0xA9,0x01,0x00,0x06,0xD5};
const uint8_t UUID_MYO_CMD_CHAR[16]  = {0x42,0x48,0x12,0x4A,0x7F,0x2C,0x48,0x47,0xB9,0xDE,0x04,0xA9,0x01,0x04,0x06,0xD5};

const uint8_t UUID_MYO_EMG_SVC[16]   = {0x42,0x48,0x12,0x4A,0x7F,0x2C,0x48,0x47,0xB9,0xDE,0x04,0xA9,0x05,0x00,0x06,0xD5};
const uint8_t UUID_MYO_EMG_0[16]     = {0x42,0x48,0x12,0x4A,0x7F,0x2C,0x48,0x47,0xB9,0xDE,0x04,0xA9,0x05,0x01,0x06,0xD5};
const uint8_t UUID_MYO_EMG_1[16]     = {0x42,0x48,0x12,0x4A,0x7F,0x2C,0x48,0x47,0xB9,0xDE,0x04,0xA9,0x05,0x02,0x06,0xD5};
const uint8_t UUID_MYO_EMG_2[16]     = {0x42,0x48,0x12,0x4A,0x7F,0x2C,0x48,0x47,0xB9,0xDE,0x04,0xA9,0x05,0x03,0x06,0xD5};
const uint8_t UUID_MYO_EMG_3[16]     = {0x42,0x48,0x12,0x4A,0x7F,0x2C,0x48,0x47,0xB9,0xDE,0x04,0xA9,0x05,0x04,0x06,0xD5};
uint8_t          last_haptic_state = 1;   // 1 = "nothing"
unsigned long    last_haptic_vib_ms = 0;

// Estimated Myo vibration durations in ms (approx)
const unsigned long VIB_DUR_MS_STATE2 = 50;   // short
const unsigned long VIB_DUR_MS_STATE3 = 50;   // short (unused in current logic)
const unsigned long VIB_DUR_MS_STATE4 = 250;  // long

// NEW: EMG block window (stop sending EMG during vib + 500 ms)
unsigned long emg_block_until_ms = 0;
const unsigned long EMG_BLOCK_EXTRA_MS = 500;   // 500 ms after vibration


// Service d5060002-a904-deb9-4748-2c7f4a124842
const uint8_t UUID_MYO_IMU_SVC[16]   = {0x42,0x48,0x12,0x4A,0x7F,0x2C,0x48,0x47,0xB9,0xDE,0x04,0xA9,0x02,0x00,0x06,0xD5};
// Data    d5060402-a904-deb9-4748-2c7f4a124842  (IMU data notify)
const uint8_t UUID_MYO_IMU_DATA[16]  = {0x42,0x48,0x12,0x4A,0x7F,0x2C,0x48,0x47,0xB9,0xDE,0x04,0xA9,0x02,0x04,0x06,0xD5};
BLEClientService        svcIMU(UUID_MYO_IMU_SVC);
BLEClientCharacteristic chrIMU(UUID_MYO_IMU_DATA);

// OPTIONAL: lock to your MAC (df:da:1a:1d:a6:35)
const bool USE_MAC_FILTER = false;
const uint8_t MYO_MAC[6]  = {0xDF,0xDA,0x1A,0x1D,0xA6,0x35};

BLEClientService        svcControl(UUID_MYO_CONTROL);
BLEClientCharacteristic chrCommand(UUID_MYO_CMD_CHAR);

BLEClientService        svcEMG(UUID_MYO_EMG_SVC);
BLEClientCharacteristic chrEMG0(UUID_MYO_EMG_0);
BLEClientCharacteristic chrEMG1(UUID_MYO_EMG_1);
BLEClientCharacteristic chrEMG2(UUID_MYO_EMG_2);
BLEClientCharacteristic chrEMG3(UUID_MYO_EMG_3);

uint16_t conn_handle = BLE_CONN_HANDLE_INVALID;

// ===== HSTATE PARSER STATE (NEW) ===================================
enum HStateParseState {
  HSTATE_WAIT_HEADER1 = 0,
  HSTATE_WAIT_HEADER2,
  HSTATE_READ_PAYLOAD
};

HStateParseState hstate_parse_state = HSTATE_WAIT_HEADER1;
uint8_t          hstate_payload[4];
uint8_t          hstate_payload_idx = 0;

const unsigned long HAPTIC_MIN_INTERVAL_MS = 2000; // cooldown between pulses
// ===================================================================

// ---------- Helpers ----------
bool mac_match(const ble_gap_addr_t& addr) {
  for (int i = 0; i < 6; ++i) if (addr.addr[i] != MYO_MAC[i]) return false;
  return true;
}

// Vibrate: 1=short, 2=med, 3=long
void vibrate(uint8_t type = 1) {
  // Decide approximate vibration duration based on Myo pattern
  unsigned long vib_ms = 50;  // default

  switch (type) {
    case 1: vib_ms = VIB_DUR_MS_STATE2; break;  // short
    case 2: vib_ms = VIB_DUR_MS_STATE3; break;  // another short / medium
    case 3: vib_ms = VIB_DUR_MS_STATE4; break;  // long
    default: break;
  }

  unsigned long now = millis();

  // Block EMG during vibration + 500 ms
  emg_block_until_ms = now + vib_ms + EMG_BLOCK_EXTRA_MS;

  // Send the Myo vibration command
  uint8_t cmd[3] = { 0x03, 0x01, type };
  chrCommand.write(cmd, sizeof(cmd));
}


// ===== HSTATE HANDLER (NEW) ========================================
// void handle_hstate_frame(const uint8_t *payload4) {
//   float f;
//   memcpy(&f, payload4, 4);          // interpret 4 bytes as little-endian float
//   int state = (int)roundf(f);       // 1..4 ideally

//   if (state < 1 || state > 4) {
//     return;
//   }

//   // State 1 = no haptic, just update last state and exit
//   if (state == 1) {
//     last_haptic_state = (uint8_t)state;
//     return;
//   }

//   unsigned long now = millis();

//   // *** ONE-SHOT: only vibrate on state CHANGE ***
//   if (state == last_haptic_state) {
//     // same state as before → do nothing
//     return;
//   }

//   // *** NEW: cooldown – skip any haptic if last one was < 2 s ago ***
//   if (now - last_haptic_vib_ms < HAPTIC_MIN_INTERVAL_MS) {
//     // ignore this command, but DO NOT update last_haptic_state
//     // so that once cooldown expires, a new frame with this state can still vibrate
//     return;
//   }

//   // Different haptic patterns:
//   switch (state) {
//     case 2: // near border: one short buzz
//       vibrate(1);   // short
//       break;

//     case 3: // hit/exceeded border: double short buzz
//       vibrate(1);
//       delay(80);    // short gap
//       vibrate(1);
//       break;

//     case 4: // grasp success: one long buzz
//       vibrate(3);   // long
//       break;

//     default:
//       break;
//   }

//   last_haptic_state   = (uint8_t)state;
//   last_haptic_vib_ms  = now;
// }

void handle_hstate_frame(const uint8_t *payload4) {
  float f;
  memcpy(&f, payload4, 4);          // interpret 4 bytes as little-endian float
  int state = (int)roundf(f);       // expected range 1..4

  if (state < 1 || state > 4) {
    return; // invalid, ignore
  }

  // FAR (1): just update and arm the next NEAR pulse
  if (state == 1) {
    last_haptic_state = 1;
    return;
  }

  // Only NEAR (2) should cause vibration
  if (state == 2) {
    // Edge trigger: vibrate only when we *enter* NEAR from FAR (1)
    if (last_haptic_state == 1) {
      vibrate(1);                   // short ~50 ms pulse on Myo
    }
    // In all cases, update last state to 2
    last_haptic_state = 2;
    return;
  }

  // BORDER (3) or GRASP (4): no vibration, just track state
  last_haptic_state = (uint8_t)state;
}



// ===================================================================

// --- Enable raw EMG + IMU, classifier off ---
bool set_mode_emg_raw_and_imu() {
  uint8_t payload[5] = { 0x01, 0x03, 0x02, 0x01, 0x00 };  // emg=0x02 raw, imu=0x01 on, classifier=0x00
  return chrCommand.write(payload, sizeof(payload));
}

// IMU notify callback ...
void imu_notify_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  if (!data || len < 20) return;
  auto rd_i16 = [&](int idx)->int16_t {
    return (int16_t)((data[2*idx+1]<<8) | data[2*idx]);
  };

  int16_t qw = rd_i16(0), qx = rd_i16(1), qy = rd_i16(2), qz = rd_i16(3);
  int16_t ax = rd_i16(4), ay = rd_i16(5), az = rd_i16(6);
  int16_t gx = rd_i16(7), gy = rd_i16(8), gz = rd_i16(9);

  unsigned long t = micros();
  int src = 0;

  Serial.print('i'); Serial.print(src); Serial.print(',');
  Serial.print(t); Serial.print(',');
  Serial.print(qw); Serial.print(','); Serial.print(qx); Serial.print(',');
  Serial.print(qy); Serial.print(','); Serial.print(qz); Serial.print(',');
  Serial.print(ax); Serial.print(','); Serial.print(ay); Serial.print(',');
  Serial.print(az); Serial.print(',');
  Serial.print(gx); Serial.print(','); Serial.print(gy); Serial.print(',');
  Serial.print(gz); Serial.print('\n');
}

// Set mode: EMG=raw, IMU off, Classifier off (unused here)
bool set_mode_emg_raw_only() {
  uint8_t payload[5] = { 0x01, 0x03, 0x02, 0x00, 0x00 };
  return chrCommand.write(payload, sizeof(payload));
}

bool sleep_mode_never() {
  uint8_t cmd[3] = { 0x09, 0x01, 0x00 };
  return chrCommand.write(cmd, sizeof(cmd));
}

bool unlock_hold() {
  uint8_t cmd[3] = { 0x0A, 0x01, 0x01 };
  return chrCommand.write(cmd, sizeof(cmd));
}

int emg_src_index(BLEClientCharacteristic* chr) {
  if (chr == &chrEMG0) return 0;
  if (chr == &chrEMG1) return 1;
  if (chr == &chrEMG2) return 2;
  if (chr == &chrEMG3) return 3;
  return -1;
}

void emg_notify_cb(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  if (!data || len < 16) return;

  // NEW: drop EMG during haptic + 500 ms after
  unsigned long now_ms = millis();
  if (now_ms < emg_block_until_ms) {
    // We *do not* buffer — notifications are simply ignored.
    return;
  }

  int src = emg_src_index(chr);
  if (src < 0) return;

  unsigned long t = micros();

  // First sample
  Serial.print('e'); Serial.print(src); Serial.print(',');
  Serial.print(t); Serial.print(',');
  for (int i = 0; i < 8; ++i) {
    int8_t v = (int8_t)data[i];
    Serial.print(v);
    Serial.print(i < 7 ? ',' : '\n');
  }

  // Second sample
  Serial.print('e'); Serial.print(src); Serial.print(',');
  Serial.print(t); Serial.print(',');
  for (int i = 0; i < 8; ++i) {
    int8_t v = (int8_t)data[8 + i];
    Serial.print(v);
    Serial.print(i < 7 ? ',' : '\n');
  }
}


// ---------- Scan / Connect flow ----------
void scan_cb(ble_gap_evt_adv_report_t* report) {
  if (USE_MAC_FILTER && !mac_match(report->peer_addr)) {
    Bluefruit.Scanner.resume();
    return;
  }
  Bluefruit.Central.connect(report);
}

void connect_cb(uint16_t handle) {
  conn_handle = handle;
  Serial.println(F("Connected. Discovering Myo services..."));

  ble_gap_conn_params_t params;
  params.min_conn_interval = 9;
  params.max_conn_interval = 16;
  params.slave_latency     = 0;
  params.conn_sup_timeout  = 600;
  Bluefruit.Connection(handle)->requestConnectionParameter(12, 0, 600);
  Bluefruit.Connection(handle)->requestDataLengthUpdate();
  Bluefruit.Connection(handle)->requestMtuExchange(247);

  if (!svcControl.discover(handle)) { Serial.println(F("No Control svc.")); Bluefruit.disconnect(handle); return; }
  if (!chrCommand.discover())       { Serial.println(F("No Command chr.")); Bluefruit.disconnect(handle); return; }

  if (!svcEMG.discover(handle))     { Serial.println(F("No EMG svc."));     Bluefruit.disconnect(handle); return; }
  if (!chrEMG0.discover())          { Serial.println(F("No EMG0 chr."));    Bluefruit.disconnect(handle); return; }
  if (!chrEMG1.discover())          { Serial.println(F("No EMG1 chr."));    Bluefruit.disconnect(handle); return; }
  if (!chrEMG2.discover())          { Serial.println(F("No EMG2 chr."));    Bluefruit.disconnect(handle); return; }
  if (!chrEMG3.discover())          { Serial.println(F("No EMG3 chr."));    Bluefruit.disconnect(handle); return; }

  if (!set_mode_emg_raw_and_imu()) { Serial.println(F("Failed set_mode.")); Bluefruit.disconnect(handle); return; }
  if (!sleep_mode_never())         { Serial.println(F("Failed sleep_off.")); Bluefruit.disconnect(handle); return; }
  if (!unlock_hold())              { Serial.println(F("Failed unlock."));    Bluefruit.disconnect(handle); return; }

  if (!svcIMU.discover(handle))   { Serial.println(F("No IMU svc."));        Bluefruit.disconnect(handle); return; }
  if (!chrIMU.discover())         { Serial.println(F("No IMU data chr."));   Bluefruit.disconnect(handle); return; }
  chrIMU.setNotifyCallback(imu_notify_cb);
  if (!chrIMU.enableNotify())     { Serial.println(F("IMU notify failed.")); Bluefruit.disconnect(handle); return; }

  chrEMG0.setNotifyCallback(emg_notify_cb);
  chrEMG1.setNotifyCallback(emg_notify_cb);
  chrEMG2.setNotifyCallback(emg_notify_cb);
  chrEMG3.setNotifyCallback(emg_notify_cb);

  bool ok = true;
  ok &= chrEMG0.enableNotify();
  ok &= chrEMG1.enableNotify();
  ok &= chrEMG2.enableNotify();
  ok &= chrEMG3.enableNotify();

  if (!ok) { Serial.println(F("Failed to enable EMG notifications.")); Bluefruit.disconnect(handle); return; }

  //vibrate(1);
  Serial.println(F("Ready. Streaming EMG (e,...) ~200 Hz and IMU (i,...) ~50 Hz."));
}

void disconnect_cb(uint16_t, uint8_t) {
  conn_handle = BLE_CONN_HANDLE_INVALID;
  Serial.println(F("Disconnected. Rescanning..."));
  Bluefruit.Scanner.start(0);
}

// ===== HSTATE BYTE-PARSER (NEW) ====================================
void poll_hstate_from_serial() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();

    switch (hstate_parse_state) {
      case HSTATE_WAIT_HEADER1:
        if (b == 0xAA) {
          hstate_parse_state = HSTATE_WAIT_HEADER2;
        }
        break;

      case HSTATE_WAIT_HEADER2:
        if (b == 0x55) {
          hstate_parse_state = HSTATE_READ_PAYLOAD;
          hstate_payload_idx = 0;
        } else {
          // not correct second header, restart
          hstate_parse_state = HSTATE_WAIT_HEADER1;
        }
        break;

      case HSTATE_READ_PAYLOAD:
        hstate_payload[hstate_payload_idx++] = b;
        if (hstate_payload_idx >= 4) {
          // full frame received
          handle_hstate_frame(hstate_payload);
          hstate_parse_state = HSTATE_WAIT_HEADER1;
        }
        break;
    }
  }
}
// ===================================================================

// ---------- Setup / Loop ----------
unsigned long last_keepalive = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(5); }

  Bluefruit.begin(0, 1); // central only
  Bluefruit.setName("Feather-Myo-EMG");
  Bluefruit.Central.setConnectCallback(connect_cb);
  Bluefruit.Central.setDisconnectCallback(disconnect_cb);

  svcControl.begin();  chrCommand.begin();
  svcEMG.begin();
  chrEMG0.begin();  chrEMG1.begin();  chrEMG2.begin();  chrEMG3.begin();
  svcIMU.begin();
  chrIMU.begin();

  Bluefruit.Scanner.setRxCallback(scan_cb);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.filterUuid(UUID_MYO_CONTROL);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.start(0);

  Serial.println(F("Scanning for Myo..."));
}

void loop() {
  // Keep Myo awake
  if (millis() - last_keepalive > 10000 && conn_handle != BLE_CONN_HANDLE_INVALID) {
    sleep_mode_never();
    last_keepalive = millis();
  }

  // NEW: process incoming hstate frames from host PC
  poll_hstate_from_serial();
}
