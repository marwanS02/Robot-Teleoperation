/* Transmission of Magnetic Field data from NUM_MAG_SENSORS via BLE.
 * This code is used to transmit recorded MF in the Wrist/Glove Exo.
 *
 * Sensors used are the MLX90393 from Melexis. This code works with evaluation boards
 * from Adafruit, whose i2c addressing is {0x18, 0x19, 0x1A, 0x1B}. This should be changed
 * to {0x0C, 0x0D, 0x0E, 0x0F} if other sensors are used.
 * Do not forget to hardwire the different i2c addresses through the chip pins A0,A1.
 *
 * BLE is used from the Adafruit Feather NRF52832 board. Important: BLE max size message is 
 * 20 bytes. Since every sensor collects a uint16 measure per axis (x,y,z), for a total of 
 * 6 bytes per sensors to be transmitted, at least 2 transmissions per loop are required when
 * more than 3 sensors are used. 
 * When changing the parameters of the BLE transmission, you should check to have
 * - 1 name of 8 character
 * - 1 byte address
 * - these information chosen from the transmitter should be also reported in the code of the
 *   receiver. 
 *
 * Very important: 
 * To work, this code require a modification of the original Melexis Library, in which data are read
 * directly from the sensor chip without converting them in float. 
 * These modified functions are called "readRawData" and "readRawMeasurement".
 * Data from these functions are recorded in the native 16-bit adc measurement of the chip.
 * To obtain the correct conversion, such a conversion such be then implemented at the level of the 
 * BLE receiver.
 * It is important to note that conversion depends on the Resolution and Gain settings of each sensor.
 *
 * checked and commented by Federico
 */
// Needs ArxTypeTraits, Adafruit Unified Sensor, Adafruit BusIO, https://www.pjrc.com/teensy/package_teensy_index.json
 

//Huimin_Modified is noted where Huimin made modifications

#include <Wire.h>



#include "Adafruit_MLX90393.h"  // COSTUMIZED LIBRARY
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include "lm_library_pro_Huimin.h"
#include <time.h>
// Setup Serial Connection Parameters
#define BAUDRATE 115200
/*--------------------------------------------------------------------------------------------------------------------*/
// Setup Debugging Parameters
#define DEBUG true
#define DEBUGPIN1 16
#define DEBUGPIN2 7
#define TEST_DELAY false
/*--------------------------------------------------------------------------------------------------------------------*/
// Setup magnetic sensors parameters
#define GAIN_LEVEL 7 // allowed values integers 0-7 -- Signal gain
#define RESOLUTION_LEVEL 0 // allowed values integers 0-3 -- ADC Resolution
#define OVERSAMPLING_LEVEL 0 // allowed values integers 0-3 -- How much the signal is smoothened before ADC
#define FILTER_LEVEL 4 // allowed values integers 0-7 -- How much the signal is smoothened after ADC
#define NUM_MAG_SENSORS 3                                               //Huimin_Modified 4>3
#define SIZE_MAG_DATA 6                                                 // 2 bytes per sensor axis * 3 axis components (x,y,z)
/*--------------------------------------------------------------------------------------------------------------------*/
// Setup Bluetooth Parameters
#define THROUGHPUT_LEVEL 1 // allowed values integers 0-1 (slow or fast) -- How many bytes per second can be exchanged through the link
#define RANGE_LEVEL 7 // allowed values integers 0-7 -- How big is the connection range
#define MAX_PACKET_SIZE 20                    // maximum number of bytes that can be transmitted using BLE without splitting individual sensor data + Header and Tail
#define MAG_PAYLOAD_SIZE 4  // bluetooth buffer size - all data to be transmitted
#define NAME_BLE_DEVICE "CoroARML"  // IMPORTANT: name length is 8
#define BLUETOOTH_ADDRESS 0x33F9
#define MANUFACTURER "Adafruit Industries"
#define MODEL "Bluefruit Feather52"
#define BLE_header 0xAA
#define BLE_tail 0xBB
#define BEND_PAYLOAD_SIZE 2    
/*--------------------------------------------------------------------------------------------------------------------*/
// Setup Bending Sensors Parameters Huimin_Modified Adding bend sensor data
#define BEND_PIN 20  //Huimin_Modified Bending sensor pin                                             // 16 bytes
/*--------------------------------------------------------------------------------------------------------------------*/


#define ARDUINO_ADDRESS      45
#define BUFF_SIZE_SINGLE 2 // Huimin_Modified
#define BUFF_SIZE_BYTE   (4 * BUFF_SIZE_SINGLE) // 10 floats * 4 = 40 bytes
float sensorDataFloat[BUFF_SIZE_SINGLE]; // 3 float/single readings per magnetic sensor + 1 bending sensor
byte sensorDataBytes[BUFF_SIZE_BYTE];    // same buffer as above but converted in bytes


#define DEV_RADIUS  (35.73E-3)

// +1 is for the reference which is not included in the tracking algorithm
int sensor_2_ignore[0];
int size = 0;
LM_library LM;

float state;

float thetaw;
float bendVal_raw;
// Magnetic field sensors
Adafruit_MLX90393 sensors[NUM_MAG_SENSORS];

// Possible addresses for the magnetic sensors
uint8_t addresses[] = { 0x18, 0x19, 0x1A };  //Huimin_Modified 4>3


union u_split {
  byte to_byte[4];
  float to_float;
} split;

// Timing
unsigned long t0 = 0, t1 = 0, t_diff = 0;
float t_frequ = 300;  // in Hz (max 1150Hz)

int loop_counter = 0;
const int trigger_loop = 50;  // e.g., every 50th loop inject 1

bool mag_valid  = false;
bool bend_valid = false;


void setup() {
 //SCB_AIRCR = 0x05FA0004;  // System reset register for ARM Cortex-M7

  if (TEST_DELAY)
    setup_debug_pins(DEBUGPIN1, DEBUGPIN2);
  // configure and open serial port
  setup_SerialPort();
  
  if (!TEST_DELAY){
    // Begin I2C with magnetic sensors
    begin_MAG_I2C(NUM_MAG_SENSORS, sensors, addresses);
    // Signal conditioning setup 
    setup_MagSensors(NUM_MAG_SENSORS, sensors, GAIN_LEVEL, RESOLUTION_LEVEL, OVERSAMPLING_LEVEL, FILTER_LEVEL);
    // Huimin_Modified Initial Bending sensor Pin
    setup_BendingSensor();
  }

  Wire1.begin(ARDUINO_ADDRESS);
  Wire1.setClock(400000);
  Wire1.onRequest(requestEvent);
  //Serial1.begin(115200);  // Or your preferred baud rate
  Serial.println("Setup");
 
}


void requestEvent() {
  union {
    float f;
    uint8_t b[4];
  } split_local;

  
  for (int i = 0; i < BUFF_SIZE_SINGLE; i++){
    split_local.f = sensorDataFloat[i];
    for (int j = 0; j < 4; j++){
      sensorDataBytes[(4 * i) + j] = split_local.b[j];
    }
  }


  Wire1.write(sensorDataBytes, BUFF_SIZE_BYTE);
}



void loop() {
  t0 = micros();
  if (TEST_DELAY) /* to measure loop rate execution (use oscilloscope) */
    loop_counter++;
  
    if (loop_counter == trigger_loop) {
      togglePin(DEBUGPIN1);
      loop_counter = 0;
    }


    if (TEST_DELAY){
    state  = readTestData();
    sensorDataFloat[0] = state;
    sensorDataFloat[1] = state;
  }
  else {
    // ---------- MAGNET ----------
    thetaw = readMagData();   // this already enforces safe = 0
    sensorDataFloat[0] = thetaw;

    // ---------- BEND WATCHDOG ----------
    static uint16_t last_good_bend_raw = 1000;  // some mid/neutral value
    static uint8_t  bend_bad_count     = 0;
    const uint8_t   MAX_BEND_BAD       = 10;    // adjust if needed
    const uint16_t  LOW_RAW_LIMIT      = 10;    // treat very low as suspicious
    const uint16_t  HIGH_RAW_LIMIT     = 4095 - 10; // treat near max as suspicious

    bendVal_raw = readBendData();  // raw ADC value

    // Very simple plausibility check:
    bool bend_plausible =
        (bendVal_raw > LOW_RAW_LIMIT) &&
        (bendVal_raw < HIGH_RAW_LIMIT);

    if (bend_plausible) {
      // Normal case: accept, reset counters
      bend_valid          = true;
      bend_bad_count      = 0;
      last_good_bend_raw  = bendVal_raw;

      // Here you still export the real raw value
      sensorDataFloat[1] = (float) bendVal_raw;
    } else {
      // Suspicious reading → increase bad counter
      if (bend_bad_count < MAX_BEND_BAD) {
        bend_bad_count++;
      }

       if (bend_bad_count >= MAX_BEND_BAD) {
        // Consider bend sensor disconnected → HOLD LAST VALUE
        bend_valid         = false;
        sensorDataFloat[1] = (float) last_good_bend_raw;   // <--- HOLD LAST
        if (DEBUG) {
          Serial.println("Bend safety: timeout, holding last bend");
        }
      } else {
        // Short glitch → hold last good bend
        bend_valid         = true;
        sensorDataFloat[1] = (float) last_good_bend_raw;
      }

    }
  }

  
/*
  uint8_t uartBuffer[10];  // 2 floats = 8 bytes
  uartBuffer[0] = 0xAA;
  for (int i = 0; i < 2; ++i) {
    union {
      float f;
      uint8_t b[4];
    } split;
    split.f = sensorDataFloat[i];
    memcpy(&uartBuffer[i * 4 + 1], split.b, 4);
  }
  
  uartBuffer[9] = 0xBB;
  int16_t space = Serial4.availableForWrite();
  if (space >= 10) {
    printBuffer(uartBuffer,10);
    size_t n = Serial4.write(uartBuffer, 10);  // Send 2 floats (8 bytes)
    (void)n; // n==10 expected; we ignore partial writes by construction
  }*/
  
  
  //if (DEBUG) printBuffer(uartBuffer,10);
  t1 = micros();
  t_diff = t1 - t0;
  while (t_diff < 1000000 / t_frequ ) {
    t1 = micros();
    t_diff = t1 - t0;
  }
  //Serial.print("loop period: ");
  //Serial.println(t_diff);
  //Serial.print("loop freq: ");
  //Serial.println(1000000/t_diff);
  // Blocking sleep to make the loop run AT MOST at t_frequ

}

void begin_MAG_I2C(uint8_t num_mag_sensors,
                   Adafruit_MLX90393 sensors[],
                   const uint8_t addresses[]) {
  for (uint8_t i = 0; i < num_mag_sensors; i++) {
    if (!sensors[i].begin_I2C(addresses[i])) {
      Serial.print("Cannot find sensor at address 0x");
      Serial.println(addresses[i], HEX);
      while (1);  // Halt here
    }
  }
}

void setup_debug_pins(uint8_t debug_pin1, uint8_t debug_pin2){
  pinMode(debug_pin1, OUTPUT);
  pinMode(debug_pin2, OUTPUT);
}



void setup_SerialPort() {
  Serial.begin(BAUDRATE);
  Serial4.begin(BAUDRATE);
  //Serial2.begin(BAUDRATE);
  //while (!Serial) delay(10);
}


void setup_MagSensors(uint8_t num_sensors, Adafruit_MLX90393 sensors[], uint8_t gain_level, uint8_t resolution_level, uint8_t oversampling_level,
                   uint8_t filter_level) {

  
  // assign rotation matrices to corresponding sensors
  Rsens[0] = wRs1;
  Rsens[1] = wRs2;
  Rsens[2] = wRs3;
  // assign sensors position
  float sx1 = DEV_RADIUS*sin(PI/6);
  float sz1 = DEV_RADIUS*(1-cos(PI/6));

  Eigen::Matrix<float, 3, 1> s1{ { -sx1 }, { 0 }, { -sz1 } };
  Eigen::Matrix<float, 3, 1> s2{ {  sx1 }, { 0 }, { -sz1 } };
  sPos.row(0) = s1;
  sPos.row(1) = s2;
  for (uint8_t i = 0; i < num_sensors; i++) {
    setup_gain(sensors[i], gain_level);
    setup_resolution(sensors[i], resolution_level);
    setup_oversampling(sensors[i], oversampling_level);
    setup_filter(sensors[i], filter_level);
    // Set Burst mode
    sensors[i].setBurstMode(MLX90393_AXIS_ALL, 0);

    // Start Burst mode for each sensor
    if (!sensors[i].startBurstMode()) {
      Serial.printf("Failed to start measurement for sensor %d\n", i + 1);
    }
  }
}

void setup_gain(Adafruit_MLX90393& sensor, uint8_t level) {
  mlx90393_gain_t gain;

  if (level == 0)
    gain = MLX90393_GAIN_1X;
  else if (level == 1)
    gain = MLX90393_GAIN_1_33X;
  else if (level == 2)
    gain = MLX90393_GAIN_1_67X;
  else if (level == 3)
    gain = MLX90393_GAIN_2X;
  else if (level == 4)
    gain = MLX90393_GAIN_2_5X;
  else if (level == 5)
    gain = MLX90393_GAIN_3X;
  else if (level == 6)
    gain = MLX90393_GAIN_4X;
  else if (level == 7)
    gain = MLX90393_GAIN_5X;
  else
    gain = MLX90393_GAIN_1X;

  sensor.setGain(gain);
}

void setup_resolution(Adafruit_MLX90393& sensor, uint8_t level) {
  mlx90393_resolution_t resolution;

  if (level == 0)
    resolution = MLX90393_RES_16;
  else if (level == 1)
    resolution = MLX90393_RES_17;
  else if (level == 2)
    resolution = MLX90393_RES_18;
  else if (level == 3)
    resolution = MLX90393_RES_19;
  else 
    resolution = MLX90393_RES_16;
  sensor.setResolution(MLX90393_X, resolution);
  sensor.setResolution(MLX90393_Y, resolution);
  sensor.setResolution(MLX90393_Z, resolution);
}

void setup_oversampling(Adafruit_MLX90393& sensor, uint8_t level) {
  mlx90393_oversampling_t oversampling;

  if (level == 0)
    oversampling = MLX90393_OSR_0;
  else if (level == 1)
    oversampling = MLX90393_OSR_1;
  else if (level == 2)
    oversampling = MLX90393_OSR_2;
  else if (level == 3)
    oversampling = MLX90393_OSR_3;
  else 
    oversampling = MLX90393_OSR_0;  
  sensor.setOversampling(oversampling);
}

void setup_filter(Adafruit_MLX90393& sensor, uint8_t level) {
  mlx90393_filter_t filter;

  if (level == 0)
    filter = MLX90393_FILTER_0;
  else if (level == 1)
    filter = MLX90393_FILTER_1;
  else if (level == 2)
    filter = MLX90393_FILTER_2;
  else if (level == 3)
    filter = MLX90393_FILTER_3;
  else if (level == 4)
    filter = MLX90393_FILTER_4;
  else if (level == 5)
    filter = MLX90393_FILTER_5;
  else if (level == 6)
    filter = MLX90393_FILTER_6;
  else if (level == 7)
    filter = MLX90393_FILTER_7;
  else 
    filter = MLX90393_FILTER_0;  
  sensor.setFilter(filter);
}



void setup_BendingSensor(){
  pinMode(BEND_PIN, INPUT);
}

float readMagData() {
  float thetaW;

  static float last_good_thetaW = 0.0f;   // <--- NEW: persistent last good value
  bool all_ok = true;                     // track if all MLX sensors replied successfully

  // Read measurements from all sensors
  for (int i = 0; i < NUM_SENSORS+1; i++) {

    float x, y, z;
    if (sensors[i].readBurstMeasurement(&x, &y, &z)) {
      
      sensorData(i, 0) = x;
      sensorData(i, 1) = y;
      sensorData(i, 2) = z;

      sensorMagneticField.row(i) = sensorData.row(i) * Rsens[i].transpose();

    } else {
      all_ok = false;
      Serial.printf("Unable to read XYZ data from sensor %d.\n", i + 1);
      // NOTE: we do NOT update sensorMagneticField.row(i) -> it keeps old value
    }
  }

  // If any sensor failed: return last good thetaW
  if (!all_ok) {
    mag_valid = false;
    thetaW    = last_good_thetaW;   // <--- HOLD LAST
    if (DEBUG) {
      Serial.println("Magnet safety: sensor read failed, holding last thetaW");
    }
    return thetaW;
  }

  // sensorField stores only the data required for tracking
  sensorField.row(0) = sensorMagneticField.row(0) - sensorMagneticField.row(2);
  sensorField.row(1) = sensorMagneticField.row(1) - sensorMagneticField.row(2);

  LM.localize_magnets_lmder_huimin(x0, sensorMagneticField, sPos, sensor_2_ignore, size, valrel);

  // Check if magnet is in a reasonable region
  float r_norm = std::sqrt(x0[0]* x0[0] + x0[1] * x0[1] + x0[2] * x0[2]);

  bool in_range =
      (r_norm <= 73.0f/1000.0f) &&   // <= 73 mm from origin
      (fabs(x0[0]) < 0.033f);        // |x| < 33 mm

  if (!in_range) {
    // Magnet is too far or inside ring → no valid new measurement
    mag_valid = false;
    thetaW    = last_good_thetaW;    // <--- HOLD LAST
    if (DEBUG) {
      Serial.println("Magnet safety: out of range, holding last thetaW");
    }
    return thetaW;
  }

  // Normal case: compute thetaW from tracked x0
  thetaW = (fmod(atan2(x0[2] + DEV_RADIUS, x0[0]) , (2*PI)) - PI/2 ) * 180.0f / PI;

  if (DEBUG) {
    Serial.print("Mag Val: ");
    Serial.println(thetaW);
  }

  mag_valid        = true;
  last_good_thetaW = thetaW;         // <--- UPDATE LAST GOOD VALUE
  return thetaW;
}



float readTestData() {
  
  float state = digitalRead(DEBUGPIN1);
  
  if (DEBUG) {
    Serial.print("Test raw: ");
    Serial.println(state);
    //Serial.print(" | Voltage: ");
    //Serial.print((float)bendValue * 3.3 / 1023.0);  // Convert to voltage (if 10-bit ADC)
    //Serial.println(" V");
  }
  return state;
}

uint16_t readBendData() {
  uint16_t bendValue = analogRead(BEND_PIN);

  
  if (DEBUG) {
    Serial.print("Bend raw: ");
    Serial.println(bendValue);
    //Serial.print(" | Voltage: ");
    //Serial.print((float)bendValue * 3.3 / 1023.0);  // Convert to voltage (if 10-bit ADC)
    //Serial.println(" V");
  }
  return bendValue;
}


/** This is a Debug function to print out the elements from the buffer
  * to be sent via bluetooth. 
  */
void printBuffer(uint8_t* buffer, size_t length) {
  for (size_t i = 0; i < length; i++) {
    Serial.print(buffer[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void togglePin(uint8_t pin) {
  uint8_t state = digitalRead(pin);
  // Toggle the state
  if (state == HIGH)
    digitalWrite(pin, LOW);
  else
    digitalWrite(pin, HIGH);
}



uint8_t LSB(uint16_t x) {
  return x & 0xFF;
}

uint8_t MSB(uint16_t x) {
  return (x >> 8) & 0xFF;
}

