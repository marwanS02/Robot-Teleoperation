# `firmware_v2_0___IMU`

Arduino firmware for an nRF52 Bluefruit-based board that acts as a BLE-to-USB serial gateway for a Myo armband. This version streams both raw EMG and IMU data and can also receive simple haptic state commands from the host PC over the same serial link.

## What This Sketch Does

- Scans for a Myo armband over BLE and connects as a BLE central.
- Discovers the Myo Control, EMG, and IMU services.
- Configures the Myo for raw EMG streaming with IMU enabled.
- Forwards EMG and IMU packets to the host PC over `Serial` at `115200` baud.
- Listens for host-side haptic state frames and triggers Myo vibration when the state changes into the "near" state.
- Temporarily suppresses EMG forwarding during vibration and for 500 ms afterward to reduce haptic artifacts in the EMG stream.

## Output Serial Protocol

The sketch prints newline-terminated ASCII records.

### EMG frames

Format:

```text
eK,<timestamp_us>,v0,v1,v2,v3,v4,v5,v6,v7
```

Where:

- `K` is the EMG source index in `{0,1,2,3}` and corresponds to Myo EMG characteristics `EMG0..EMG3`.
- `timestamp_us` is the MCU timestamp from `micros()`.
- `v0..v7` are signed 8-bit EMG values.

Each BLE notification contains two 8-channel samples, so the callback emits two `e...` lines per packet.

### IMU frames

Format:

```text
iK,<timestamp_us>,qw,qx,qy,qz,ax,ay,az,gx,gy,gz
```

Where:

- `K` is currently `0`.
- `timestamp_us` is the MCU timestamp from `micros()`.
- `qw,qx,qy,qz` are quaternion components as signed 16-bit integers.
- `ax,ay,az` are accelerometer values as signed 16-bit integers.
- `gx,gy,gz` are gyroscope values as signed 16-bit integers.

This matches the expected input format used by [`myo_emg_rt_pyqtgraph v2.0 + IMU.py`](../myo_emg_rt_pyqtgraph%20v2.0%20%2B%20IMU.py).

## Input Serial Protocol For Haptics

The firmware also reads binary host commands from `Serial`.

Frame format:

```text
0xAA 0x55 <float32 little-endian>
```

The 4-byte float is interpreted as a haptic state and rounded to an integer:

- `1`: far / idle, no vibration, resets the edge trigger
- `2`: near, triggers one short vibration only when transitioning from state `1`
- `3`: border, tracked internally but does not vibrate in the current logic
- `4`: grasp, tracked internally but does not vibrate in the current logic

Example sender behavior exists in the training/runtime scripts under [`train/`](../train), where frames are written as:

```python
frame = b"\xAA\x55" + struct.pack("<f", float(hstate))
```

## Haptic And EMG Artifact Handling

When a vibration command is sent to the Myo, the sketch sets an EMG block window:

- vibration duration, plus
- `500 ms` extra holdoff

During this window, incoming EMG notifications are ignored rather than buffered.

Current vibration timings in the sketch:

- short pulse: about `50 ms`
- long pulse: about `250 ms`

## Requirements

- A BLE central-capable Adafruit nRF52 / Bluefruit board
- Arduino IDE or PlatformIO
- Adafruit nRF52 Bluefruit library (`#include <bluefruit.h>`)
- A Myo armband advertising the standard Myo BLE services

## Build And Upload

1. Open [`firmware_v2_0___IMU.ino`](./firmware_v2_0___IMU.ino) in the Arduino IDE.
2. Select your Bluefruit/nRF52 board.
3. Make sure the Adafruit Bluefruit nRF52 library is installed.
4. Compile and upload the sketch.
5. Open the serial monitor at `115200` baud if you want to inspect connection logs and streamed packets.

## Optional Configuration

The sketch exposes a few easy configuration points near the top:

- `USE_MAC_FILTER`
  Set to `true` to connect only to the Myo with the MAC address stored in `MYO_MAC`.
- `MYO_MAC`
  Target MAC address when filtering is enabled.
- `EMG_BLOCK_EXTRA_MS`
  Extra EMG mute interval after a haptic pulse.

## Typical Runtime Flow

1. The board boots and starts scanning for the Myo control UUID.
2. After connecting, it discovers Control, EMG, and IMU services.
3. It sends Myo commands to:
   - enable raw EMG
   - enable IMU
   - disable sleep
   - unlock hold mode
4. It subscribes to all four EMG notify characteristics and the IMU notify characteristic.
5. It continuously forwards EMG and IMU data over USB serial.
6. It also polls serial input for `0xAA 0x55 + float32` haptic state frames.

## Companion Files In This Repo

- [`myo_emg_rt_pyqtgraph v2.0 + IMU.py`](../myo_emg_rt_pyqtgraph%20v2.0%20%2B%20IMU.py): real-time EMG + IMU viewer
- [`firmware/firmware.ino`](../firmware/firmware.ino): earlier EMG-only gateway version
- [`train/`](../train): runtime and ML scripts that can emit haptic state frames back to the board

## Troubleshooting

- If you only see `Scanning for Myo...`, confirm the Myo is awake and advertising.
- If connection succeeds but streaming does not start, check that the board supports BLE central mode.
- If your host parser breaks, make sure it ignores the plain-text status lines printed during connect/disconnect events.
- If haptics seem unresponsive, verify the host is sending the binary header `AA 55` followed by a little-endian `float32`.
- If EMG appears to "drop out" after haptics, that is expected because the sketch intentionally suppresses EMG during the artifact block window.

## Notes

- The sketch uses `Bluefruit.begin(0, 1)`, so it is configured as a BLE central only.
- IMU values are forwarded raw; scaling to physical units is handled on the host side.
- The current haptic logic is intentionally conservative: only state `2` produces a buzz.
