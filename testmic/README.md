# testmic

A small PlatformIO project to check a MAX9814 analog microphone amplifier with
an ESP32-S3 DevKitC-1. It prints the microphone's DC level, peak-to-peak range,
and AC RMS level to the serial monitor.

## Wiring

| MAX9814 module | ESP32-S3 DevKitC-1 |
| --- | --- |
| VDD | 3V3 |
| GND | GND |
| OUT | GPIO 34 |

Power the module from 3.3 V so its analog output stays within the ESP32-S3 ADC
input range. Do not connect the module's OUT pin to 5 V. Leave GAIN and AR at
their breakout-board defaults for this basic test.

## Run

1. Open this folder in VS Code with the PlatformIO extension.
2. Build and upload the `esp32-s3-n16r8` environment.
3. Open the serial monitor at 115200 baud.
4. Speak or make a sound near the microphone. The `p-p` and `AC RMS` values
   should rise and fall with the sound level.

The readings are raw ADC counts, not calibrated sound pressure levels. A
`[clipping?]` marker means samples are reaching the ADC's lower or upper limit.
If your ESP32 board differs, change `MIC_PIN` in `src/main.cpp` to an exposed
ADC-capable pin and update the wiring table.
