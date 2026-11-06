/*
 * Water flow sound detector for ESP32 (Arduino framework), MAX9814 version
 *
 * The MAX9814 is an ANALOG microphone amplifier (with automatic gain
 * control), so it is read with the ESP32's ADC instead of I2S.
 * The sketch measures sound energy between 0.5 and 4.5 kHz and triggers an
 * alarm when the level stays high for longer than FLOW_ALARM_MS.
 *
 * Wiring (MAX9814 module -> ESP32):
 *   VDD  -> 3.3V         (NOT 5V: the output must stay below 3.3V)
 *   GND  -> GND
 *   OUT  -> GPIO 35     (ADC1 pin; do NOT use ADC2 pins, they clash with Wi-Fi)
 *   GAIN -> VDD          (40 dB = lowest maximum gain)
 *   AR   -> GND          (slowest AGC release, steadier level for constant noise)
 *   (GAIN/AR settings may differ between breakout boards, check yours)
 * Buzzer (active) or LED + resistor on GPIO 4.
 *
 * Works on Arduino-ESP32 2.x and 3.x, ESP32 / S2 / S3.
 *
 * TUNING: set CALIBRATION_MODE to true, open the serial monitor (115200),
 * note the dB values with water OFF and ON, then set LEVEL_HIGH / LEVEL_LOW
 * in between and set CALIBRATION_MODE back to false.
 */

#include <Arduino.h>
#include <math.h>

// ---------------------------------------------------------------- Settings
#define CALIBRATION_MODE   true      // true = only print levels, no alarm

// Pins
#define MIC_PIN            34        // MAX9814 OUT (ADC1)
#define ALARM_PIN          4
#define LED_PIN            2         // onboard LED shows "flow detected"

// Audio
#define SAMPLE_RATE        10000     // Hz, Nyquist = 5 kHz
#define BLOCK_SIZE         512       // samples per block (~51 ms)
#define SAMPLE_PERIOD_US   (1000000UL / SAMPLE_RATE)

// Detection thresholds in dB (relative scale, tune for your pipe!)
#define LEVEL_HIGH         -40.0f    // above this -> water is flowing
#define LEVEL_LOW          -46.0f    // below this -> water stopped (hysteresis)

// Timing
#define SMOOTHING          0.97f     // 0..1, higher = slower/smoother (~1.5 s)
#define FLOW_ALARM_MS      60000UL   // flow must last this long to alarm
#define STOP_DEBOUNCE_MS   5000UL    // level must stay low this long to reset
#define ALARM_MAX_MS       120000UL  // stop beeping after this long

// Frequencies (Hz) analysed with the Goertzel algorithm (must be < 5000)
const float BAND_FREQS[] = {500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500};
const int NUM_FREQS = sizeof(BAND_FREQS) / sizeof(BAND_FREQS[0]);

// ----------------------------------------------------------------- State
static uint16_t rawBuf[BLOCK_SIZE];
static float    samples[BLOCK_SIZE];
static float    goertzelCoeff[NUM_FREQS];

float    smoothedDb   = -90.0f;
bool     flowing      = false;
bool     alarmActive  = false;
uint32_t flowStartMs  = 0;
uint32_t lowSinceMs   = 0;
uint32_t alarmStartMs = 0;
uint32_t lastPrintMs  = 0;

// Diagnostics for the last block
uint16_t lastMin = 0, lastMax = 0;
float    lastActualRate = SAMPLE_RATE;

// ------------------------------------------------------- Signal processing
void setupGoertzel() {
  for (int i = 0; i < NUM_FREQS; i++) {
    float k = BAND_FREQS[i] * BLOCK_SIZE / SAMPLE_RATE;
    goertzelCoeff[i] = 2.0f * cosf(2.0f * PI * k / BLOCK_SIZE);
  }
}

// Power at one frequency, normalised by block size squared
float goertzelPower(const float *x, int n, float coeff) {
  float s1 = 0, s2 = 0;
  for (int i = 0; i < n; i++) {
    float s = x[i] + coeff * s1 - s2;
    s2 = s1;
    s1 = s;
  }
  float power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
  return power / ((float)n * (float)n);
}

// Samples one block from the ADC at SAMPLE_RATE (software paced).
// Slight timing jitter is fine, we only measure broadband noise energy.
void acquireBlock() {
  uint32_t start = micros();
  uint32_t next = start;
  for (int i = 0; i < BLOCK_SIZE; i++) {
    while ((int32_t)(micros() - next) < 0) { /* wait */ }
    rawBuf[i] = analogRead(MIC_PIN);
    next += SAMPLE_PERIOD_US;
  }
  uint32_t elapsed = micros() - start;
  lastActualRate = (float)BLOCK_SIZE * 1000000.0f / (float)elapsed;
}

// Returns band level in dB for the block in rawBuf
float computeBandLevelDb() {
  double mean = 0;
  lastMin = 4095;
  lastMax = 0;
  for (int i = 0; i < BLOCK_SIZE; i++) {
    if (rawBuf[i] < lastMin) lastMin = rawBuf[i];
    if (rawBuf[i] > lastMax) lastMax = rawBuf[i];
    mean += rawBuf[i];
  }
  float dc = (float)(mean / BLOCK_SIZE);

  // Remove DC (MAX9814 output sits at about 1.25 V), scale to about -1..1,
  // and apply a Hann window to reduce spectral leakage
  for (int i = 0; i < BLOCK_SIZE; i++) {
    float w = 0.5f * (1.0f - cosf(2.0f * PI * i / (BLOCK_SIZE - 1)));
    samples[i] = ((float)rawBuf[i] - dc) / 2048.0f * w;
  }

  float sum = 0;
  for (int f = 0; f < NUM_FREQS; f++) {
    sum += goertzelPower(samples, BLOCK_SIZE, goertzelCoeff[f]);
  }
  return 10.0f * log10f(sum + 1e-12f);
}

// ------------------------------------------------------------------ Alarm
void updateAlarmOutput(uint32_t now) {
  if (alarmActive && (now - alarmStartMs < ALARM_MAX_MS)) {
    // Beep pattern: 300 ms on, 300 ms off
    digitalWrite(ALARM_PIN, ((now / 300) % 2 == 0) ? HIGH : LOW);
  } else {
    digitalWrite(ALARM_PIN, LOW);
  }
}

void updateDetection(uint32_t now) {
  if (!flowing) {
    if (smoothedDb > LEVEL_HIGH) {
      flowing = true;
      flowStartMs = now;
      lowSinceMs = 0;
      Serial.println(">> Flow detected, timer started");
    }
  } else {
    if (smoothedDb < LEVEL_LOW) {
      if (lowSinceMs == 0) lowSinceMs = now;
      if (now - lowSinceMs > STOP_DEBOUNCE_MS) {
        flowing = false;
        alarmActive = false;
        Serial.println(">> Flow stopped, reset");
      }
    } else {
      lowSinceMs = 0;   // still loud, cancel the stop timer
    }

    if (flowing && !alarmActive && (now - flowStartMs > FLOW_ALARM_MS)) {
      alarmActive = true;
      alarmStartMs = now;
      Serial.println("!! ALARM: water running too long");
    }
  }
}

// ------------------------------------------------------------------ Setup
void setup() {
  Serial.begin(115200);
  pinMode(ALARM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);

  analogReadResolution(12);                       // 0..4095
  analogSetPinAttenuation(MIC_PIN, ADC_11db);     // input range up to about 3.1 V

  setupGoertzel();

  Serial.println();
  Serial.println(CALIBRATION_MODE ? "Water flow detector (MAX9814): CALIBRATION MODE"
                                  : "Water flow detector (MAX9814): RUNNING");
}

// ------------------------------------------------------------------- Loop
void loop() {
  acquireBlock();
  float levelDb = computeBandLevelDb();

  // Exponential smoothing in the dB domain
  if (smoothedDb < -89.0f) smoothedDb = levelDb;   // quick init
  smoothedDb = SMOOTHING * smoothedDb + (1.0f - SMOOTHING) * levelDb;

  uint32_t now = millis();

  if (CALIBRATION_MODE) {
    if (now - lastPrintMs > 500) {
      lastPrintMs = now;
      Serial.printf("level: %.1f dB  smoothed: %.1f dB  p2p: %u  rate: %.0f Hz%s\n",
                    levelDb, smoothedDb, (unsigned)(lastMax - lastMin), lastActualRate,
                    (lastMax >= 4090 || lastMin <= 5) ? "  [CLIPPING!]" : "");
      if (lastActualRate < SAMPLE_RATE * 0.9f) {
        Serial.println("   warning: ADC too slow for SAMPLE_RATE, lower it");
      }
    }
    return;
  }

  updateDetection(now);
  updateAlarmOutput(now);
  digitalWrite(LED_PIN, flowing ? HIGH : LOW);

  if (now - lastPrintMs > 2000) {
    lastPrintMs = now;
    Serial.printf("smoothed: %.1f dB  flowing: %d  alarm: %d  t=%lus\n",
                  smoothedDb, flowing, alarmActive,
                  flowing ? (unsigned long)((now - flowStartMs) / 1000) : 0UL);
  }
}
