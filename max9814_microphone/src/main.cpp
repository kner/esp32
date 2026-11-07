/*
 * ESP32 / Arduino water-flow sound detector with analog MAX9814.
 *
 * MAX9814 wiring: VDD -> 3.3 V, GND -> GND, OUT -> MIC_PIN (ADC1).
 * GPIO 4 is ADC1 on ESP32-S3. For classic ESP32 use an ADC1 pin such as GPIO 35.
 * Original module settings: GAIN -> VDD, AR -> GND.
 * ALARM_PIN drives the alarm, LED_PIN indicates detected flow.
 *
 * Calibration: open Serial Monitor at 115200 baud and send s. Open and
 * close the tap several times at your own pace over the next three minutes.
 * Leave each state steady for several seconds. Start with a quiet interval
 * if possible so the online FFT gate can learn the background early.
 *
 * Copy the four reported defines into Settings, fill in the Wi-Fi credentials
 * and set CALIBRATION_MODE to false to enable the original alarm and HTTP POST.
 * The reported levels are relative to the ADC scale, not calibrated dB SPL.
 */

#include <Arduino.h>
#include <math.h>
#include <algorithm>
#include <cstring>
#include <WiFi.h>
#include <HTTPClient.h>

// ---------------------------------------------------------------- Wi-Fi
const char* WIFI_SSID  = "pf19wlan";
const char* WIFI_PASS  ="Silupada2025!";
const char* SERVER_URL = "http://192.168.1.9/event";   // IP of the server board
const char* TOKEN      = "change-me";                   // same as on the server

// ---------------------------------------------------------------- Settings
#define CALIBRATION_MODE   false       // false after copying the reported settings

// Pins
#define MIC_PIN            4          // MAX9814 OUT (ADC1)
#define ALARM_PIN          5
#define LED_PIN            6          // LED shows "flow detected"

// 10 kHz leaves room above the 4 kHz measurement band.
constexpr int SAMPLE_RATE = 10000;
constexpr int BLOCK_SIZE = 512;
constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE;
constexpr int BIN_STEP = 2;

// Replace these defaults with the values printed after calibration.
#define BAND_LOW_HZ        1000
#define BAND_HIGH_HZ       1400
#define LEVEL_HIGH         -35.0f
#define LEVEL_LOW          -40.0f
static_assert(LEVEL_LOW < LEVEL_HIGH, "LEVEL_LOW must be below LEVEL_HIGH");

// Timing
#define SMOOTHING          0.976f      // ~2 s with 51.2 ms blocks
#define FLOW_ALARM_MS      5000UL
#define STOP_DEBOUNCE_MS   5000UL
#define ALARM_MAX_MS       1200UL

struct Band { int low, high; };
float bandLevelDb(Band band);  // Declare after Band for the Arduino preprocessor.
constexpr Band operatingBand = {BAND_LOW_HZ, BAND_HIGH_HZ};
constexpr Band spectrumBand = {200, 4000};
constexpr float OUTLIER_RATIO = 100.0f;  // 20 dB above the median

static uint16_t rawBuf[BLOCK_SIZE];
static float fftReal[BLOCK_SIZE], fftImag[BLOCK_SIZE];
static float hann[BLOCK_SIZE];
static float twiddleReal[BLOCK_SIZE / 2], twiddleImag[BLOCK_SIZE / 2];
static float powerBins[BLOCK_SIZE / 2];
static float sortBuf[BLOCK_SIZE / 2];

float    smoothedDb   = -90.0f;
bool     flowing      = false;
bool     alarmActive  = false;
uint32_t flowStartMs  = 0;
uint32_t lowSinceMs   = 0;
uint32_t alarmStartMs = 0;
uint32_t lastPrintMs  = 0;

// ------------------------------------------------------- Signal processing
void setupSpectrum() {
  for (int i = 0; i < BLOCK_SIZE; ++i) {
    hann[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (BLOCK_SIZE - 1)));
  }
  for (int i = 0; i < BLOCK_SIZE / 2; ++i) {
    float angle = -2.0f * PI * i / BLOCK_SIZE;
    twiddleReal[i] = cosf(angle);
    twiddleImag[i] = sinf(angle);
  }
}

// A delayed read is flagged rather than silently shifting the frequency axis.
bool acquireBlock() {
  uint32_t next = micros();
  bool onTime = true;
  for (int i = 0; i < BLOCK_SIZE; ++i) {
    int32_t late;
    do {
      late = (int32_t)(micros() - next);
    } while (late < 0);
    if ((uint32_t)late > SAMPLE_PERIOD_US / 2) onTime = false;
    rawBuf[i] = analogRead(MIC_PIN);
    next += SAMPLE_PERIOD_US;
  }
  return onTime;
}

// Prepare the windowed signal and compute a one-sided energy bound.
float prepareSamples() {
  uint32_t sum = 0;
  for (uint16_t sample : rawBuf) sum += sample;
  const float dc = (float)sum / BLOCK_SIZE;

  float energy = 0.0f;
  for (int i = 0; i < BLOCK_SIZE; ++i) {
    float sample = (rawBuf[i] - dc) / 2048.0f * hann[i];
    fftReal[i] = sample;
    energy += sample * sample;
  }
  return 10.0f * log10f(energy / (2.0f * BLOCK_SIZE) + 1e-12f);
}

void computeSpectrum() {
  for (int i = 0, reversed = 0; i < BLOCK_SIZE; ++i) {
    if (i < reversed) std::swap(fftReal[i], fftReal[reversed]);
    fftImag[i] = 0.0f;
    for (int bit = BLOCK_SIZE / 2; bit != 0; bit >>= 1) {
      reversed ^= bit;
      if (reversed & bit) break;
    }
  }

  for (int size = 2; size <= BLOCK_SIZE; size *= 2) {
    const int half = size / 2;
    for (int start = 0; start < BLOCK_SIZE; start += size) {
      for (int offset = 0; offset < half; ++offset) {
        int a = start + offset;
        int b = a + half;
        int t = offset * (BLOCK_SIZE / size);
        float real = twiddleReal[t] * fftReal[b] - twiddleImag[t] * fftImag[b];
        float imag = twiddleReal[t] * fftImag[b] + twiddleImag[t] * fftReal[b];
        fftReal[b] = fftReal[a] - real;
        fftImag[b] = fftImag[a] - imag;
        fftReal[a] += real;
        fftImag[a] += imag;
      }
    }
  }

  const int first = (spectrumBand.low * BLOCK_SIZE + SAMPLE_RATE - 1) / SAMPLE_RATE;
  const int last = spectrumBand.high * BLOCK_SIZE / SAMPLE_RATE;
  for (int k = first; k <= last; ++k) {
    powerBins[k] = (fftReal[k] * fftReal[k] + fftImag[k] * fftImag[k])
                   / (BLOCK_SIZE * BLOCK_SIZE);
  }
}

float bandLevelDb(Band band) {
  const int first = (band.low * BLOCK_SIZE + SAMPLE_RATE - 1) / SAMPLE_RATE;
  const int last = band.high * BLOCK_SIZE / SAMPLE_RATE;
  int count = 0;
  for (int k = first; k <= last; k += BIN_STEP) sortBuf[count++] = powerBins[k];
  std::nth_element(sortBuf, sortBuf + count / 2, sortBuf + count);
  const float limit = sortBuf[count / 2] * OUTLIER_RATIO;

  float sum = 0.0f;
  int kept = 0;
  for (int k = first; k <= last; k += BIN_STEP) {
    if (powerBins[k] <= limit) {
      sum += powerBins[k];
      ++kept;
    }
  }
  return 10.0f * log10f(sum / kept * count + 1e-12f);
}

// ------------------------------------------------------------- Calibration
constexpr uint32_t WINDOW_MS = 1000;
constexpr int WINDOW_COUNT = 180;
constexpr int MIN_BLOCKS = 10;
constexpr int MIN_STATE_WINDOWS = 5;
constexpr float GATE_DB = 6.0f;
constexpr float MIN_GAP_DB = 3.0f;
static_assert(BAND_LOW_HZ >= 200 && BAND_HIGH_HZ <= 4000 &&
              BAND_HIGH_HZ - BAND_LOW_HZ >= 40, "Invalid detection band");

// Compare overlapping bands and the original band with the same estimator
// used in normal operation.
const Band candidateBands[] = {
  {200, 600}, {400, 800}, {600, 1000}, {800, 1200}, {1000, 1400},
  {1200, 1600}, {1400, 1800}, {1600, 2000}, {1800, 2200},
  {2000, 2400}, {2200, 2600}, {2400, 2800}, {2600, 3000},
  {2800, 3200}, {3000, 3400}, {3200, 3600}, {3400, 3800}, {3600, 4000},
  {200, 1000}, {600, 1400}, {1000, 1800}, {1400, 2200}, {1800, 2600},
  {2200, 3000}, {2600, 3400}, {3000, 3800}, {3200, 4000},
  {300, 2000}, operatingBand, spectrumBand
};
constexpr int CANDIDATE_COUNT = sizeof(candidateBands) / sizeof(candidateBands[0]);
struct CalibrationWindow {
  float broadSum;
  float bandSums[CANDIDATE_COUNT];
  uint16_t blocks;
  uint16_t spectra;
};

static CalibrationWindow windows[WINDOW_COUNT];
static unsigned lateBlocks, clippedBlocks;

float broadLevel(const CalibrationWindow &window) {
  return window.broadSum / window.blocks;
}

// Parseval bounds any selected band by the one-sided total energy computed
// without an FFT. Removing bins above the median can only lower their mean.
void printCalibration() {
  Serial.printf("Timing: %u, Clipping: %u Bloecke verworfen.\n", lateBlocks, clippedBlocks);
  float sorted[WINDOW_COUNT];
  int count = 0;
  for (const auto &window : windows) {
    if (window.blocks >= MIN_BLOCKS) sorted[count++] = broadLevel(window);
  }
  if (count < WINDOW_COUNT * 4 / 5) {
    Serial.println("Kalibrierung ungueltig: zu wenige gueltige Messfenster.");
    return;
  }

  std::sort(sorted, sorted + count);
  // Robust endpoints let a few short transients fall outside either state.
  const float quietRef = sorted[(count - 1) / 10];
  const float flowRef = sorted[(count - 1) * 9 / 10];
  if (flowRef - quietRef < GATE_DB + MIN_GAP_DB) {
    Serial.println("Ruhe und Wasserfluss im Gesamtpegel nicht sicher trennbar.");
    return;
  }
  const float midpoint = (quietRef + flowRef) / 2.0f;
  float quietSum = 0.0f, quietMax = -INFINITY;
  float flowSum[CANDIDATE_COUNT] = {};
  float flowMin[CANDIDATE_COUNT];
  std::fill(flowMin, flowMin + CANDIDATE_COUNT, INFINITY);
  int quietCount = 0, flowCount = 0;

  // Use the center of steady stretches, away from switching and AGC changes.
  for (int i = 1; i < WINDOW_COUNT - 1; ++i) {
    const auto &before = windows[i - 1];
    const auto &current = windows[i];
    const auto &after = windows[i + 1];
    if (before.blocks < MIN_BLOCKS || current.blocks < MIN_BLOCKS ||
        after.blocks < MIN_BLOCKS) continue;
    const float level = broadLevel(current);
    if (fmaxf(level, fmaxf(broadLevel(before), broadLevel(after))) <= midpoint - 1.5f) {
      quietSum += level;
      quietMax = fmaxf(quietMax, level);
      ++quietCount;
    } else if (fminf(level, fminf(broadLevel(before), broadLevel(after))) >= midpoint + 1.5f &&
               current.spectra == current.blocks) {
      for (int band = 0; band < CANDIDATE_COUNT; ++band) {
        float bandLevel = current.bandSums[band] / current.spectra;
        flowSum[band] += bandLevel;
        flowMin[band] = fminf(flowMin[band], bandLevel);
      }
      ++flowCount;
    }
  }

  Serial.printf("Stabile Messfenster: %d Ruhe, %d Wasserfluss.\n", quietCount, flowCount);
  if (quietCount < MIN_STATE_WINDOWS || flowCount < MIN_STATE_WINDOWS) {
    Serial.println("Zu wenig stabile Messzeit in mindestens einem Zustand.");
    return;
  }

  int bestBand = 0;
  for (int band = 1; band < CANDIDATE_COUNT; ++band) {
    if (flowMin[band] > flowMin[bestBand]) bestBand = band;
  }
  const float strongestFlow = flowMin[bestBand];
  const float requiredFlow = fmaxf(strongestFlow - 1.0f, quietMax + MIN_GAP_DB);
  // Prefer a narrower passband if it retains nearly the same safety margin.
  for (int band = 0; band < CANDIDATE_COUNT; ++band) {
    int width = candidateBands[band].high - candidateBands[band].low;
    int bestWidth = candidateBands[bestBand].high - candidateBands[bestBand].low;
    if (flowMin[band] >= requiredFlow && width < bestWidth) bestBand = band;
  }
  const float gap = flowMin[bestBand] - quietMax;
  const Band band = candidateBands[bestBand];
  Serial.printf("%s: %d bis %d Hz\n", gap >= MIN_GAP_DB ? "Messband" : "Bester Kandidat",
                band.low, band.high);
  Serial.printf("Wasserfluss im Band: %.1f dB relativ (unterer Messwert %.1f dB)\n",
                flowSum[bestBand] / flowCount, flowMin[bestBand]);
  Serial.printf("Ruhe: im Mittel hoechstens %.1f dB relativ "
                "(obere Grenze %.1f dB)\n", quietSum / quietCount, quietMax);
  if (gap < MIN_GAP_DB) {
    Serial.printf("Keine sicheren Schwellen: Abstand zur Ruhe-Obergrenze %.1f dB, "
                  "mindestens %.1f dB erforderlich.\n", gap, MIN_GAP_DB);
    return;
  }

  Serial.printf("#define BAND_LOW_HZ   %d\n#define BAND_HIGH_HZ  %d\n", band.low, band.high);
  Serial.printf("#define LEVEL_HIGH    %.1ff  // EIN\n#define LEVEL_LOW     %.1ff  // AUS\n",
                quietMax + gap * 2.0f / 3.0f, quietMax + gap / 3.0f);
}

void runCalibration() {
  static bool running = false;
  static uint32_t startedAt;
  static int windowIndex;
  static float noiseFloor;
  static bool previousWasLoud;

  if (!running) {
    if (Serial.read() != 's') return;
    for (auto &window : windows) window = {};
    lateBlocks = clippedBlocks = 0;
    windowIndex = 0;
    noiseFloor = INFINITY;
    previousWasLoud = false;
    Serial.println("Messung laeuft: Wasser nach Belieben mehrmals auf- und abdrehen.");
    startedAt = millis();
    running = true;
  }

  const int index = (millis() - startedAt) / WINDOW_MS;
  if (index >= WINDOW_COUNT) {
    printCalibration();
    Serial.println("Fuer eine neue Messung s senden.");
    running = false;
    return;
  }
  if (index != windowIndex) {
    const auto &previous = windows[windowIndex];
    previousWasLoud = false;
    if (previous.blocks >= MIN_BLOCKS) {
      const float previousLevel = broadLevel(previous);
      noiseFloor = fminf(noiseFloor, previousLevel);
      previousWasLoud = previousLevel >= noiseFloor + GATE_DB;
    }
    windowIndex = index;
  }

  bool onTime = acquireBlock();
  if ((millis() - startedAt) / WINDOW_MS != (uint32_t)index) return;
  if (!onTime) {
    ++lateBlocks;
    return;
  }
  for (uint16_t sample : rawBuf) {
    if (sample <= 5 || sample >= 4090) {
      ++clippedBlocks;
      return;
    }
  }

  float level = prepareSamples();
  auto &window = windows[index];
  window.broadSum += level;
  ++window.blocks;
  if (!previousWasLoud || level < noiseFloor + GATE_DB) return;

  computeSpectrum();
  for (int band = 0; band < CANDIDATE_COUNT; ++band) {
    window.bandSums[band] += bandLevelDb(candidateBands[band]);
  }
  ++window.spectra;
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

// ------------------------------------------------------------ Networking
enum MsgType : uint8_t { MSG_FLOWING, MSG_ALARM, MSG_STOPPED };
static QueueHandle_t msgQueue = nullptr;

// Called from the main loop, never blocks
void notify(MsgType t) {
  if (msgQueue) xQueueSend(msgQueue, &t, 0);
}

static const char* msgText(MsgType t) {
  switch (t) {
    case MSG_ALARM:   return "Alarm";
    case MSG_STOPPED: return "Water stopped";
    default:          return "Water is flowing";
  }
}

static bool waitForWiFi(uint32_t timeoutMs) {
  // WiFi.begin and auto-reconnect own connection attempts.
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
    vTaskDelay(pdMS_TO_TICKS(250));
  }
  const auto status = WiFi.status();
  if (status == WL_CONNECTED) return true;
  Serial.printf("[net] WLAN-Verbindung fehlt (Status %d)\n", (int)status);
  return false;
}

static bool sendMessage(const char* text) {
  WiFiClient client;            // declared first, destroyed after http
  HTTPClient http;
  if (!http.begin(client, SERVER_URL)) return false;

  http.setTimeout(5000);
  http.addHeader("Content-Type", "text/plain");
  http.addHeader("X-Token", TOKEN);
  int code = http.POST((uint8_t*)text, strlen(text));
  http.end();

  Serial.printf("[net] \"%s\" -> HTTP %d\n", text, code);
  return code >= 200 && code < 300;
}

static void networkTask(void*) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  for (;;) {
    MsgType t;
    if (xQueueReceive(msgQueue, &t, portMAX_DELAY) != pdTRUE) continue;

    const char* text = msgText(t);
    for (int attempt = 0; attempt < 5; attempt++) {
      if (waitForWiFi(15000) && sendMessage(text)) break;
      Serial.println("[net] send failed, retrying");
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

// -------------------------------------------------------------- Detection
void updateDetection(uint32_t now) {
  if (!flowing) {
    if (smoothedDb > LEVEL_HIGH) {
      flowing = true;
      flowStartMs = now;
      lowSinceMs = 0;
      Serial.println(">> Flow detected, timer started");
      notify(MSG_FLOWING);
    }
  } else {
    if (smoothedDb < LEVEL_LOW) {
      if (lowSinceMs == 0) lowSinceMs = now;
      if (now - lowSinceMs > STOP_DEBOUNCE_MS) {
        flowing = false;
        alarmActive = false;
        Serial.println(">> Flow stopped, reset");
        notify(MSG_STOPPED);
      }
    } else {
      lowSinceMs = 0;   // still loud, cancel the stop timer
    }

    if (flowing && !alarmActive && (now - flowStartMs > FLOW_ALARM_MS)) {
      alarmActive = true;
      alarmStartMs = now;
      Serial.println("!! ALARM: water running too long");
      notify(MSG_ALARM);
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

  setupSpectrum();

  Serial.println();
  Serial.println(CALIBRATION_MODE ? "Water flow detector (MAX9814): CALIBRATION MODE"
                                  : "Water flow detector (MAX9814): RUNNING");

  if (CALIBRATION_MODE) {
    Serial.println("s senden, um die automatische 3-Minuten-Messung zu starten.");
  } else {
    msgQueue = xQueueCreate(8, sizeof(MsgType));
    xTaskCreate(networkTask, "net", 8192, nullptr, 1, nullptr);
  }
}

// ------------------------------------------------------------------- Loop
void loop() {
  if (CALIBRATION_MODE) {
    runCalibration();
    delay(1);  // Let the idle task run between ADC blocks.
    return;
  }

  acquireBlock();
  prepareSamples();
  computeSpectrum();
  float levelDb = bandLevelDb(operatingBand);

  // Exponential smoothing in the dB domain
  if (smoothedDb < -89.0f) smoothedDb = levelDb;   // quick init
  smoothedDb = SMOOTHING * smoothedDb + (1.0f - SMOOTHING) * levelDb;

  uint32_t now = millis();

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
