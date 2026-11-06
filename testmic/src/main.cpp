#include <Arduino.h>
#include <math.h>

namespace {

constexpr uint8_t MIC_PIN = 4;
constexpr uint32_t SAMPLE_RATE_HZ = 8000;
constexpr size_t SAMPLE_COUNT = 512;
constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;

uint16_t samples[SAMPLE_COUNT];

void readSampleBlock() {
  uint32_t nextSampleUs = micros();

  for (size_t i = 0; i < SAMPLE_COUNT; ++i) {
    while (static_cast<int32_t>(micros() - nextSampleUs) < 0) {
    }
    samples[i] = analogRead(MIC_PIN);
    nextSampleUs += SAMPLE_PERIOD_US;
  }
}

void printSampleStats() {
  uint16_t minimum = 4095;
  uint16_t maximum = 0;
  double sum = 0;

  for (uint16_t sample : samples) {
    minimum = min(minimum, sample);
    maximum = max(maximum, sample);
    sum += sample;
  }

  const double mean = sum / SAMPLE_COUNT;
  double squaredDeviationSum = 0;
  for (uint16_t sample : samples) {
    const double deviation = sample - mean;
    squaredDeviationSum += deviation * deviation;
  }

  const double rms = sqrt(squaredDeviationSum / SAMPLE_COUNT);
  Serial.printf("DC=%4.0f  min=%4u  max=%4u  p-p=%4u  AC RMS=%.1f ADC counts%s\n",
                mean, minimum, maximum, maximum - minimum, rms,
                (minimum <= 5 || maximum >= 4090) ? "  [clipping?]" : "");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);

  Serial.println();
  Serial.println("MAX9814 microphone level test");
  Serial.printf("ADC input: GPIO %u; sampling at %lu Hz\n",
                MIC_PIN, static_cast<unsigned long>(SAMPLE_RATE_HZ));
  Serial.println("Speak, tap near the microphone, or make other sounds to see levels change.");
}

void loop() {
  readSampleBlock();
  printSampleStats();
}
