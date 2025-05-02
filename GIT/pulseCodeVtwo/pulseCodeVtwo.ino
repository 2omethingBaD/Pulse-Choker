#include <MAX3010x.h>
#include "filters.h"

#define led_pin 10

MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_100SPS;
const float kSamplingFrequency = 100.0;

// Finger Detection (IR strength)
const unsigned long kFingerThreshold = 2000;
const unsigned int kFingerCooldownMs = 500;

// Heartbeat Detection
const float kEdgeThreshold = -500.0;
const float kLowPassCutoff = 3.0;
const float kHighPassCutoff = 0.5;

// Advanced Motion Noise Protection
const float kMaxSlopeThreshold = -5000.0; // ignore huge drops from talking
const int kMinBPM = 50;
const int kMaxBPM = 200;

// Averaging
const bool kEnableAveraging = true;
const int kAveragingSamples = 8;
const int kSampleThreshold = 5;

// Filters
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter(kLowPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager;

long last_heartbeat = 0;
long finger_timestamp = 0;
bool finger_detected = false;

float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

// Heartbeat LED pulse states
enum PulseStage { NONE, LUB, DUB, FADE };
PulseStage pulse_stage = NONE;
unsigned long pulse_timer = 0;
int fade_value = 0;

// BPM sync
int synced_bpm = 75;
int lub_duration = 80;
int dub_duration = 60;
int fade_interval = 15;
int fade_step = 15;


void setup() 
{
  Serial.begin(9600);

  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) 
  { 
    Serial.println("MAX30102 initialized");
  }else 
  {
    Serial.println("Sensor not found");  
    while(1);
  }

  pinMode(led_pin, OUTPUT);
  analogWrite(led_pin, 0);
}


void loop() 
{
  unsigned long now = millis();

  // LED animation
  if (pulse_stage != NONE) 
  {
    switch (pulse_stage) 
    {
      case LUB:
        analogWrite(led_pin, 255);
        pulse_timer = now + lub_duration;
        pulse_stage = DUB;
        break;

      case DUB:
        if (now >= pulse_timer) 
        {
          analogWrite(led_pin, 150);
          pulse_timer = now + dub_duration;
          pulse_stage = FADE;
        }
        break;

      case FADE:
        if (now >= pulse_timer) 
        {
          fade_value -= fade_step;
          fade_value = constrain(fade_value, 0, 255);
          analogWrite(led_pin, fade_value);

          if (fade_value == 0) 
          {
            pulse_stage = NONE;
          }else 
          {
            pulse_timer = now + fade_interval;
          }
        }
        break;

      default:
        break;
    }
  }

  // Read IR signal
  auto sample = sensor.readSample(1000);
  float current_value = sample.ir;

  if (current_value == 0) return;

  // Finger/neck contact detection
  if (current_value > kFingerThreshold) 
  {
    if (!finger_detected && millis() - finger_timestamp > kFingerCooldownMs) 
    {
      finger_detected = true;
      Serial.println("Neck contact detected.");
    }
  }else 
  {
    if (finger_detected) 
    {
      Serial.println("Sensor contact lost.");
    }

    finger_detected = false;
    finger_timestamp = millis();

    differentiator.reset();
    averager.reset();
    low_pass_filter.reset();
    high_pass_filter.reset();
    last_diff = NAN;
  }

  if (finger_detected) 
  {
    current_value = low_pass_filter.process(current_value);
    current_value = high_pass_filter.process(current_value);
    float current_diff = differentiator.process(current_value);

    // Skip noisy spikes from talking/moving
    if (current_diff < kMaxSlopeThreshold) return;

    if (!isnan(current_diff) && !isnan(last_diff)) 
    {
      if (last_diff > 0 && current_diff < 0) 
      {
        crossed = true;
        crossed_time = millis();
      }

      if (current_diff > 0) 
      {
        crossed = false;
      }

      if (crossed && current_diff < kEdgeThreshold) 
      {
        int beat_interval = crossed_time - last_heartbeat;
        if (last_heartbeat != 0 && beat_interval > 300) 
        {
          int bpm = 60000 / beat_interval;
          if (bpm >= kMinBPM && bpm <= kMaxBPM) 
          {
            synced_bpm = bpm;

            // Scale LED timing to BPM
            float beat_time = 60000.0 / bpm;
            lub_duration = beat_time * 0.1;
            dub_duration = beat_time * 0.08;
            fade_interval = beat_time * 0.012;
            fade_step = max(5, 255 / (beat_time / fade_interval));

            pulse_stage = LUB;
            pulse_timer = millis();
            fade_value = 150;

            // BPM display
            if (kEnableAveraging) 
            {
              int average_bpm = averager.process(bpm);
              if (averager.count() >= kSampleThreshold) 
              {
                Serial.print("Heart Rate (avg, bpm): ");
                Serial.println(average_bpm);
              }
            }else 
            {
              Serial.print("Heart Rate (current, bpm): ");
              Serial.println(bpm);  
            }
          }
        }

        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
  }
}