#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "EMGFilters.h"

#define TIMING_DEBUG 1

#define SensorInputPin1 A5 // input pin number for sensor 1
#define NUM_FLEX_SENSORS 5


EMGFilters myFilter1;

// discrete filters must work with fixed sample frequency
// our emg filter only supports SAMPLE_FREQ_500HZ or SAMPLE_FREQ_1000HZ
int sampleRate = SAMPLE_FREQ_1000HZ;

// power line interference frequency: NOTCH_FREQ_50HZ or NOTCH_FREQ_60HZ
int humFreq = NOTCH_FREQ_50HZ;

// Calibration threshold
static int Threshold = 0;

// Baseline variables
int baselineValue = 0;
bool isCalibrated = false; // Flag to check if calibration is done
unsigned long calibrationStartTime = 0; // Stores the start time for calibration
const unsigned long calibrationDuration = 10000; // 10 seconds in milliseconds
int sampleCount = 0; // Count the number of samples taken during calibration
long totalValue = 0; // Accumulate the total value during calibration

unsigned long timeStamp;
unsigned long timeBudget;

// flex sensor stuff


const int sensorPins[NUM_FLEX_SENSORS] = {A0, A1, A2, A3, A4};
int sensorValues[NUM_FLEX_SENSORS];


void setup() {
  myFilter1.init(sampleRate, humFreq, true, true, true);

  Serial.begin(9600);

  timeBudget = 1e6 / sampleRate;

  // flex sensor set up.
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Start calibration
  calibrationStartTime = millis();
}

void loop() {
  // Check if the system is still in calibration mode (first 10 seconds)
  if (!isCalibrated) {
    unsigned long currentTime = millis();
    
    // During calibration (within the first 10 seconds)
    if (currentTime - calibrationStartTime < calibrationDuration) {
      int Value1 = analogRead(SensorInputPin1); // Read EMG sensor value
      int DataAfterFilter1 = myFilter1.update(Value1); // Filter the signal
      totalValue += DataAfterFilter1; // Accumulate the filtered value
      sampleCount++; // Increment the sample count
    } else {
      // Calibration period has ended
      baselineValue = totalValue / sampleCount; // Calculate the average baseline value
      isCalibrated = true; // Set flag to indicate calibration is done
      Serial.print("Baseline calculated: ");
      Serial.println(baselineValue); // Output the baseline value
    }
    return;
  }

  // Regular EMG processing after calibration
  timeStamp = micros();
  int Value1 = analogRead(SensorInputPin1);
  int DataAfterFilter1 = myFilter1.update(Value1);
  int envlope1 = sq(DataAfterFilter1);

  // Adjust the envelope relative to the baseline
  envlope1 = envlope1 - baselineValue;
  envlope1 = (envlope1 > Threshold) ? envlope1 : 0;

  timeStamp = micros() - timeStamp;
  if (TIMING_DEBUG) {
    Serial.println(envlope1);
  }

  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensorValues[i]);
    Serial.print("   ");
  }

  delayMicroseconds(10000);
}
