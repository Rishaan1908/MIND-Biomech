#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "EMGFilters.h"

// Define constants
#define TIMING_DEBUG 1
#define NUM_FLEX_SENSORS 5
#define SAMPLE_RATE SAMPLE_FREQ_1000HZ // EMG filter supports 500Hz or 1000Hz
#define HUM_FREQ NOTCH_FREQ_50HZ       // Power line interference frequency

// Calibration settings
const int calibrationDuration = 10000; // 10 seconds in milliseconds
static const int threshold = 0;        // EMG calibration threshold

// Motor and stepper settings
const int stepsPerRevolution = 1000;
const int stepDelay = 1000;            // Delay between steps (microseconds)

// Pin definitions
const int sensorPins[NUM_FLEX_SENSORS] = {A0, A1, A2, A3, A4};
const int emgInputPin = A5;
const int stepPinThumb = 9, dirPinThumb = 8;
const int stepPinPoint = 3, dirPinPoint = 2;
const int stepPinFingers = 5, dirPinFingers = 4;

// Global variables
EMGFilters emgFilter;
int baselineEmgValue = 0;
bool isCalibrated = false;
unsigned long calibrationStartTime = 0;
int totalEmgValue = 0;
int sampleCount = 0;
int sensorValues[NUM_FLEX_SENSORS];
long baselineFlexSensorValues[NUM_FLEX_SENSORS] = {0, 0, 0, 0, 0};

// Function Prototypes
void calibrateSensors();
void readFlexSensors();
void tightenMotors(bool thumb, bool point, bool fingers);
void loosenMotors(bool thumb, bool point, bool fingers);
void stopMotors();

void setup() {
  Serial.begin(9600);
  
  // Initialize EMG Filter
  emgFilter.init(SAMPLE_RATE, HUM_FREQ, true, true, true);

  // Flex sensor pin setup
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Motor pin setup
  pinMode(stepPinThumb, OUTPUT);
  pinMode(dirPinThumb, OUTPUT);
  pinMode(stepPinPoint, OUTPUT);
  pinMode(dirPinPoint, OUTPUT);
  pinMode(stepPinFingers, OUTPUT);
  pinMode(dirPinFingers, OUTPUT);

  // Start calibration
  calibrationStartTime = millis();
}

void loop() {
  if (!isCalibrated) {
    calibrateSensors();
    return;
  }

  // Regular EMG processing after calibration
  unsigned long timeStamp = micros();
  int emgValue = analogRead(emgInputPin);
  int filteredEmgValue = emgFilter.update(emgValue);
  int envelope = sq(filteredEmgValue) - baselineEmgValue;
  envelope = (envelope > threshold) ? envelope : 0;
  
  if (TIMING_DEBUG) {
    Serial.println(envelope);
  }

  readFlexSensors();

  delayMicroseconds(10000);  // Delay for smoother reading
}

// --- Calibration Functions ---

void calibrateSensors() {
  unsigned long currentTime = millis();
  if (currentTime - calibrationStartTime < calibrationDuration) {
    int emgValue = analogRead(emgInputPin);
    int filteredEmgValue = emgFilter.update(emgValue);
    totalEmgValue += filteredEmgValue;
    sampleCount++;

    // Accumulate sensor values for baseline calibration
    for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
      sensorValues[i] = analogRead(sensorPins[i]);
      Serial.println(sensorValues[i]);
      baselineFlexSensorValues[i] += sensorValues[i];
    }
  } else {
    // Calibration done, calculate baselines
    baselineEmgValue = totalEmgValue / sampleCount;
    for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
      Serial.println(baselineFlexSensorValues[i]);
      baselineFlexSensorValues[i] /= sampleCount;
    }
    Serial.print("Sample Count");
    Serial.println(sampleCount);
    
    isCalibrated = true;
    Serial.println("Calibration Complete. Baseline EMG Value: ");
    Serial.println(baselineEmgValue);

    for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
      Serial.println(baselineFlexSensorValues[i]);
    }
  }
}

// --- Flex Sensor Functions ---

void readFlexSensors() {
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);

    // Print sensor values with names
    String sensorName;
    switch (i) {
      case 0: sensorName = "Pinky"; break;
      case 1: sensorName = "Ring"; break;
      case 2: sensorName = "Middle"; break;
      case 3: sensorName = "Index"; break;
      case 4: sensorName = "Thumb"; break;
    }
    Serial.print(sensorName + ": ");
    Serial.print(sensorValues[i]);
    Serial.print("   ");
  }
  Serial.println();
}

// --- Motor Control Functions ---

void tightenMotors(bool thumb, bool point, bool fingers) {
  digitalWrite(dirPinThumb, LOW);
  digitalWrite(dirPinPoint, LOW);
  digitalWrite(dirPinFingers, HIGH);
  
  for (int x = 0; x < stepsPerRevolution; x++) {
    if (thumb) digitalWrite(stepPinThumb, HIGH);
    if (point) digitalWrite(stepPinPoint, HIGH);
    if (fingers) digitalWrite(stepPinFingers, HIGH);
    
    delayMicroseconds(stepDelay);
    
    if (thumb) digitalWrite(stepPinThumb, LOW);
    if (point) digitalWrite(stepPinPoint, LOW);
    if (fingers) digitalWrite(stepPinFingers, LOW);
    
    delayMicroseconds(stepDelay);
  }
}

void loosenMotors(bool thumb, bool point, bool fingers) {
  digitalWrite(dirPinThumb, HIGH);
  digitalWrite(dirPinPoint, HIGH);
  digitalWrite(dirPinFingers, LOW);
  
  for (int x = 0; x < stepsPerRevolution; x++) {
    if (thumb) digitalWrite(stepPinThumb, HIGH);
    if (point) digitalWrite(stepPinPoint, HIGH);
    if (fingers) digitalWrite(stepPinFingers, HIGH);
    
    delayMicroseconds(stepDelay);
    
    if (thumb) digitalWrite(stepPinThumb, LOW);
    if (point) digitalWrite(stepPinPoint, LOW);
    if (fingers) digitalWrite(stepPinFingers, LOW);
    
    delayMicroseconds(stepDelay);
  }
}

void stopMotors() {
  digitalWrite(stepPinThumb, LOW);
  digitalWrite(stepPinPoint, LOW);
  digitalWrite(stepPinFingers, LOW);
}
