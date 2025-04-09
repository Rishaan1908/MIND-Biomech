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
const int stepsPerRevolution = 200;
const int stepDelay = 1000;            // Delay between steps (microseconds)
// Motor-specific step settings
const int stepsThumb = 200;
const int stepsIndex = 400;
const int stepsFingers = 450;


// Pin definitions
const int sensorPins[NUM_FLEX_SENSORS] = {A0, A1, A2, A3, A4};
const int emgInputPin = A5;
const int stepPinThumb = 9, dirPinThumb = 8;
const int stepPinPoint = 3, dirPinPoint = 2;
const int stepPinFingers = 5, dirPinFingers = 4;

bool thumbMotor = false;
bool indexMotor = false;
bool fingersMotor = false;

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

  readFlexSensors(envelope);

  delayMicroseconds(100);  // Delay for smoother reading
}

// --- Calibration Functions ---
void calibrateSensors() {
  unsigned long currentTime = millis();
  int timeRemaining = (calibrationDuration - (currentTime - calibrationStartTime)) / 1000;

  if (currentTime - calibrationStartTime < calibrationDuration) {
    int emgValue = analogRead(emgInputPin);
    int filteredEmgValue = emgFilter.update(emgValue);
    totalEmgValue += filteredEmgValue;
    sampleCount++;

    // Accumulate sensor values for baseline calibration
    for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
      sensorValues[i] = analogRead(sensorPins[i]);
      baselineFlexSensorValues[i] += sensorValues[i];
    }

    // Print only once per second
    static int lastPrintedSecond = -1;
    if (timeRemaining != lastPrintedSecond) {
      Serial.print("EMG and Flex sensors calibration in process: Please do not move your hand or fingers. ");
      Serial.print("Time remaining: ");
      Serial.print(timeRemaining);
      Serial.println(" seconds");
      lastPrintedSecond = timeRemaining;
    }

  } else {
    // Calibration done, calculate baselines
    baselineEmgValue = totalEmgValue / sampleCount;
    for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
      baselineFlexSensorValues[i] /= sampleCount;
    }

    Serial.println("Calibration Complete.");
    Serial.print("Baseline EMG Value: ");
    Serial.println(baselineEmgValue);
    Serial.println("Baseline Flex Sensor Values:");
    for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(baselineFlexSensorValues[i]);
    }

    isCalibrated = true;
  }
}

// --- Flex Sensor Functions ---
void readFlexSensors(int emg_value) {
  // Step 1: Read each flex sensor and store value
  sensorValues[0] = analogRead(sensorPins[0]); // Pinky
  sensorValues[1] = analogRead(sensorPins[1]); // Ring
  sensorValues[2] = analogRead(sensorPins[2]); // Middle
  sensorValues[3] = analogRead(sensorPins[3]); // Index
  sensorValues[4] = analogRead(sensorPins[4]); // Thumb

  // Step 2: Check which fingers are triggered
  bool pinkyTriggered  = (baselineFlexSensorValues[0] - sensorValues[0] > 100);
  bool ringTriggered   = (baselineFlexSensorValues[1] - sensorValues[1] > 100);
  bool middleTriggered = (baselineFlexSensorValues[2] - sensorValues[2] > 100);
  bool indexTriggered  = (baselineFlexSensorValues[3] - sensorValues[3] > 100);
  bool thumbTriggered  = (baselineFlexSensorValues[4] - sensorValues[4] > 100);

  // Step 3: Debug print
  Serial.print("Pinky: "); Serial.print(sensorValues[0]); Serial.print("   ");
  Serial.print("Ring: "); Serial.print(sensorValues[1]); Serial.print("   ");
  Serial.print("Middle: "); Serial.print(sensorValues[2]); Serial.print("   ");
  Serial.print("Index: "); Serial.print(sensorValues[3]); Serial.print("   ");
  Serial.print("Thumb: "); Serial.println(sensorValues[4]);

  // Step 4: Prioritized logic
  if (thumbTriggered && indexTriggered && middleTriggered && ringTriggered && pinkyTriggered) {
    if (!thumbMotor && !indexMotor && !fingersMotor) {
      Serial.println("All fingers flexed. Moving all motors.");
      tightenMotors(true, true, true);
      delay(1000);
      loosenMotors(true, true, true);
    }
  } 
  else if (thumbTriggered && indexTriggered) {
    if (!thumbMotor && !indexMotor) {
      Serial.println("Thumb + Index triggered.");
      tightenMotors(true, true, false);
      delay(1000);
      loosenMotors(true, true, false);
    }
  } 
  else if (indexTriggered && (middleTriggered || ringTriggered || pinkyTriggered)) {
    if (!indexMotor && !fingersMotor) {
      Serial.println("Index + Fingers triggered.");
      tightenMotors(false, true, true);
      delay(1000);
      loosenMotors(false, true, true);
    }
  } 
  else {
    // Individual motor movement
    if (indexTriggered && !indexMotor && !thumbMotor && !fingersMotor ) {
      Serial.println("All fingers flexed. Moving all motors.");
      tightenMotors(true, true, true);
      delay(5000);
      loosenMotors(true, true, true);
      /*
      Serial.println("Index only triggered.");
      tightenMotors(false, true, false);
      delay(1000);
      loosenMotors(false, true, false);
      */
    }

    if ((middleTriggered || ringTriggered || pinkyTriggered) && !fingersMotor) {
      Serial.println("Fingers only triggered.");
      tightenMotors(false, false, true);
      delay(1000);
      loosenMotors(false, false, true);
    }

    if (thumbTriggered && !thumbMotor) {
      Serial.println("Thumb only triggered.");
      tightenMotors(true, false, false);
      delay(1000);
      loosenMotors(true, false, false);
    }
  }
}

// --- Motor Control Functions ---
void tightenMotors(bool thumb, bool point, bool fingers) {
  Serial.println("Tightening");
  digitalWrite(dirPinThumb, LOW);
  digitalWrite(dirPinPoint, LOW);
  digitalWrite(dirPinFingers, HIGH);

  for (int x = 0; x < max(max(stepsThumb, stepsIndex), stepsFingers); x++) {
    if (thumb && x < stepsThumb) digitalWrite(stepPinThumb, HIGH);
    if (point && x < stepsIndex) digitalWrite(stepPinPoint, HIGH);
    if (fingers && x < stepsFingers) digitalWrite(stepPinFingers, HIGH);

    delayMicroseconds(stepDelay);

    if (thumb && x < stepsThumb) digitalWrite(stepPinThumb, LOW);
    if (point && x < stepsIndex) digitalWrite(stepPinPoint, LOW);
    if (fingers && x < stepsFingers) digitalWrite(stepPinFingers, LOW);

    delayMicroseconds(stepDelay);
  }

  if (point) indexMotor = true;
  if (thumb) thumbMotor = true;
  if (fingers) fingersMotor = true;
}


void loosenMotors(bool thumb, bool point, bool fingers) {
  digitalWrite(dirPinThumb, HIGH);
  digitalWrite(dirPinPoint, HIGH);
  digitalWrite(dirPinFingers, LOW);

  for (int x = 0; x < max(max(stepsThumb, stepsIndex), stepsFingers); x++) {
    if (thumb && x < stepsThumb) digitalWrite(stepPinThumb, HIGH);
    if (point && x < stepsIndex) digitalWrite(stepPinPoint, HIGH);
    if (fingers && x < stepsFingers) digitalWrite(stepPinFingers, HIGH);

    delayMicroseconds(stepDelay);

    if (thumb && x < stepsThumb) digitalWrite(stepPinThumb, LOW);
    if (point && x < stepsIndex) digitalWrite(stepPinPoint, LOW);
    if (fingers && x < stepsFingers) digitalWrite(stepPinFingers, LOW);

    delayMicroseconds(stepDelay);
  }

  if (point) indexMotor = false;
  if (thumb) thumbMotor = false;
  if (fingers) fingersMotor = false;
}


void stopMotors() {
  digitalWrite(stepPinThumb, LOW);
  digitalWrite(stepPinPoint, LOW);
  digitalWrite(stepPinFingers, LOW);
}
