#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Servo.h>

// Constants
#define NUM_FLEX_SENSORS 5
#define SAMPLE_RATE SAMPLE_FREQ_1000HZ
#define HUM_FREQ NOTCH_FREQ_50HZ

const int calibrationDuration = 10000;

const int sensorPins[NUM_FLEX_SENSORS] = { A0, A1, A2, A3, A4 }; // A0 is thumb
// Servo pins
const int thumbServoPin = 3;
const int fingersServoPin = 4;

// Servo objects
Servo thumbServo;
Servo fingersServo;

// Servo state
int thumbAngle = 0;
int fingersAngle = 0;

// Limits
const int THUMB_MIN = 0;
const int THUMB_MAX = 180;

const int FINGERS_MIN = 0;
const int FINGERS_MAX = 180;

// Angle step per trigger
const int thumbStep   = 45;
const int fingersStep = 45;


// Globals
const int forceThumbPin = A5;
const int forceIndexPin = A6;
long baselineForceThumb = 0;
long baselineForceIndex = 0;
bool isCalibrated = false;
unsigned long calibrationStartTime = 0;
int sampleCount = 0;
int sensorValues[NUM_FLEX_SENSORS];
long baselineFlexSensorValues[NUM_FLEX_SENSORS] = { 0, 0, 0, 0, 0 };


// For flex/relax detection
bool fingerState[NUM_FLEX_SENSORS] = { false, false, false, false, false };
const int FORCE_THRESHOLD = 0;
const int FLEX_THRESHOLD = 100;
const int RELAX_THRESHOLD = 20;
const unsigned long TRIGGER_COOLDOWN = 300;
unsigned long lastTriggerTime[NUM_FLEX_SENSORS] = { 0, 0, 0, 0, 0 };

// Prototypes
typedef void (*MoveFn)(bool tighten);
void calibrateSensors();
void readFlexSensors();
int moveServo(Servo &servo, int &currentAngle, int delta, int minAngle, int maxAngle, bool tighten);
void moveThumb(bool tighten);
void moveFingers(bool tighten);
void setupMotors();

// Setup
void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(forceThumbPin, INPUT);
  pinMode(forceIndexPin, INPUT);

  setupMotors();
  calibrationStartTime = millis();
  Serial.println("Thumb Index Middle Ring Pinky");
}

// Loop constantly running
void loop() {
  if (!isCalibrated) {
    calibrateSensors();
    return;
  }

  readFlexSensors();
}

// Initial calibration
void calibrateSensors() {
  unsigned long currentTime = millis();
  int timeRemaining = (calibrationDuration - (currentTime - calibrationStartTime)) / 1000;

  if (currentTime - calibrationStartTime < calibrationDuration) {
    sampleCount++;

    for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
      sensorValues[i] = analogRead(sensorPins[i]);
      baselineFlexSensorValues[i] += sensorValues[i];
    }

    baselineForceThumb += analogRead(forceThumbPin);
    baselineForceIndex += analogRead(forceIndexPin);

    static int lastPrintedSecond = -1;
    if (timeRemaining != lastPrintedSecond) {
      Serial.print("Calibrating... Time left: ");
      Serial.print(timeRemaining);
      Serial.println(" sec");
      lastPrintedSecond = timeRemaining;
    }

  } else {
    for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
      baselineFlexSensorValues[i] /= sampleCount;
    }
    baselineForceThumb /= sampleCount;
    baselineForceIndex /= sampleCount;
    Serial.println("Calibration complete!");
    isCalibrated = true;
  }
}

// Initial motor setup
void setupMotors() {
  thumbServo.attach(thumbServoPin);
  fingersServo.attach(fingersServoPin);

  thumbServo.write(thumbAngle);
  fingersServo.write(fingersAngle);
}

void moveThumb(bool tighten) {
  thumbAngle = moveServo(thumbServo, thumbAngle, thumbStep, THUMB_MIN, THUMB_MAX, tighten);
}

void moveFingers(bool tighten) {
  fingersAngle = moveServo(fingersServo, fingersAngle, fingersStep, FINGERS_MIN, FINGERS_MAX, tighten);
}



// Flex sensor reading
void readFlexSensors() {
  unsigned long now = millis();
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  int forceThumbValue = analogRead(forceThumbPin);
  int forceIndexValue = analogRead(forceIndexPin);

  // For force sensors any nonzero reading above baseline indicates contact
  bool forceThumbTriggered = (forceThumbValue - baselineForceThumb) > FORCE_THRESHOLD;
  bool forceIndexTriggered = (forceIndexValue - baselineForceIndex) > FORCE_THRESHOLD;

  bool pinkyTriggered = (baselineFlexSensorValues[4] - sensorValues[4]) > FLEX_THRESHOLD;
  bool ringTriggered = (baselineFlexSensorValues[3] - sensorValues[3]) > FLEX_THRESHOLD;
  bool middleTriggered = (baselineFlexSensorValues[2] - sensorValues[2]) > FLEX_THRESHOLD;
  bool indexTriggered = (baselineFlexSensorValues[1] - sensorValues[1]) > FLEX_THRESHOLD;
  bool thumbTriggered = (baselineFlexSensorValues[0] - sensorValues[0]) > FLEX_THRESHOLD;

  // Print readings for debugging
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Plot force sensors
  /*Serial.print(forceThumbValue);
  Serial.print(" and ");
  Serial.print(forceIndexValue);
  Serial.println();*/

  // Handle each finger group with state tracking
  auto processFinger = [&](bool triggered, int idx, MoveFn moveFn) {
    if (triggered && !fingerState[idx] && (now - lastTriggerTime[idx] > TRIGGER_COOLDOWN)) {
      fingerState[idx] = true;
      moveFn(true);  // tighten
      lastTriggerTime[idx] = now;

      Serial.print("Finger ");
      Serial.print(idx);
      Serial.println(" flexed");

    } else if (!triggered && fingerState[idx] && (now - lastTriggerTime[idx] > TRIGGER_COOLDOWN)) {
      int diff = baselineFlexSensorValues[idx] - sensorValues[idx];
      if (diff < RELAX_THRESHOLD) {
        fingerState[idx] = false;
        moveFn(false); // loosen
        lastTriggerTime[idx] = now;

        Serial.print("Finger ");
        Serial.print(idx);
        Serial.println(" relaxed");
      }
    }
  };


  // Map groups to motors
  // Thumb
  processFinger(thumbTriggered/* && forceThumbTriggered*/, 0, moveThumb);

  // Fingers
  processFinger(indexTriggered || middleTriggered || ringTriggered || pinkyTriggered/* && forceIndexTriggered*/, 1, moveFingers);

}

// motor control
int moveServo(Servo &servoToMove, int &currentAngle, int delta, int minAngle, int maxAngle, bool tighten) {
  if (tighten) {
    currentAngle += delta;
  } else {
    currentAngle -= delta;
  }

  currentAngle = constrain(currentAngle, minAngle, maxAngle);
  servoToMove.write(currentAngle);
  return currentAngle;
}

