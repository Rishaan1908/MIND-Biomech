
	/*
* Copyright 2017, OYMotion Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
* THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "EMGFilters.h"

#define TIMING_DEBUG 1

#define SensorInputPin1 A0 // input pin number for sensor 1
#define SensorInputPin2 A1 // input pin number for sensor 2

EMGFilters myFilter1;
EMGFilters myFilter2;

// discrete filters must work with fixed sample frequency
// our emg filter only supports SAMPLE_FREQ_500HZ or SAMPLE_FREQ_1000HZ
int sampleRate = SAMPLE_FREQ_1000HZ;

// power line interference frequency: NOTCH_FREQ_50HZ or NOTCH_FREQ_60HZ
int humFreq = NOTCH_FREQ_50HZ;

// Calibration threshold
static int Threshold = 0;

unsigned long timeStamp;
unsigned long timeBudget;

void setup() {
  myFilter1.init(sampleRate, humFreq, true, true, true);
  myFilter2.init(sampleRate, humFreq, true, true, true);
  
  Serial.begin(9600);
  
  timeBudget = 1e6 / sampleRate;
}

void loop() {
  timeStamp = micros();
  
  int Value1 = analogRead(SensorInputPin1);
  int Value2 = analogRead(SensorInputPin2);
  
  int DataAfterFilter1 = myFilter1.update(Value1);
  int DataAfterFilter2 = myFilter2.update(Value2);
  
  int envlope1 = sq(DataAfterFilter1);
  int envlope2 = sq(DataAfterFilter2);
  
  envlope1 = (envlope1 > Threshold) ? envlope1 : 0;
  envlope2 = (envlope2 > Threshold) ? envlope2 : 0;
  
  timeStamp = micros() - timeStamp;
  if (TIMING_DEBUG) {
    Serial.print(envlope1);
    Serial.print(",");
    Serial.println(envlope2);
  }
  
  delayMicroseconds(10000);
}
