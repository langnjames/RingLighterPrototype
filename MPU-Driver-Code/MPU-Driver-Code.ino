/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

// Vars for intital raw values
int16_t gx, gy, gz, ax, ay, az;

#define OUTPUT_READABLE_ACCELGYRO

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define USE_GYRO_ACCEL 6
// #define LED_PIN 4
// #define BUTTON_PIN

// Change this to be the correct button pinout when we know which one it will be
const int buttonPin = 3;
const int ledPin = 4;

// Number of readings that are averaged out to become one data point (Scale w/ available dynamic memory allocation)
const int numReadings = 25;

const int numAxis = USE_GYRO_ACCEL;
const int AY = 0;
const int GX = 1;
const int GY = 2;
const int GZ = 3;


int32_t readings[numAxis][numReadings];  // the reading history
int32_t readIndex[numAxis];              // the index of the current reading
int32_t total[numAxis];                  // the running total
int32_t average[numAxis];                // the average

// Define the timeout duration in milliseconds
const unsigned long positionTimeout = 250;  // 500 milliseconds

unsigned long positionTimerStart = 0;


void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // chose standard baud rate for MPU6050
  Serial.begin(115200);

  // initialize device
  DEBUG_PRINTLN("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  DEBUG_PRINTLN("Testing device connections...");
  DEBUG_PRINTLN(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Gyroscope offsets to calibrate the gyro to start as close to zero as possible
  accelgyro.setXGyroOffset(-100);  // Adjust as needed
  accelgyro.setYGyroOffset(1680);  // Adjust as needed
  accelgyro.setZGyroOffset(675);   // Adjust as needed

  // configure Arduino LED for
  pinMode(ledPin, OUTPUT);

  // configure Arduino BTN for
  pinMode(buttonPin, INPUT);


  // zero-fill all the arrays:
  for (int axis = 0; axis < numAxis; axis++) {
    readIndex[axis] = 0;
    total[axis] = 0;
    average[axis] = 0;
    for (int i = 0; i < numReadings; i++) {
      readings[axis][i] = 0;
    }
  }
}


void loop() {
  // read raw gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calls smoothing function for each necessary axis
  smooth(AY, ay);
  smooth(GX, gx);
  smooth(GY, gy);
  smooth(GZ, gz);

  if (inCorrectPosition()) {
    if (digitalRead(buttonPin) == HIGH) {
      // If the button is pressed and the device is in the correct position
      // Turn on the LED and start/restart the timer
      if (millis() - positionTimerStart >= positionTimeout) {
        digitalWrite(ledPin, HIGH);
        Serial.println("\tLED ON");
      }
    } else {
      // If the button is not pressed, turn off the LED
      digitalWrite(ledPin, LOW);
      Serial.println("\tLED OFF");
      // Reset the timer when the device goes out of the correct position
      positionTimerStart = millis();
    }
  } else {
    // If the device is not in the correct position, turn off the LED
    digitalWrite(ledPin, LOW);
    Serial.println("\tLED OFF");
    // Reset the timer when the device goes out of the correct position
    positionTimerStart = millis();
  }

// //DEBUGGING STATEMENTS
#ifdef OUTPUT_READABLE_ACCELGYRO
  // display tab-separated gyro x/y/z values
  DEBUG_PRINT("GX:");
  DEBUG_PRINT(average[GX]);
  DEBUG_PRINT("\t");

  DEBUG_PRINT("GY:");
  DEBUG_PRINT(average[GY]);
  DEBUG_PRINT("\t");

  DEBUG_PRINT("GZ:");
  DEBUG_PRINT(average[GZ]);
  DEBUG_PRINT("\t");

  DEBUG_PRINT("AY:");
  DEBUG_PRINT(average[AY]);
  DEBUG_PRINT("\t");
#endif
}

// Bool method to make conditional statement easier to read
// Values based on sensitivity and height of MPU in relation to arm at side. Acute elbow angle
bool inCorrectPosition() {
  return (average[GY] >= -5000 && average[GY] <= 5000 
  && average[GX] >= -5000 && average[GX] <= 5000 
  && average[GZ] >= -5000 && average[GZ] <= 5000
  && average[AY] >= 5000);
}

// Method for smoothing out raw gyro/accel values and averaging them over a given number of readings
void smooth(int axis, int32_t val) {
  // pop and subtract the last reading:
  total[axis] -= readings[axis][readIndex[axis]];
  total[axis] += val;

  // add value to running total
  readings[axis][readIndex[axis]] = val;
  readIndex[axis]++;

  if (readIndex[axis] >= numReadings)
    readIndex[axis] = 0;

  // calculate the average:
  average[axis] = total[axis] / numReadings;
}
