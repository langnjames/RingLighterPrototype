/* Servo motor with Arduino example code. Position and sweep. More info: https://www.makerguides.com/ */

// Include the servo library:
#include <Servo.h>

// Create a new servo object:
Servo myservo;

// Define the servo pin:
#define servoPin 9

const int sparkPin = 8;
// const unsigned long interval = 10000;  // Set the interval in milliseconds (e.g., 10 seconds)
// unsigned long previousMillis = 0;     // Holds the last time the condition was checked

void setup() {
  // Attach the Servo variable to a pin:
  myservo.attach(servoPin);
  pinMode(sparkPin, OUTPUT);
}

void loop() {

  // unsigned long currentMillis = millis();


  //   // Check if the specified interval has passed since the last time the condition was checked
  // if (currentMillis - previousMillis >= interval) {
  //   // Update the last time the condition was checked
  //   previousMillis = currentMillis;
 
  //   }
  // Tell the servo to go to a particular angle:
  myservo.write(127);
  delay(5000);
  myservo.write(90);
  delay(50);
  digitalWrite(sparkPin, HIGH);
  delay(175);
  digitalWrite(sparkPin, LOW);
  delay(2000);


}