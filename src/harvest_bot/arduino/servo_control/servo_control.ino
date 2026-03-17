#include <Servo.h>

Servo cutterServo;
const int servoPin = 9;

void setup() {
  Serial.begin(9600); 
  cutterServo.attach(servoPin);
  
  // Start the motor in the STOPPED state (90 means stop for your motor)
  cutterServo.write(90); 
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == '1') {
      // OPEN BLADES
      cutterServo.write(225);  // 1. Start spinning forward
      delay(1000);              // 2. Keep spinning for 400 milliseconds
      // 3. FORCE STOP
    } 
    else if (command == '0') {
      // CLOSE BLADES
      cutterServo.write(70);    // 1. Start spinning backward
      delay(1000);              // 2. Keep spinning for 400 milliseconds
     // 3. FORCE STOP
    }
  }
}