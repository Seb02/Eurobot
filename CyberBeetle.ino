#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();


// Select which 'port' M1, M2, M3 or M4. In this case, M1
//Moteur gauche
Adafruit_DCMotor *MotorLeft = AFMS.getMotor(1);
//Moteur droit
Adafruit_DCMotor *MotorRight= AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Start up");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  MotorLeft->setSpeed(0);
  MotorLeft->run(FORWARD);
  // turn on motor
  MotorLeft->run(RELEASE);
}

void loop() {
  

  moveForward(150, 2);

  delay(1000);

  moveBackward(150, 2);
}

void moveForward(uint8_t speed, int time) {
  
  MotorLeft->run(FORWARD);
  MotorLeft->setSpeed(speed);
  
  MotorRight->run(FORWARD);
  MotorRight->setSpeed(speed);
  

  delay(time * 1000);  // temps en millisecondes
  stopMotors();


}

void backward(uint8_t speed, int time) {
  MotorLeft->run(BACKWARD);
  MotorLeft->setSpeed(speed);
  MotorRight->run(BACKWARD);
  MotorRight->setSpeed(speed);

  delay(time * 1000);  
  stopMotors();        
}

void stopMotors() {
  MotorLeft->setSpeed(0);
  MotorLeft->run(RELEASE);
  MotorRight->setSpeed(0);
  MotorRight->run(RELEASE);
}
