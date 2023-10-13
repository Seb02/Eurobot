#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3, or M4. In this case, M1
// Moteur gauche
Adafruit_DCMotor *MotorLeft = AFMS.getMotor(1);
// Moteur droit
Adafruit_DCMotor *MotorRight = AFMS.getMotor(2);

// Define constants for encoder parameters
const int ticksPerRevolution = 40; // Number of ticks per revolution
const float wheelDiameter = 7.0;   // Diameter of the wheel in centimeters

// Define variables for encoder counts
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// Function to calculate robot speed using encoder data
float calculateRobotSpeed() {
  // Calculate the distance traveled by each wheel
  float leftDistance = (leftEncoderTicks / ticksPerRevolution) * wheelDiameter * 3.14159265;
  float rightDistance = (rightEncoderTicks / ticksPerRevolution) * wheelDiameter * 3.14159265;
  
  // Calculate the average speed of the robot
  float robotSpeed = (leftDistance + rightDistance) / 2.0;
  return robotSpeed;
}

// Function for the PID controller
void pidController(float targetSpeed, float currentSpeed) {
  float Kp = 1.0;  // Proportional constant
  float Ki = 0.1;  // Integral constant
  float Kd = 0.01; // Derivative constant
  
  static float previousError = 0.0;
  static float integral = 0.0;

  // Calculate the error between the target and current speed
  float error = targetSpeed - currentSpeed;
  
  // Calculate the integral term
  integral += error;

  // Calculate the derivative term
  float derivative = error - previousError;

  // Calculate the control signal
  float controlSignal = Kp * error + Ki * integral + Kd * derivative;

  // Apply the control signal to the motors
  int motorSpeed = constrain(controlSignal, -255, 255); // Constrain the value to the motor speed range
  MotorLeft->setSpeed(abs(motorSpeed));
  MotorRight->setSpeed(abs(motorSpeed));

  // Update previous error
  previousError = error;
}

// Function to handle left encoder interrupt
void leftEncoderISR() {
  leftEncoderTicks++;
}

// Function to handle right encoder interrupt
void rightEncoderISR() {
  rightEncoderTicks++;
}

void setup() {
  Serial.begin(9600); // Set up Serial library at 9600 bps
  Serial.println("Start up");

  if (!AFMS.begin()) { // Create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) { // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  MotorLeft->setSpeed(0);
  MotorLeft->run(FORWARD);
  // Turn on motor
  MotorLeft->run(RELEASE);

  // Attach interrupt service routines for encoders
  attachInterrupt(digitalPinToInterrupt(2), leftEncoderISR, RISING); // Attach left encoder interrupt
  attachInterrupt(digitalPinToInterrupt(3), rightEncoderISR, RISING); // Attach right encoder interrupt
}

void loop() {
  float robotSpeed = calculateRobotSpeed();

  // Set the target speed for the PID controller
  float targetSpeed = 150.0; // Target speed

  pidController(targetSpeed, robotSpeed);

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

void moveBackward(uint8_t speed, int time) {
  MotorLeft->run(BACKWARD);
  MotorLeft->setSpeed(speed);
  MotorRight->run(BACKWARD);
  MotorRight->setSpeed(speed);

  delay(time * 1000);  
  stopMotors();        
}

void turn(char direction, int angle) { //il faudra calculer le temps optimal, ou arrêter la rotation en se basant sur l'encodeur/accéléromètre

  //float rotationTime = (float)angle / //vitesse angulaire à calculer !!!

  // Tournez les moteurs dans des directions opposées pour tourner sur place
  MotorLeft->run(FORWARD);
  MotorLeft->setSpeed(155);
  MotorRight->run(BACKWARD);
  MotorRight->setSpeed(155);

  delay(2 * 1000);  
  stopMotors();                
}




void stopMotors() {
  MotorLeft->setSpeed(0);
  MotorLeft->run(RELEASE);
  MotorRight->setSpeed(0);
  MotorRight->run(RELEASE);
}
