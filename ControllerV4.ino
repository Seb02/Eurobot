#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_MotorShield.h>
#include <Arduino.h>
#include <avr/wdt.h>


#define CLK_PIN 3
#define SDO_PIN 2

float totalDistance = 0;

float angle = 0;
float prevAngle;
float Xtot = 0;
float Ytot = 0;
float Xshift = 0;
float Yshift = 0;
float RightWheelSpeed = 0;
float LeftWheelSpeed = 0;
float WheelSpacing = 11;
float AngularSpeed = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *MotorLeft = AFMS.getMotor(1);//Moteur gauche
Adafruit_DCMotor *MotorRight= AFMS.getMotor(2);//Moteur droit


MPU6050 mpu(Wire);



//############optical sensor functions###############

void sendByte(uint8_t byte) {
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDO_PIN, (byte >> i) & 0x01);
    delayMicroseconds(3);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(CLK_PIN, LOW);
  }
  digitalWrite(CLK_PIN, HIGH);
}

uint8_t readByte() {
  uint8_t byte = 0;
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(1);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(1);
    byte |= (digitalRead(SDO_PIN) << i);
    //****Serial.print(digitalRead(SDO_PIN));
  }
  digitalWrite(CLK_PIN, HIGH);
  

  return byte;
}
void WriteConfig(uint8_t addr,uint8_t config, int delay){
  digitalWrite(CLK_PIN, HIGH);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDO_PIN, (addr >> i) & 0x01);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, HIGH);
  }
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDO_PIN, (config >> i) & 0x01);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, HIGH);
  }
}
int8_t Ask(uint8_t addr, int delay) {
  int8_t byte = 0;
  pinMode(SDO_PIN,OUTPUT);
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDO_PIN, (addr >> i) & 0x01);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, LOW);
  }
  pinMode(SDO_PIN,INPUT);
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(delay);
    byte |= (digitalRead(SDO_PIN) << i);;
  }
  digitalWrite(CLK_PIN, HIGH);
  return byte;
  //Serial.println();
}
void AskSilent(uint8_t addr, int delay) {
  pinMode(SDO_PIN,OUTPUT);
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SDO_PIN, (addr >> i) & 0x01);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(delay);
    digitalWrite(CLK_PIN, LOW);
  }
  //digitalWrite(CLK_PIN, HIGH);
  pinMode(SDO_PIN,INPUT);
  digitalWrite(CLK_PIN, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(delay);
    //Serial.print(digitalRead(SDO_PIN));
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(delay);
    //byte |= (digitalRead(SDO_PIN) << i);;
  }
  digitalWrite(CLK_PIN, HIGH);
  //Serial.println();
}

//############# Computation functions ###################

double toRadians(double degrees) {
  return degrees * (PI / 180.0);
}

void calculateCoordinates(double addedDistance, double angle, float Xshift, float Yshift, float &Xtot, float &Ytot) {
  
    
    double radians = toRadians(angle);

    
    double Xcomponent = addedDistance * cos(radians);
    double Ycomponent = addedDistance * sin(radians);

    
    Xtot = Xtot + Xcomponent + Xshift;
    Ytot = Ytot + Ycomponent + Yshift;
}

void computeAngularVelocity(float ZangleSpeed, float Xspeed, float &AngularSpeed){

  LeftWheelSpeed = Xspeed - (ZangleSpeed * (WheelSpacing/2));
  RightWheelSpeed = Xspeed + (ZangleSpeed * (WheelSpacing/2));

  AngularSpeed = (RightWheelSpeed - LeftWheelSpeed) / WheelSpacing;


}





//############ Motors functions ###############

void moveMotors(int leftSpeed, int rightSpeed, int leftDirection, int rightDirection) {
  MotorLeft->run(leftDirection);
  MotorLeft->setSpeed(leftSpeed);
  MotorRight->run(rightDirection);
  MotorRight->setSpeed(rightSpeed);
}


void stopMotors() {
  MotorLeft->setSpeed(0);
  MotorLeft->run(RELEASE);
  MotorRight->setSpeed(0);
  MotorRight->run(RELEASE);
}




//############# UART functions ###################

void sendSpeedsViaUART(float Xspeed, float ZangleSpeed, float Xtot, float Ytot, int totalAngle) {
  
  Serial.print("linear: ");
  Serial.println(Xspeed);
  Serial.print("angular: ");
  Serial.println(ZangleSpeed);
  Serial.print("XY: ");
  Serial.print("(");
  Serial.print(Xtot);
  Serial.print(",");
  Serial.print(Ytot);
  Serial.println(")");
  Serial.print("angle: ");
  Serial.println(totalAngle);
}



void readMotorSpeedsFromUART(int &leftSpeed, int &rightSpeed, int &leftDirection, int &rightDirection, float Xspeed, float ZangleSpeed, float Xtot, float Ytot, int totalAngle) {
  if (Serial.available() >= 1) {
    String speedData = Serial.readStringUntil('\n'); 

  int reset = speedData.indexOf("RESET");
  int setPosition = speedData.indexOf("setPosition");
  if (reset != -1) {
    wdt_enable(WDTO_15MS); //RESET avec un Watchdogtimer
      while (1);  
  }
  if (setPosition != -1){
      changeCoordinates(speedData);
    }

    int colonIndex = speedData.indexOf(':');
    
    if (colonIndex != -1 && colonIndex < speedData.length() - 1) {
      
      String leftSpeedStr = speedData.substring(0, colonIndex);
      String rightSpeedStr = speedData.substring(colonIndex + 1);

      
      int receivedLeftSpeed = leftSpeedStr.toInt();
      int receivedRightSpeed = rightSpeedStr.toInt();

      
      leftDirection = (receivedLeftSpeed <= 0) ? FORWARD : BACKWARD;
      rightDirection = (receivedRightSpeed <= 0) ? FORWARD : BACKWARD;

      
      leftSpeed = map(abs(receivedLeftSpeed), 0, 100, 0, 255);
      rightSpeed = map(abs(receivedRightSpeed), 0, 100, 0, 255);

      sendSpeedsViaUART(Xspeed, AngularSpeed, Xtot, Ytot, angle);
    } else {
      
      leftSpeed = leftSpeed ;
      rightSpeed = rightSpeed;
      leftDirection = leftDirection;
      rightDirection = rightDirection;
    }
  } else {
    
    leftSpeed = leftSpeed ;
    rightSpeed = rightSpeed;
    leftDirection = leftDirection;
    rightDirection = rightDirection;
  }
}

void changeCoordinates(String serialRead){
  int indexParenthesis1 = serialRead.indexOf("(");
  int indexColumn = serialRead.indexOf(",");
  int indexParenthesis2 = serialRead.indexOf(")");

  String Xpos = serialRead.substring(indexParenthesis1 + 1, indexColumn);
  String Ypos = serialRead.substring(indexColumn + 1, indexParenthesis2);
  
  Xshift = Xpos.toFloat();
  Yshift = Ypos.toFloat();
  

}

void setup() {

  //setup gyro : 

  Serial.begin(115200);
  Wire.begin();
  byte status = mpu.begin();
  //****Serial.print(F("MPU6050 status: "));
  //****Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  //****Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  //*****Serial.println("Done!\n");

  

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    //******Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  //******Serial.println("Motor Shield found.");

  
  MotorLeft->setSpeed(0);
  MotorLeft->run(FORWARD);
  MotorLeft->run(RELEASE);
  MotorRight->setSpeed(0);
  MotorRight->run(FORWARD);
  MotorRight->run(RELEASE);


 //setup optical:

  pinMode(CLK_PIN,OUTPUT);
  pinMode(SDO_PIN,INPUT);
  digitalWrite(CLK_PIN,HIGH);
  digitalWrite(SDO_PIN,HIGH);
  
  delayMicroseconds(10000);
  WriteConfig(0b10000101,0b10111000,3);
  delayMicroseconds(10000);
  WriteConfig(0b10000110,0b10000100,3); //reg 0x06
  delayMicroseconds(10000); 
  WriteConfig(0b10001110,0b11101001,3);
  Serial.println("reset done");

  sendSpeedsViaUART(0, 0, 0, 0, 0);

}

int posX = 0;
float Xcm = 0;
unsigned long currentTimeMPU = 0;
unsigned long prevTimeMPU = 0;
unsigned long elapsedTimeMPU = 0;

float ZangleSpeed = 0;

int iMPU = 4;
unsigned long prevTime = 0;

int leftSpeed = 0;
int rightSpeed = 0;
int leftDirection = FORWARD;
int rightDirection = FORWARD;
int accelY = 0;


void loop() {
  if(iMPU>3){
    currentTimeMPU = millis();
    mpu.update();
    angle = -(mpu.getAngleZ());
    accelY = (mpu.getAccY())*100;
    elapsedTimeMPU = currentTimeMPU - prevTimeMPU;
    ZangleSpeed = ((angle - prevAngle) / elapsedTimeMPU) * 1000;
  }
  
  AskSilent(0x02,3);
  
  int8_t Xvel = Ask(0x03,3);
  
 
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - prevTime;
  
  
  //i+=1;
  posX = posX + Xvel; //-2650 = 10cm
  Xcm = -posX * 0.00403877;  //ecam : 0.00403877; parquet axel : //0.0037735849;
  

  
  
  
  float Xspeed = (Xcm / elapsedTime) * 1000;
  
  

  totalDistance = totalDistance + Xcm;
  
  

  //remise du temps à 0 pour tenir compte du temps écoulé pour le reste de la loop
  
  if (iMPU>3) {
    prevTimeMPU = millis();
    mpu.update();
    prevAngle = -(mpu.getAngleZ());
    iMPU = 0;
   
  }
  calculateCoordinates(Xcm, angle, Xshift, Yshift, Xtot, Ytot);
  posX = 0;
  prevTime = millis();
  //****Serial.println(Xspeed);
  delay(3);//10
  
  computeAngularVelocity(ZangleSpeed, Xspeed, AngularSpeed);
  //sendSpeedsViaUART(Xspeed, ZangleSpeed, Xtot, Ytot, angle);
  
  readMotorSpeedsFromUART(leftSpeed, rightSpeed, leftDirection, rightDirection, Xspeed, AngularSpeed, Xtot, Ytot, angle);
  moveMotors(leftSpeed, rightSpeed, leftDirection, rightDirection);
  iMPU +=1;

}

