/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder enc0(2,10);
Encoder enc1(3,12);
Encoder enc2(18,16);
Encoder enc3(19,17);
//   avoid using pins with LEDs attached

// Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Adafruit_DCMotor *motor0 = AFMS.getMotor(1);
// Adafruit_DCMotor *motor1 = AFMS.getMotor(2);
// Adafruit_DCMotor *motor2 = AFMS.getMotor(3);
// Adafruit_DCMotor *motor3 = AFMS.getMotor(4);

bool printFlag = false;
long enc0old  = -999;
long enc1old = -999;
long enc2old = -999;
long enc3old = -999;
long enc0new = enc0.read();
long enc1new = enc1.read();
long enc2new = enc2.read();
long enc3new = enc3.read();

void setup() {
  // AFMS.begin();
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  // m0set(0);
  // m1set(0);
  // m2set(0);
  // m3set(0);
}

void loop() {
  delay(10);
  checkEncoders();
  printEncoders();
}

void checkEncoders(){
  enc0new = enc0.read();
  enc1new = enc1.read();
  enc2new = enc2.read();
  enc3new = enc3.read();

  if (enc0new != enc0old) {
    enc0old = enc0new;
    printFlag = true;
  }
  if (enc1new != enc1old) {
    enc1old = enc1new;
    printFlag = true;
  }
  if (enc2new != enc2old) {
    enc2old = enc2new;
    printFlag = true;
  }
  if (enc3new != enc3old) {
    enc3old = enc3new;
    printFlag = true;
  }
}

void printEncoders(){
  if (printFlag) {
    long t = millis();
    Serial.print(t);
    Serial.print(", ");
    Serial.print(enc0old);
  
    Serial.print(", ");
    Serial.print(enc1old);
  
    Serial.print(", ");
    Serial.print(enc2old);
  
    Serial.print(", ");
    Serial.println(enc3old);
    
    printFlag = false; 
  }
}

// void m0set(int i){
//   if (i==0) {
//     motor0->run(RELEASE);
//     motor0->setSpeed(0);
//   } else if (i<0){
//     motor0->setSpeed(abs(i));
//     motor0->run(BACKWARD);
//   } else {
//     motor0->setSpeed(i);
//     motor0->run(FORWARD);
//   }
// }

// void m1set(int i){
//   if (i==0) {
//     motor1->run(RELEASE);
//     motor1->setSpeed(0);
//   } else if (i<0){
//     motor1->setSpeed(abs(i));
//     motor1->run(BACKWARD);
//   } else {
//     motor1->setSpeed(i);
//     motor1->run(FORWARD);
//   }
// }

// void m2set(int i){
//   if (i==0) {
//     motor2->run(RELEASE);
//     motor2->setSpeed(0);
//   } else if (i<0){
//     motor2->setSpeed(abs(i));
//     motor2->run(BACKWARD);
//   } else {
//     motor2->setSpeed(i);
//     motor2->run(FORWARD);
//   }
// }

// void m3set(int i){
//   if (i==0) {
//     motor3->run(RELEASE);
//     motor3->setSpeed(0);
//   } else if (i<0){
//     motor3->setSpeed(abs(i));
//     motor3->run(BACKWARD);
//   } else {
//     motor3->setSpeed(i);
//     motor3->run(FORWARD);
//   }
// }