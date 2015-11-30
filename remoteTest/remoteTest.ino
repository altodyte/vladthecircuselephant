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

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motor0 = AFMS.getMotor(1);
Adafruit_DCMotor *motor1 = AFMS.getMotor(2);
Adafruit_DCMotor *motor2 = AFMS.getMotor(3);
Adafruit_DCMotor *motor3 = AFMS.getMotor(4);

bool printFlag = false;
long enc0old  = -999;
long enc1old = -999;
long enc2old = -999;
long enc3old = -999;
long enc0new = enc0.read();
long enc1new = enc1.read();
long enc2new = enc2.read();
long enc3new = enc3.read();

int throttlePin = 9; // Throttle PWM Input | [Action tied to Throttle]
int elevatorPin = 11;

// STATE VARIABLES
// RC state variables
int throttlePWM; // current throttle pwm width reading
int elevatorPWM;

// System Parameters
int throttlePWMmin = 1120, throttlePWMmax = 1880; // Might need to be updated
int elevatorPWMmin = 1085, elevatorPWMmax = 1800;
int actMin = -100, actMax = 100;
int throttle = 0, elevator = 0;
// 1099-1105, 1482-1490, 1801-07 (Elevator)
// 1113-20, X ,1881-7 (Throttle)

int loopDelay = 10; // ms
long nextLoop = millis()+loopDelay;

bool serFlag = false;
bool remoteFlag = true;

void setup() {
  Serial.begin(9600);
  Serial.println("Before shield initialization.");
  AFMS.begin();
  Serial.println("Basic Encoder Test:");
  m0set(0);
  m1set(0);
  m2set(0);
  m3set(0);
  // Initialize Pins
  pinMode(throttlePin, INPUT);
  pinMode(elevatorPin, INPUT);
}

void loop() {
  if (millis()>nextLoop){
    checkEncoders();
    printEncoders();
    if ((serFlag) && (Serial.available())){
      int speed = Serial.parseInt();
      m0set(speed);
      // m1set(speed);
      m2set(-speed);
      // m3set(speed);
      Serial.print("MOTOR SPEED: ");
      Serial.println(speed);
    }
    nextLoop = millis()+loopDelay;
  }
  // Read PWM
  throttlePWM = pulseIn(throttlePin, HIGH);
  elevatorPWM = pulseIn(elevatorPin, HIGH);
  // Scale things
  throttle = scale(throttlePWM,throttlePWMmin,throttlePWMmax,actMin,actMax);
  elevator = scale(elevatorPWM,elevatorPWMmin,elevatorPWMmax,actMin,actMax);
  throttle = modify(throttle);
  elevator = modify(elevator);
  // Print Status for Debugging
  if (serFlag){
    Serial.println("Throttle: "+String(throttlePWM)+
                   ", Elevator: "+String(elevatorPWM)+
                   "| Throttle: "+String(throttle)+
                   ", Elevator: "+String(elevator));
  } 
  if (remoteFlag) {
    m0set(throttle);
    m1set(elevator);
    m2set(-throttle);
    m3set(-elevator);
  }
}

// Mapping Functions
int coerce(int in, int rangeMin, int rangeMax){
  // Bound integer input to range determined by min and max
  if (in<rangeMin){ return rangeMin; }
  else if (in>rangeMax){ return rangeMax; }
  else { return in; }
}

int scale(int in, int inputMin, int inputMax, int outputMin, int outputMax){
  in = coerce(in, inputMin, inputMax);
  int inputRange = inputMax - inputMin;
  int outputRange = outputMax - outputMin;
  float scaleFactor = outputRange / float(inputRange);
  int a = (in-inputMin); // shifted to zero
  int b = int(a*scaleFactor); // scaled to output range
  return b+outputMin; // shifted to output Min
}

int modify(int in){
  int deadZoneMin = -18, deadZoneMax = 18;
  if ((in>deadZoneMin)&&(in<deadZoneMax)){
    return 0;
  }
}

// Motor & Encoder Functions

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
    Serial.print("Enc0: ");
    Serial.print(enc0old);
    Serial.print(" | Enc1: ");
    Serial.print(enc1old);
    Serial.print(" | Enc2: ");
    Serial.print(enc2old);
    Serial.print(" | Enc3: ");
    Serial.println(enc3old);
    printFlag = false; 
  }
}

void m0set(int i){
  if (i==0) {
    motor0->run(RELEASE);
    motor0->setSpeed(0);
  } else if (i<0){
    motor0->setSpeed(abs(i));
    motor0->run(BACKWARD);
  } else {
    motor0->setSpeed(i);
    motor0->run(FORWARD);
  }
}

void m1set(int i){
  if (i==0) {
    motor1->run(RELEASE);
    motor1->setSpeed(0);
  } else if (i<0){
    motor1->setSpeed(abs(i));
    motor1->run(BACKWARD);
  } else {
    motor1->setSpeed(i);
    motor1->run(FORWARD);
  }
}

void m2set(int i){
  if (i==0) {
    motor2->run(RELEASE);
    motor2->setSpeed(0);
  } else if (i<0){
    motor2->setSpeed(abs(i));
    motor2->run(BACKWARD);
  } else {
    motor2->setSpeed(i);
    motor2->run(FORWARD);
  }
}

void m3set(int i){
  if (i==0) {
    motor3->run(RELEASE);
    motor3->setSpeed(0);
  } else if (i<0){
    motor3->setSpeed(abs(i));
    motor3->run(BACKWARD);
  } else {
    motor3->setSpeed(i);
    motor3->run(FORWARD);
  }
}