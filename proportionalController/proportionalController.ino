#include <Encoder.h>
#include <Wire.h> // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

/******************************************************************************
************** OBJECTS AND PIN-DEPENDENT DECLARATIONS *************************
******************************************************************************/

#define LED_PIN 13
bool blinkState = false;

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

HardwareSerial & motorSer = Serial;
HardwareSerial & proSer = Serial3;
char inChar;
char mOutNames[] = {'a', 'b', 'c', 'd'};
int mOutVals[4];
unsigned char j;
int k = 0;

/******************************************************************************
***************************** Encoder State Variables *************************
******************************************************************************/

long enc0old  = -999;
long enc1old = -999;
long enc2old = -999;
long enc3old = -999;
long enc0new = enc0.read();
long enc1new = enc1.read();
long enc2new = enc2.read();
long enc3new = enc3.read();

/******************************************************************************
****************************** Motor State Variables **************************
******************************************************************************/

int pitchActuators = 0;
int rollActuators = 0;
double proportionalConstant = 6; 
double killZone = 30.0; // not working because of rapid acceleration
int deadZone = 2;

int loopDuration = 2; // loop should last as close to 2 milliseconds as possible
long long nextLoop = 0;

double roll = 0, pitch = 0;
double rollError = 0, pitchError = 0;

double constantA = 333.333;
double constantB = 332.333;
double constantC = 0.9998;
double k_P = 1;

double pitchVp = 0;
double pitchVpLast = pitchVp;
double pitchErrorLast = pitchError;
double pitchPsi = 0;

double rollVp = 0;
double rollVpLast = rollVp;
double rollErrorLast = rollError;
double rollPsi = 0;


void setup() {
  proSer.setTimeout(5);

  // initialize serial communication
  Serial.begin(250000);
  proSer.begin(250000);

  // configure Arduino LED for heartbeat
  pinMode(LED_PIN, OUTPUT);

  // Initialize motor shield
  // Serial.println("Initializing AFMS I2C device...");
  // AFMS.begin();
  fuck(); // kills motors

  nextLoop = millis() + loopDuration;
}

// long long startTime = 0;

void loop() {
  if (millis() > nextLoop) {
    // startTime = micros();

    nextLoop += loopDuration;

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    // takes 5.5 ms for accelerometer to get new data
    while (proSer.available() > 0) {
      inChar = proSer.read();
      if (inChar == 'r') roll = proSer.parseFloat();
      else if (inChar == 'p') pitch = proSer.parseFloat();
    }

    // Serial.print(roll, 5);
    // Serial.print(' ');
    // Serial.println(pitch, 5);

    // proSer.print(roll, 5);
    // proSer.print(' ');
    // proSer.println(pitch, 5);

    // calculate roll and pitch errors
    rollError = 0 - roll;
    pitchError = 0 - pitch;

    // pitchPsi = ticksToRadians(long((-enc1.read()+enc3.read())/2));
    // voltageIntoPitchPositionFeedback = constantC*previousVoltageIntoPitchPositionFeedback + constantA*pitchError + constantB*previousPitchError;
    // pitchActuators = voltageToMotorShield(voltageIntoPitchPositionFeedback + kayPee*pitchPsi);
    // previousPitchError = pitchError;
    // previousVoltageIntoPitchPositionFeedback = voltageIntoPitchPositionFeedback;
    // m1set(pitchActuators);
    // m3set(-pitchActuators);

    // rollPsi = ticksToRadians(long((-enc0.read()+enc2.read())/2));
    // voltageIntoRollPositionFeedback = constantC*previousVoltageIntoRollPositionFeedback + constantA*rollError + constantB*previousRollError;
    // rollActuators = voltageToMotorShield(voltageIntoRollPositionFeedback + kayPee*rollPsi);
    // previousRollError = rollError;
    // previousVoltageIntoRollPositionFeedback = voltageIntoRollPositionFeedback;
    // m0set(-rollActuators);
    // m2set(rollActuators);

    rollPsi = ticksToRadians(long((-enc0.read()-enc2.read())/2));
    rollVp = constantC*rollVpLast - constantA*rollError + constantB*rollErrorLast;
    // rollActuators = voltageToMotorShield(rollVp + k_P*rollPsi);
    rollActuators = 500.0*roll; // p control
    rollErrorLast = rollError;
    rollVpLast = rollVp;
    // m0set(-rollActuators);
    // m2set(rollActuators);
    mOutVals[0] = -rollActuators;
    mOutVals[2] = rollActuators;

    pitchPsi = ticksToRadians(long((enc1.read()+enc3.read())/2));
    pitchVp = constantC*pitchVpLast - constantA*pitchError + constantB*pitchErrorLast;
    // pitchActuators = voltageToMotorShield(pitchVp + k_P*pitchPsi);
    pitchActuators = 500.0*pitch; // p control
    pitchErrorLast = pitchError;
    pitchVpLast = pitchVp;
    // m1set(pitchActuators);
    // m3set(-pitchActuators);
    mOutVals[1] = pitchActuators;
    mOutVals[3] = -pitchActuators;

    // proSer.print(roll, 5);
    // proSer.print(' ');
    // proSer.println(pitch, 5);

    // proSer.print(rollVp, 5);
    // proSer.print(' ');
    // proSer.println(pitchVp, 5);

    // proSer.print(rollActuators);
    // proSer.print(' ');
    // proSer.println(pitchActuators);
    
    if (++k % 2 == 0) {
      for (j = 0; j < 4; ++j) {
        motorSer.print(mOutNames[j]);
        motorSer.print(mOutVals[j]);
      }
    }

    // proSer.println((long) (micros() - startTime));

    // pitchActuators = modify(proportionalConstant*pitchError);
    // m1set(-pitchActuators);
    // m3set(pitchActuators);

    // rollActuators = modify(proportionalConstant*rollError);
    // m0set(rollActuators);
    // m2set(-rollActuators);

    // Serial.print("Pitch: ");
    // Serial.print(pitch);
    // Serial.print(" || ");
    // Serial.print("M2: ");
    // Serial.print(-pitchActuators);
    // Serial.print(" | ");
    // Serial.print("M4: ");
    // Serial.print(pitchActuators);
    // Serial.print(" ||| ");
    // Serial.print("Roll: ");
    // Serial.print(roll);
    // Serial.print(" || ");
    // Serial.print("M1: ");
    // Serial.print(rollActuators);
    // Serial.print(" | ");
    // Serial.print("M3: ");
    // Serial.println(-rollActuators);
  }
}

void fuck(){
    // Serial.println("KILL MOTORS");
    // m0set(0);
    // m1set(0);
    // m2set(0);
    // m3set(0);
    for (j = 0; j < 4; ++j) {
      motorSer.print(mOutNames[j]);
      motorSer.print(0);
    }
}

/******************************************************************************
***************************** IMU Scaling Functions ***************************
******************************************************************************/

double val2g(int16_t acc){
    return double(acc)/16384.0; // currently set to +/- 2g; needs to be updated/extended if that changes
}

/******************************************************************************
**************************** Control Scaling Functions ************************
******************************************************************************/

// Mapping Functions
int coerce(int in, int rangeMin, int rangeMax){
  // Bound integer input to range determined by min and max
  if (in<rangeMin){ return rangeMin; }
  else if (in>rangeMax){ return rangeMax; }
  else { return in; }
}

double coerce(double in, double rangeMin, double rangeMax){
  // Bound doublee input to range determined by min and max
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

double scale(double in, double inputMin, double inputMax, double outputMin, double outputMax){
  in = coerce(in, inputMin, inputMax);
  double inputRange = inputMax - inputMin;
  double outputRange = outputMax - outputMin;
  float scaleFactor = outputRange / float(inputRange);
  double a = (in-inputMin); // shifted to zero
  double b = int(a*scaleFactor); // scaled to output range
  return b+outputMin; // shifted to output Min
}

int modify(int in){
  int deadZoneMin = -deadZone, deadZoneMax = deadZone;
  if ((in>deadZoneMin)&&(in<deadZoneMax)){
    return 0;
  }
}

// int voltageToMotorShield(double voltage){
//     return int(scale(coerce(voltage, -12.0, 12.0),-12.0,12.0,-255.0,255.0));
// }

long long voltageToMotorShield(double voltage){
    return (long long) 255*(voltage/12.0);
}

double ticksToRadians(long ticks){
    return ticks*6.0*M_PI/1000.0;
}

double d2r(double d){
  return d*M_PI/180.0;
}

/******************************************************************************
***************************** Motor Driving Functions *************************
******************************************************************************/

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