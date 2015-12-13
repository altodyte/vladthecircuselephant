#include <EEPROM.h>
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

// HardwareSerial & compSer = Serial;
HardwareSerial & sysSer = Serial3;
// #define DEBUG
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


const unsigned char numC = 5;
double constants[numC];
const char inChars[] = {'a', 'b', 'c', 'k', 'p'};

double pitchPsi = 0;
double pitchPsiSet = 0;
double pitchPsiError = 0;
double pitchPhi = 0;
double pitchPhiError = 0;
double pitchVp = 0;
double pitchVpLast = 0;
double pitchVv = 0;
double pitchVa = 0;
double pitchPhiErrorLast = 0;

double rollPsi = 0;
double rollPsiSet = 0;
double rollPsiError = 0;
double rollPhi = 0;
double rollPhiError = 0;
double rollVp = 0;
double rollVpLast = 0;
double rollVv = 0;
double rollVa = 0;
double rollPhiErrorLast = 0;


void setup() {
  sysSer.setTimeout(5);

  // initialize serial communication
  Serial.begin(250000);
  sysSer.begin(250000);
  fuck(); // kills motors

  // configure Arduino LED for heartbeat
  pinMode(LED_PIN, OUTPUT);

  for (k = 0; k < numC; ++k) constants[k] = EEPROM_readDouble(4*k);

  nextLoop = millis() + loopDuration;
}

#ifdef DEBUG
  long long startTime = 0;
#endif

void loop() {
  if (millis() > nextLoop) {
    #ifdef DEBUG
      startTime = micros();
    #endif

    nextLoop += loopDuration;

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    // takes 5.5 ms for accelerometer to get new data
    while (sysSer.available()) {
      inChar = sysSer.read();
      if (inChar == 'r') rollPhi = sysSer.parseFloat();
      else if (inChar == 'p') pitchPhi = sysSer.parseFloat();
    }

    // read constants from serial
    while (Serial.available()) {
      inChar = Serial.read();
      for (k = 0; k < numC; ++k) if (inChar == inChars[k]) EEPROM_writeDouble(k*4, constants[k] = Serial.parseFloat());
      if (inChar == 'd') {
        for (k = 0; k < numC; ++k) { Serial.print(constants[k]); Serial.print(" "); }
        Serial.println();
      }
    }

    // Serial.print(roll, 5);
    // Serial.print(' ');
    // Serial.println(pitch, 5);

    // calculate phi errors
    rollPhiError = 0 - rollPhi;
    pitchPhiError = 0 - pitchPhi;

    // calculate psi and psi errors
    rollPsi = ticksToRadians(long((-enc0.read()-enc2.read())/2));
    rollPsiError = rollPsi - rollPsiSet;
    pitchPsi = ticksToRadians(long((enc1.read()+enc3.read())/2));
    pitchPsiError = pitchPsi - pitchPsiSet;

    // calculate control signals
    rollVp = constants[2]*rollVpLast - constants[0]*rollPhiError + constants[1]*rollPhiErrorLast;
    rollVv = rollVp + constants[3]*rollPsiError;
    rollVa = voltageToMotorShield(rollVv);
    // rollActuators = constants[4]*rollPhi; // p control
    pitchVp = constants[2]*pitchVpLast - constants[0]*pitchPhiError + constants[1]*pitchPhiErrorLast;
    pitchVv = pitchVp + constants[3]*pitchPsiError;
    pitchVa = voltageToMotorShield(pitchVv);
    // pitchActuators = constants[4]*pitchPhi; // p control

    // save current values for next loop
    rollPhiErrorLast = rollPhiError;
    rollVpLast = rollVp;
    pitchPhiErrorLast = pitchPhiError;
    pitchVpLast = pitchVp;

    // set motor control value array, doing sign conversion for motor orientation correction
    mOutVals[0] = -rollActuators;
    mOutVals[2] = rollActuators;
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
    
    // send motor commands every other loop, because motor controller is too slow to take commands every loop
    if (++k % 2 == 0) {
      for (j = 0; j < 4; ++j) {
        sysSer.print(mOutNames[j]);
        sysSer.print(mOutVals[j]);
      }
    }

    #ifdef DEBUG
      Serial.println((long) (micros() - startTime));
    #endif
  }
}

void fuck(){
    for (j = 0; j < 4; ++j) {
      sysSer.print(mOutNames[j]);
      sysSer.print(0);
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

long long voltageToMotorShield(double voltage){
    return (long long) 255*(voltage/12.0);
}

double ticksToRadians(long ticks){
    return ticks*6.0*M_PI/1000.0;
}

double d2r(double d){
  return d*M_PI/180.0;
}

void EEPROM_writeDouble(int ee, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
       EEPROM.write(ee++, *p++);
}

double EEPROM_readDouble(int ee)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
       *p++ = EEPROM.read(ee++);
   return value;
}