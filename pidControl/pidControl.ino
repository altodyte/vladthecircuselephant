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

int loopDuration = 2; // loop should last as close to 2 milliseconds as possible
long long nextLoop = 0;

bool rollDebug = false;

const unsigned char numC = 12;
double constants[numC];
const char inChars[] = {'a', 'b', 'c', 'd', 'e', 'k', 'P', 'I', 'D', 'r','y','h'};

// PITCH
// PSI
double pitchPsi = 0;
double pitchPsiSet = 0;
double pitchPsiError = 0;
// PHI
double pitchPhi = 0;
double pitchPhiError = 0, pitchPhiErrorLast = 0;
double pitchPhiDeriv = 0;
// Commands / Voltages
double pitchVp = 0, pitchVpLast = 0;
double pitchVv = 0;
long pitchVa = 0;
// ROLL
// PSI
double rollPsi = 0;
double rollPsiSet = 0;
double rollPsiError = 0;
// PHI
double rollPhi = 0;
double rollPhiError = 0, rollPhiErrorLast = 0;
double rollPhiDeriv = 0;
// Commands / Voltages
double rollVp = 0, rollVpLast = 0;
double rollVv = 0;
long rollVa = 0;

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
      if (inChar == 'z') {
        for (k = 0; k < numC; ++k) { Serial.print(constants[k],5); Serial.print(" "); }
        Serial.println();
      }
      if (inChar == 'w') {
        rollDebug = !rollDebug;
      }
    }

    // calculate phi errors
    rollPhiError = 0 - rollPhi;
    pitchPhiError = 0 - pitchPhi;

    // calculate phi derivatives
    rollPhiDeriv = (rollPhiError-rollPhiErrorLast)/loopDuration;
    pitchPhiDeriv = (pitchPhiError-pitchPhiErrorLast)/loopDuration;

    // calculate psi and psi errors
    rollPsi = ticksToRadians(long((-enc0.read()-enc2.read())/2));
    rollPsiError = rollPsi - rollPsiSet;
    pitchPsi = ticksToRadians(long((enc1.read()+enc3.read())/2));
    pitchPsiError = pitchPsi - pitchPsiSet;

    // calculate control signals
    rollVp = -constants[6]*rollPhiError - constants[8]*rollPhiDeriv - constants[9]*rollPsiError; // p-d control
    pitchVp = -constants[6]*pitchPhiError - constants[8]*pitchPhiDeriv - constants[9]*pitchPsiError; // p control

    rollVa = coerce(long(rollVp),-255,255);
    pitchVa = coerce(long(pitchVp),-255,255);

    if (rollDebug){
      Serial.print(rollVa);
      Serial.print(" = -");
      Serial.print(constants[6]); // Proportional Coefficient
      Serial.print("*");
      Serial.print(rollPhiError,5); // Proportional error term
      Serial.print(" + ");
      Serial.print(constants[7]); // Integral Coefficient
      Serial.print("*");
      Serial.print(0,5); // Integral error term
      Serial.print(" - ");
      Serial.print(constants[8]); // Derivative Coefficient
      Serial.print("*");
      Serial.println(rollPhiErrorLast,5); // Derivative error term
    }

    // save current values for next loop
    rollPhiErrorLast = rollPhiError;
    rollVpLast = rollVp;
    pitchPhiErrorLast = pitchPhiError;
    pitchVpLast = pitchVp;

    // set motor control value array, doing sign conversion for motor orientation correction
    mOutVals[0] = -rollVa;
    mOutVals[2] = rollVa;
    mOutVals[1] = pitchVa;
    mOutVals[3] = -pitchVa;
    
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
**************************** Control Scaling Functions ************************
******************************************************************************/
// Mapping Functions
int coerce(int in, int rangeMin, int rangeMax){
  // Bound integer input to range determined by min and max
  if (in<rangeMin){ return rangeMin; }
  else if (in>rangeMax){ return rangeMax; }
  else { return in; }
}

long voltageToMotorShield(double voltage){
    return (long) 255*(voltage/12.0);
}

double ticksToRadians(long ticks){
    return ticks*6.0*M_PI/1000.0;
}

/******************************************************************************
********************** Control Constant EEPROM Management *********************
******************************************************************************/

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