#include <EEPROM.h>
#include <Encoder.h>
#include <Wire.h> // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// misc variables
// #define DEBUG
bool spew = true;
bool driving = true;
bool blinkState = false;
unsigned char loopDuration = 10; // loop should last as close to x milliseconds as possible
unsigned long nextLoop = 0;
unsigned char j; // loop counter
static union {
  unsigned long currTime;
  byte currTimeBytes[4];
};
#define getSign(x) (x > 0) - (x < 0)

// encoder variables
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder enc0(2,10);
Encoder enc1(3,12);
Encoder enc2(18,16);
Encoder enc3(19,17);

// serial variables
HardwareSerial & sysSer = Serial3;
char inChar;

// encoder variables
long enc0old  = -999;
long enc1old = -999;
long enc2old = -999;
long enc3old = -999;
long enc0new = enc0.read();
long enc1new = enc1.read();
long enc2new = enc2.read();
long enc3new = enc3.read();

// motor variables
char mOutNames[] = {'a', 'b', 'c', 'd'};
int mOutVals[4];
double deadZone = 0.4;

// controller variables
const unsigned char numC = 14;
double constants[numC];
// a, b, c, d, e, k_p, P_phi, I_phi, D_phi, P_psi, I_psi, D_psi rollPsiSet pitchPsiSet
const char inChars[] = {'a', 'b', 'c', 'd', 'e', 'k', 'P','I','D','R','Y','H','r','p'}; 

// PITCH
// PSI
double pitchPsi = 0;
double pitchPsiSet = 0;
double pitchPsiError = 0, pitchPsiErrorLast1 = 0;
double pitchPsiErrorDeriv = 0;
// PHI -- set is always 0
double pitchPhi = 0;
double pitchPhiError = 0, pitchPhiErrorLast1 = 0, pitchPhiErrorLast2 = 0;
double pitchPhiErrorDeriv = 0;
double pitchPhiErrorInteg = 0;
// Commands / Voltages
double pitchVp = 0, pitchVpLast1 = 0, pitchVpLast2 = 0;
double pitchVv = 0;
double pitchVa = 0;

// ROLL
// PSI
double rollPsi = 0;
double rollPsiSet = 0;
double rollPsiError = 0, rollPsiErrorLast1 = 0;
double rollPsiErrorDeriv = 0;
// PHI -- set is always 0
double rollPhi = 0;
double rollPhiError = 0, rollPhiErrorLast1 = 0, rollPhiErrorLast2 = 0;
double rollPhiErrorDeriv = 0;
double rollPhiErrorInteg = 0;
// Commands / Voltages
double rollVp = 0, rollVpLast1 = 0, rollVpLast2 = 0;
double rollVv = 0;
double rollVa = 0;

// pointers to the values we're sending out for logging
double* outVals[] = {&pitchPsi, &pitchPhi, &pitchVa, &rollPsi, &rollPhi, &rollVa};

void setup() {
  sysSer.setTimeout(2);
  Serial.setTimeout(2);

  // initialize serial communication
  Serial.begin(250000);
  // sysSer.begin(250000);
  sysSer.begin(125000); // slower serial because optoisolator sucks
  stopMotors(); // kills motors

  // configure Arduino LED for heartbeat
  pinMode(LED_BUILTIN, OUTPUT);

  for (j = 0; j < numC; ++j) constants[j] = EEPROM_readDouble(4*j);

  nextLoop = millis() + loopDuration;
}

// long long startTime = 0;

void loop() {
  currTime = millis();
  if (currTime > nextLoop) {
    // startTime = micros();
    nextLoop += loopDuration;

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);

    // takes 5.5 ms for accelerometer to get new data
    while (sysSer.available()) {
      inChar = sysSer.read();
      if (inChar == 'r') rollPhi = sysSer.parseFloat();
      else if (inChar == 'p') pitchPhi = sysSer.parseFloat();
    }

    // read constants from serial
    while (Serial.available()) {
      inChar = Serial.read();
      for (j = 0; j < numC; ++j) if (inChar == inChars[j]) EEPROM_writeDouble(j*4, constants[j] = Serial.parseFloat());
      if (inChar == 'z') {
        Serial.println();
        for (j = 0; j < numC; ++j) { Serial.print(inChars[j]); Serial.print("\t"); }
        Serial.println();
        for (j = 0; j < numC; ++j) { Serial.print(constants[j], 3); Serial.print("\t"); }
        Serial.println();
      }
      if (inChar == 'w') {
        spew = !spew;
      }
      if (inChar == 'x') {
        driving = !driving;
      }
    }

    // calculate phi errors
    rollPhiError = 0 - rollPhi;
    pitchPhiError = 0 - pitchPhi;

    if (abs(rollPhiError - rollPhiErrorLast1) > 0.5) { rollPhiError = 0; rollPhi = 0; }
    if (abs(pitchPhiError - pitchPhiErrorLast1) > 0.5) { pitchPhiError = 0; pitchPhi = 0; }

    // calculate phi error derivatives
    rollPhiErrorDeriv = 1000.0*(rollPhiError-rollPhiErrorLast1)/loopDuration;
    pitchPhiErrorDeriv = 1000.0*(pitchPhiError-pitchPhiErrorLast1)/loopDuration;

    // calculate phi error integrals
    rollPhiErrorInteg += rollPhiError;
    pitchPhiErrorInteg += pitchPhiError;

    // calculate psi and psi errors
    rollPsi = ticksToRadians(long((-enc0.read()-enc2.read())/2));
    // rollPsiError = rollPsi - rollPsiSet;
    rollPsiError = rollPsi - constants[12];
    pitchPsi = ticksToRadians(long((enc1.read()+enc3.read())/2));
    // pitchPsiError = pitchPsi - pitchPsiSet;
    pitchPsiError = pitchPsi - constants[13];

    // calculate psi error derivatives
    rollPsiErrorDeriv = 1000.0*(rollPsiError - rollPsiErrorLast1)/loopDuration;
    pitchPsiErrorDeriv = 1000.0*(pitchPsiError - pitchPsiErrorLast1)/loopDuration;

    // calculate control signals
    // control from phi
    rollVp = -constants[6]*rollPhiError - constants[7]*rollPhiErrorInteg - constants[8]*rollPhiErrorDeriv;
    // control from psi
    rollVp = rollVp -constants[9]*rollPsiError - constants[10]*0 - constants[11]*rollPsiErrorDeriv;
    // control from phi
    pitchVp = -constants[6]*pitchPhiError -constants[7]*pitchPhiErrorInteg - constants[8]*pitchPhiErrorDeriv;
    // control from psi
    pitchVp = pitchVp - constants[9]*pitchPsiError -constants[10]*0 - constants[11]*pitchPsiErrorDeriv;

    // rollVa = rollVp - (rollPhiError>0.15)?(0.3*constants[6]*(rollPhiError-0.15*getSign(rollPhiError))):0;
    // pitchVa = pitchVp - (rollPhiError>0.15)?(0.3*constants[6]*rollPhiError):0;
    rollVa = rollVp;
    pitchVa = pitchVp;

    // save current values for next loop
    // rollPhiErrorLast2 = rollPhiErrorLast1;
    // rollVpLast2 = rollVpLast1;
    // pitchPhiErrorLast2 = pitchPhiErrorLast1;
    // pitchVpLast2 = pitchVpLast1;
    rollPhiErrorLast1 = rollPhiError;
    rollVpLast1 = rollVp;
    pitchPhiErrorLast1 = pitchPhiError;
    pitchVpLast1 = pitchVp;
    rollPsiErrorLast1 = rollPsiError;
    pitchPsiErrorLast1 = pitchPsiError; 

    // set motor control value array, doing sign conversion for motor orientation correction
    mOutVals[0] = coerce(voltageToMotorShield(fixDeadZone(rollVa)));
    mOutVals[2] = coerce(voltageToMotorShield(fixDeadZone(-rollVa)));
    mOutVals[1] = coerce(voltageToMotorShield(fixDeadZone(-pitchVa)));
    mOutVals[3] = coerce(voltageToMotorShield(fixDeadZone(pitchVa)));
    
    if (driving) {
      for (j = 0; j < 4; ++j) {
        sysSer.print(mOutNames[j]);
        sysSer.print(mOutVals[j]);
        // sysSer.write(mOutVals, 8);
      }
    } else {
      stopMotors();
    }
    if (spew){
      Serial.write(currTimeBytes, 4);
      for (j = 0; j < 6; ++j) Serial.write((byte*) outVals[j], 4);
    }

    // Serial.println((long) (micros() - startTime));
  }
}

void stopMotors(){
  for (j = 0; j < 4; ++j) {
    sysSer.print(mOutNames[j]);
    sysSer.print(0);
  }
}

double fixDeadZone(double in) {
  return in + deadZone*getSign(in);
}

int coerce(double in){
  if (in<-255){ return -255; }
  else if (in>255){ return 255; }
  else { return in; }
}

long long voltageToMotorShield(double voltage){
    return (long long) 255*(voltage/12.0);
}

double ticksToRadians(long ticks){
    return ticks*6.0*M_PI/1000.0;
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