#include <EEPROM.h>
#include <Encoder.h>
#include <Wire.h> // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// misc variables
// #define DEBUG
bool blinkState = false;
unsigned char loopDuration = 10; // loop should last as close to x milliseconds as possible
unsigned long nextLoop = 0;
unsigned char j; // loop counter
static union {
  unsigned long currTime;
  byte currTimeBytes[4];
};



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

// controller variables
const unsigned char numC = 7;
double constants[numC];
const char inChars[] = {'a', 'b', 'c', 'd', 'e', 'k', 'p'}; // a, b, c, d, e, k_p, p

double pitchPsi = 0;
double pitchPsiSet = 0;
double pitchPsiError = 0;
double pitchPhi = 0;
double pitchPhiError = 0;
double pitchVp = 0;
double pitchVpLast1 = 0;
double pitchVpLast2 = 0;
double pitchVv = 0;
double pitchVa = 0;
double pitchPhiErrorLast1 = 0;
double pitchPhiErrorLast2 = 0;

double rollPsi = 0;
double rollPsiSet = 0;
double rollPsiError = 0;
double rollPhi = 0;
double rollPhiError = 0;
double rollVp = 0;
double rollVpLast1 = 0;
double rollVpLast2 = 0;
double rollVv = 0;
double rollVa = 0;
double rollPhiErrorLast1 = 0;
double rollPhiErrorLast2 = 0;

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
        for (j = 0; j < numC; ++j) { Serial.print(constants[j], 5); Serial.print(" "); }
        Serial.println();
      }
    }

    // Serial.print(roll, 5);
    // Serial.print(' ');
    // Serial.println(pitch, 5);

    // calculate phi errors
    rollPhiError = 0 - rollPhi;
    pitchPhiError = 0 - pitchPhi;

    // calculate psi and psi error
    rollPsi = ticksToRadians(long((-enc0.read()-enc2.read())/2));
    rollPsiError = rollPsi - rollPsiSet;
    pitchPsi = ticksToRadians(long((enc1.read()+enc3.read())/2));
    pitchPsiError = pitchPsi - pitchPsiSet;

    // calculate control signals
    // rollVp = constants[3]*rollVpLast1 - constants[4]*rollVpLast2 - constants[0]*rollPhiError + constants[1]*rollPhiErrorLast1 - constants[2]*rollPhiErrorLast2; // double lag
    // pitchVp = constants[3]*pitchVpLast1 - constants[4]*pitchVpLast2 - constants[0]*pitchPhiError + constants[1]*pitchPhiErrorLast1 - constants[2]*pitchPhiErrorLast2;
    // rollVp = constants[2]*rollVpLast1 - constants[0]*rollPhiError + constants[1]*rollPhiErrorLast1; // single lag
    // pitchVp = constants[2]*pitchVpLast1 - constants[0]*pitchPhiError + constants[1]*pitchPhiErrorLast1;
    // rollVv = rollVp + constants[3]*rollPsiError; // single or double lag
    // rollVa = rollVv;
    // pitchVv = pitchVp + constants[3]*pitchPsiError;
    // pitchVa = pitchVv;
    rollVa = constants[6]*rollPhi; // proportional
    pitchVa = constants[6]*pitchPhi;

    // save current values for next loop
    rollPhiErrorLast2 = rollPhiErrorLast1;
    rollVpLast2 = rollVpLast1;
    pitchPhiErrorLast2 = pitchPhiErrorLast1;
    pitchVpLast2 = pitchVpLast1;
    rollPhiErrorLast1 = rollPhiError;
    rollVpLast1 = rollVp;
    pitchPhiErrorLast1 = pitchPhiError;
    pitchVpLast1 = pitchVp;

    // set motor control value array, doing sign conversion for motor orientation correction
    mOutVals[0] = coerce(voltageToMotorShield(-rollVa));
    mOutVals[2] = coerce(voltageToMotorShield(rollVa));
    mOutVals[1] = coerce(voltageToMotorShield(pitchVa));
    mOutVals[3] = coerce(voltageToMotorShield(-pitchVa));

    // proSer.print(roll, 5);
    // proSer.print(' ');
    // proSer.println(pitch, 5);

    // proSer.print(rollVp, 5);
    // proSer.print(' ');
    // proSer.println(pitchVp, 5);

    // proSer.print(rollVa);
    // proSer.print(' ');
    // proSer.println(pitchVa);
    
    for (j = 0; j < 4; ++j) {
      sysSer.print(mOutNames[j]);
      sysSer.print(mOutVals[j]);
      // sysSer.write(mOutVals, 8);
    }

    Serial.write(currTimeBytes, 4);
    for (j = 0; j < 6; ++j) Serial.write((byte*) outVals[j], 4);

    // Serial.println((long) (micros() - startTime));
  }
}

void stopMotors(){
  for (j = 0; j < 4; ++j) {
    sysSer.print(mOutNames[j]);
    sysSer.print(0);
  }
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