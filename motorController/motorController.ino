#include <Wire.h> // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define LED_PIN 13
bool blinkState = false;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor* motors[4];

HardwareSerial & masterSer = Serial;
char inChar;

int mnew[4];
int mprev[4];
unsigned int maxdiff;
unsigned int diff;
unsigned char maxj;
unsigned char j;

void setup() {
  // create four motor objects
  for (j = 0; j < 4; ++j) {
    motors[j] = AFMS.getMotor(j+1);
  }
  // initialize serial communication
  masterSer.setTimeout(5);
  masterSer.begin(250000);

  // configure Arduino LED for heartbeat
  pinMode(LED_PIN, OUTPUT);

  // Initialize motor shield
  AFMS.begin();
  stopMotors(); // kills motors
}

long long startTime = 0;

void loop() {
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  // get commands to send to motors from master Arduino
  while (masterSer.available() > 0) {
    inChar = masterSer.read();
    if (inChar == 'a') mnew[0] = masterSer.parseInt();
    else if (inChar == 'b') mnew[1] = masterSer.parseInt();
    else if (inChar == 'c') mnew[2] = masterSer.parseInt();
    else if (inChar == 'd') mnew[3] = masterSer.parseInt();
  }

  // print previous motor values
  // for (j = 0; j < 4; ++j) {
  //   Serial.print(mprev[j]);
  //   Serial.print(' ');
  // }
  // Serial.println();

  // print new (requested) motor values)
  // for (j = 0; j < 4; ++j) {
  //   Serial.print(mnew[j]);
  //   Serial.print(' ');
  // }
  // Serial.println();

  // find which motor's current value varies most from its new value
  maxj = 0;
  maxdiff = 0;
  for (j = 0; j < 4; ++j) {
    diff = abs(mnew[j] - mprev[j]);
    if (diff > maxdiff) {
      maxj = j;
      maxdiff = diff;
    }
  }

  // print which motor was found to have the greatest difference
  // Serial.print(maxj);
  // Serial.print(" ");
  // Serial.println(maxdiff);

  // set motor which has the largest requested change in speed
  if (maxdiff != 0) {
    mprev[maxj] = mnew[maxj];
    mset(motors[maxj], mnew[maxj]);
  }
}

void stopMotors() {
  for (j = 0; j < 4; ++j) mset(motors[j], 0);
}

// takes 2 ms to write new speed to motors
void mset(Adafruit_DCMotor* m, int i) {
  if (i==0) {
    m->run(RELEASE);
    m->setSpeed(0);
  } else if (i<0) {
    m->setSpeed(abs(i));
    m->run(BACKWARD);
  } else {
    m->setSpeed(i);
    m->run(FORWARD);
  }
}