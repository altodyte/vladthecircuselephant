#include <Wire.h> // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

/******************************************************************************
************** OBJECTS AND PIN-DEPENDENT DECLARATIONS *************************
******************************************************************************/

#define LED_PIN 13
bool blinkState = false;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor* motors[4];
// Adafruit_DCMotor *motor0 = AFMS.getMotor(1);
// Adafruit_DCMotor *motor1 = AFMS.getMotor(2);
// Adafruit_DCMotor *motor2 = AFMS.getMotor(3);
// Adafruit_DCMotor *motor3 = AFMS.getMotor(4);

HardwareSerial & masterSer = Serial;
char inChar;

int mnew[4];
int mprev[4];
unsigned int maxdiff;
unsigned int diff;
unsigned char maxj;
unsigned char j;

void setup() {
  for (j = 0; j < 4; ++j) {
    motors[j] = AFMS.getMotor(j+1);
  }
  // initialize serial communication
  masterSer.setTimeout(5);
  masterSer.begin(250000);

  // configure Arduino LED for heartbeat
  pinMode(LED_PIN, OUTPUT);

  // Initialize motor shield
  // Serial.println("Initializing AFMS I2C device...");
  AFMS.begin();
  fuck(); // kills motors
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

  // for (j = 0; j < 4; ++j) {
  //   Serial.print(mnew[j]);
  //   Serial.print(' ');
  // }
  // Serial.println();
  
  // for (j = 0; j < 4; ++j) {
  //   Serial.print(mprev[j]);
  //   Serial.print(' ');
  // }
  // Serial.println();

  maxj = 0;
  maxdiff = 0;
  for (j = 0; j < 4; ++j) {
    diff = abs(mnew[j] - mprev[j]);
    if (diff > maxdiff) {
      maxj = j;
      maxdiff = diff;
    }
  }

  // Serial.println(maxj);
  // Serial.println(maxdiff);

  // takes 2 ms to write new speed to motors
  if (maxdiff != 0) {
    mprev[maxj] = mnew[maxj];
    mset(motors[maxj], mnew[maxj]);
  }

}

void fuck(){
  // Serial.println("KILL MOTORS");
  for (j = 0; j < 4; ++j) mset(motors[j], 0);
}

void mset(Adafruit_DCMotor* m, int i){
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