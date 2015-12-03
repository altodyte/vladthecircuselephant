#include <Encoder.h>
#include <Wire.h> // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

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

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motor0 = AFMS.getMotor(1);
Adafruit_DCMotor *motor1 = AFMS.getMotor(2);
Adafruit_DCMotor *motor2 = AFMS.getMotor(3);
Adafruit_DCMotor *motor3 = AFMS.getMotor(4);

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here -- AD0 low = 0x68, AD0 high = 0x69
MPU6050 accelgyro;

/******************************************************************************
********************************* IMU State Variables *************************
******************************************************************************/

const uint8_t accelRange=0; // explicitly sets the accelerometer to +/- 2g

int16_t ax, ay, az;
int16_t gx, gy, gz;
double axGraw = 0, ayGraw = 0, azGraw = 0; // for storing scaled to gravity-factor accelerations
double axG = 0, ayG = 0, azG = 0; // ..., after low-pass

double roll = 0, pitch = 0;
double rollError = 0, pitchError = 0;
const float rollOffset = 2.5, pitchOffset = 5.4;

const float alpha = 0.99; // "low pass filter" coefficient -- lower = more included in rolling average

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

int loopDuration = 20; // loop should last as close to 20 milliseconds as possible
long long nextLoop = 0;

// Fancy controller
// double constantC = 0.995;
// double constantA = -56.25;
// double constantB = 51.76;

// double constantA = -41.67;
// double constantB = 38.34;
// double constantC = 0.9967;

// double constantA = -333.33;
// double constantB = 323.3;
// double constantC = 0.998;

double constantA = -133.33;
double constantB = 129.34;
double constantC = 0.998;


double voltageIntoPitchPositionFeedback = 0;
double previousVoltageIntoPitchPositionFeedback = voltageIntoPitchPositionFeedback;
double previousPitchError = pitchError;
double kayPee = 1; //1.0/3.0;
double pitchPsi = 0;

double voltageIntoRollPositionFeedback = 0;
double previousVoltageIntoRollPositionFeedback = voltageIntoRollPositionFeedback;
double previousRollError = rollError;
double rollPsi = 0;


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(250000); //57600

    // initialize IMU device
    Serial.println("Initializing IMU I2C device...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing IMU device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // set sensor scale factor for 16-bit output
    accelgyro.setFullScaleAccelRange(accelRange);

    // configure Arduino LED for heartbeat
    pinMode(LED_PIN, OUTPUT);

    // Initialize motor shield
    Serial.println("Initializing AFMS I2C device...");
    AFMS.begin();
    fuck(); // kills motors

    nextLoop = millis()+20;
}

void loop() {
    if (millis()>nextLoop){
        nextLoop+=loopDuration;

        // read raw accel measurements from device -- reading other sensors blocks too long
        accelgyro.getAcceleration(&ax, &ay, &az);

        // convert to g-scale
        axGraw = val2g(ax);
        ayGraw = val2g(ay);
        azGraw = val2g(az);

        // low-pass filter
        axG = axGraw * alpha + (axG * (1.0 - alpha));
        ayG = ayGraw * alpha + (ayG * (1.0 - alpha));
        azG = azGraw * alpha + (azG * (1.0 - alpha));

        // calculate roll and pitch
        roll  = (-(atan2(ayG, -azG)*180.0)/M_PI) - rollOffset;
        pitch = ((atan2(axG, sqrt(ayG*ayG + azG*azG))*180.0)/M_PI) - pitchOffset;

        // calculate roll and pitch errors
        rollError = 0 - roll;
        pitchError = 0 - pitch;

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        // handle control of pitch axis
        // if (pitch != coerce(pitch,-killZone,killZone)){
        //     // kill motors if we're doomed to fall
        //     fuck();
        //     delay(1000);
        // } if (roll != coerce(roll,-killZone,killZone)){
        //     // kill motors if we're doomed to fall
        //     fuck();
        //     delay(1000);
        // } else {

        if (true){
            // rollError = d2r(rollError);
            // pitchError = d2r(pitchError);
            pitchPsi = ticksToRadians(long((-enc1.read()+enc3.read())/2));
            voltageIntoPitchPositionFeedback = constantC*previousVoltageIntoPitchPositionFeedback + constantA*pitchError + constantB*previousPitchError;
            pitchActuators = voltageToMotorShield(voltageIntoPitchPositionFeedback + kayPee*pitchPsi);
            previousPitchError = pitchError;
            previousVoltageIntoPitchPositionFeedback = voltageIntoPitchPositionFeedback;
            m1set(pitchActuators);
            m3set(-pitchActuators);

            rollPsi = ticksToRadians(long((-enc0.read()+enc2.read())/2));
            voltageIntoRollPositionFeedback = constantC*previousVoltageIntoRollPositionFeedback + constantA*rollError + constantB*previousRollError;
            rollActuators = voltageToMotorShield(voltageIntoRollPositionFeedback + kayPee*rollPsi);
            previousRollError = rollError;
            previousVoltageIntoRollPositionFeedback = voltageIntoRollPositionFeedback;
            m0set(-rollActuators);
            m2set(rollActuators);

            // pitchActuators = modify(proportionalConstant*pitchError);
            // m1set(-pitchActuators);
            // m3set(pitchActuators);

            // rollActuators = modify(proportionalConstant*rollError);
            // m0set(rollActuators);
            // m2set(-rollActuators);

            Serial.print("Pitch: ");
            Serial.print(pitch);
            Serial.print(" || ");
            Serial.print("M2: ");
            Serial.print(-pitchActuators);
            Serial.print(" | ");
            Serial.print("M4: ");
            Serial.print(pitchActuators);
            Serial.print(" ||| ");
            Serial.print("Roll: ");
            Serial.print(roll);
            Serial.print(" || ");
            Serial.print("M1: ");
            Serial.print(rollActuators);
            Serial.print(" | ");
            Serial.print("M3: ");
            Serial.println(-rollActuators);
        }
    }
}

void fuck(){
    Serial.println("KILL MOTORS");
    m0set(0);
    m1set(0);
    m2set(0);
    m3set(0);
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

int modify(int in){
  int deadZoneMin = -deadZone, deadZoneMax = deadZone;
  if ((in>deadZoneMin)&&(in<deadZoneMax)){
    return 0;
  }
}

int voltageToMotorShield(double voltage){
    return int(voltage/255);
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