// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

#define LED_PIN 13
bool blinkState = false;

const uint8_t accelRange=0; // explicitly sets the accelerometer to +/- 2g
const uint8_t gyroRange=0; // explicitly sets the gyroscope range to +/- 250 deg/s

double axGraw = 0, ayGraw = 0, azGraw = 0; // for storing scaled to gravity-factor accelerations
double axG = 0, ayG = 0, azG = 0; // ..., after low-pass

double gxDraw = 0, gyDraw = 0, gzDraw = 0; // for storing degree/second measurements

double roll, pitch, accRoll, accPitch;
const float compCoeff = 0.0; // coefficient of gyroscopic portion of complementary filter estimation of Roll and Pitch
const float rollOffset = 1.5, pitchOffset = 6.4;
const float alpha = 0.05;

int sampleTime = 0;
long long lastSampleMark = micros();

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400); //57600

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    accelgyro.setFullScaleAccelRange(accelRange);
    accelgyro.setFullScaleGyroRange(gyroRange);

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getAcceleration(&ax, &ay, &az);
    accelgyro.getRotation(&gx, &gy, &gz);
    sampleTime = micros()-lastSampleMark;
    lastSampleMark = micros();

    // convert to deg/s
    gxDraw = val2dps(gx)/1000;
    gyDraw = val2dps(gy)/1000;
    gzDraw = val2dps(gz)/1000;


    // convert to g-factor
    axGraw = val2g(ax);
    ayGraw = val2g(ay);
    azGraw = val2g(az);

    // preliminary calculations
    accRoll  = (-(atan2(ayG, -azG)*180.0)/M_PI) - rollOffset;
    Serial.println(accRoll);
    accPitch = ((atan2(axG, sqrt(ayG*ayG + azG*azG))*180.0)/M_PI) - pitchOffset;

    roll = compCoeff*(roll + gxDraw*sampleTime) + (1-compCoeff)*(accRoll);
    pitch = compCoeff*(pitch + gyDraw*sampleTime) + (1-compCoeff)*(accPitch);


    // Serial.print("a/g/m:\t");
    // Serial.print(ax); Serial.print("\t");
    // Serial.print(ay); Serial.print("\t");
    // Serial.print(az); Serial.print("\t");
    // Serial.print(gx); Serial.print("\t");
    // Serial.print(gy); Serial.print("\t");
    // Serial.print(gz); Serial.print("\t");
    // Serial.print(mx); Serial.print("\t");
    // Serial.print(my); Serial.print("\t");
    // Serial.println(mz);

    

    // // low-pass filter
    // axG = axGraw * alpha + (axG * (1.0 - alpha));
    // ayG = ayGraw * alpha + (ayG * (1.0 - alpha));
    // azG = azGraw * alpha + (azG * (1.0 - alpha));

    // // calculate roll and pitch
    // roll  = (-(atan2(ayG, -azG)*180.0)/M_PI) - rollOffset;
    // pitch = ((atan2(axG, sqrt(ayG*ayG + azG*azG))*180.0)/M_PI) - pitchOffset;

 
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(", ");
    Serial.print(accRoll);
    Serial.print(" | ");
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(", ");
    Serial.println(accPitch);


    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

double val2g(int16_t acc){
    return double(acc)/16384.0; // currently set to +/- 2g
}

double val2dps(int16_t val){
    return double(val)/524.288; // currently set to +/- 250 deg/sec
}

double d2r(double d){
    return (d*M_PI)/180;
}
    
    // these methods (and a few others) are also available
    // accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

// int16_t rx, ry, rz; // for rotation testing

// int16_t ax_max = -32768, ay_max = -32768, az_max = -32768;
// int16_t ax_min = 32767, ay_min = 32767, az_min = 32767;

    // display tab-separated accel/gyro x/y/z values
    // if (az < az_min){
    //     az_min = az;
    // }
    // azG = val2g(az);
    // Serial.print(az);
    // Serial.print(" | ");
    // Serial.print(azG, 3);
    // Serial.print(" | ");
    // Serial.println(az_min);

    // if (ax < ax_min){
    //     ax_min = ax;
    // }
    // axG = val2g(ax);
    // Serial.print(ax);
    // Serial.print(" | ");
    // Serial.print(axG);
    // Serial.print(" | ");
    // Serial.println(ax_min);

    // Serial.print("a/g/m:\t");
    // Serial.print(ax); Serial.print("\t");
    // Serial.print(ay); Serial.print("\t");
    // Serial.print(az); Serial.print("\t");
    // Serial.print(gx); Serial.print("\t");
    // Serial.print(gy); Serial.print("\t");
    // Serial.print(gz); Serial.print("\t");
    // Serial.print(mx); Serial.print("\t");
    // Serial.print(my); Serial.print("\t");
    // Serial.println(mz);

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output

/* ============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/