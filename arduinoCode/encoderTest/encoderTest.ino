#include <Encoder.h>
#include <Wire.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder enc0(2,10);
Encoder enc1(3,12);
Encoder enc2(18,16);
Encoder enc3(19,17);
//   avoid using pins with LEDs attached

long enc0old  = -999;
long enc1old = -999;
long enc2old = -999;
long enc3old = -999;
long enc0new = enc0.read();
long enc1new = enc1.read();
long enc2new = enc2.read();
long enc3new = enc3.read();

long e0lastTime = millis();
long e1lastTime = millis();
long e2lastTime = millis();
long e3lastTime = millis();
long e0thisTime = millis();
long e1thisTime = millis();
long e2thisTime = millis();
long e3thisTime = millis();

double scaleFactor = 1000.0;
double e0deriv = 0;
double e1deriv = 0;
double e2deriv = 0;
double e3deriv = 0;

long loopTime = 100;
long nextLoop;

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  long nextLoop = millis();
}

void loop() {
  updateEncoders();
  if (millis()>nextLoop){
    calculateDerivatives();
    storeEncoders();
    printEncoders();
    printDerivs();
    nextLoop = millis()+loopTime;
  }
}

void updateEncoders(){
  enc0new = enc0.read();
  e0thisTime = millis();
  enc1new = enc1.read();
  e1thisTime = millis();
  enc2new = enc2.read();
  e2thisTime = millis();
  enc3new = enc3.read();
  e3thisTime = millis();
}

void storeEncoders(){
  if (enc0new != enc0old) {
    enc0old = enc0new;
    e0lastTime = millis();
  }
  if (enc1new != enc1old) {
    enc1old = enc1new;
    e1lastTime = millis();
  }
  if (enc2new != enc2old) {
    enc2old = enc2new;
    e2lastTime = millis();
  }
  if (enc3new != enc3old) {
    enc3old = enc3new;
    e3lastTime = millis();
  }
}

void calculateDerivatives(){
  // call updateEncoders first, and storeEncoders after
  e0deriv = scaleFactor*(enc0new - enc0old)/(e0thisTime-e0lastTime);
  e1deriv = scaleFactor*(enc1new - enc1old)/(e1thisTime-e1lastTime);
  e2deriv = scaleFactor*(enc2new - enc2old)/(e2thisTime-e2lastTime);
  e3deriv = scaleFactor*(enc3new - enc3old)/(e3thisTime-e3lastTime);
}

void printEncoders(){
  Serial.print(enc0old);

  Serial.print(", ");
  Serial.print(enc1old);

  Serial.print(", ");
  Serial.print(enc2old);

  Serial.print(", ");
  Serial.print(enc3old);
}

void printDerivs(){
  Serial.print(" | ");
  Serial.print(e0deriv);

  Serial.print(", ");
  Serial.print(e1deriv);

  Serial.print(", ");
  Serial.print(e2deriv);

  Serial.print(", ");
  Serial.println(e3deriv);
}