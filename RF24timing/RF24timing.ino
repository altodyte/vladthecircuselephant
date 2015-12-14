#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// Become the primary transmitter
#define TX true

// Set up nRF24L01 radio on SPI bus plus chip enable (9) & slave select (UNO:10; MEGA:53)
RF24 radio(9,SS);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

float num = 0;
bool done = false;

void setup(void) {
  Serial.begin(250000);

  radio.begin();
  radio.setPayloadSize(32);
  radio.setRetries(0, 0);
  radio.setDataRate(RF24_2MBPS);

  #if TX
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
  #else
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
    radio.startListening();
  #endif
}

void loop(void) {
  #if TX
    // Take the num, and send it.  This will block until complete
    num = micros();
    done = false;
    while (!done) { // try sending until it's successful
      done = radio.write( &num, sizeof(float) );
    }
    Serial.print("Sent payload "); Serial.println(num/1.0);
  #else
    // if there's a payload available to get
    if ( radio.available() )
    {
      done = false;
      while (!done) { // try sending until it was successful
        done = radio.read( &num, sizeof(float) );
      }
      Serial.print("Got payload "); Serial.println(num);
    }
  #endif
}
