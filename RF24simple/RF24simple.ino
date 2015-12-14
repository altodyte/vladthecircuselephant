#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// Become the primary transmitter
#define TX false

// Set up nRF24L01 radio on SPI bus plus chip enable (9) & slave select (UNO:10; MEGA:53)
RF24 radio(9,SS);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

// Random variables
bool done = false;

// Radio packet structure (PKT_SZ must match total size of PacketData_t)
#define PKT_SZ 9
typedef struct PacketData_t {
  char name;
  double value;
};
typedef union RadioPacket_t {
  PacketData_t members;
  byte bytes[PKT_SZ];
};
RadioPacket_t packet;

void setup(void) {
  Serial.begin(250000);
  Serial.setTimeout(1);

  radio.begin();
  radio.setPayloadSize(PKT_SZ);
  radio.setRetries(0, 0);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_HIGH);

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
    // If there's Serial data ready to get
    if ( Serial.available() ) {
      radio.powerUp();
      // Pull data from Serial and put it into packet
      packet.members.name = Serial.read();
      packet.members.value = Serial.parseFloat();
      // Send packet over radio
      done = false;
      // int i = 0;
      // long start = millis();
      while (!done) { // try sending until it's successful
        // ++i;
        done = radio.write( packet.bytes, PKT_SZ );
      }
      // long end = millis();
      // Serial.print(end-start); Serial.print(" "); Serial.println(i);
      // Serial.println(end-start);
      Serial.print("Sent payload "); Serial.print(packet.members.name); Serial.println(packet.members.value);
    }
  #else
    // If there's a payload ready to get
    if ( radio.available() ) {
      done = false;
      while (!done) { // try sending until it's successful
        done = radio.read( &packet, PKT_SZ );
      }
      Serial.print(packet.members.name); Serial.println(packet.members.value, 5);
      // Serial.print("Got payload "); Serial.print(packet.members.name); Serial.println(packet.members.value);
    }
  #endif
}
