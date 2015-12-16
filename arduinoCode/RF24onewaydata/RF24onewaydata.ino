#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// Become the command sender/data receiver
#define TX true
// Become the command receiver/data sender
// #define TX false

// Set up nRF24L01 radio on SPI bus plus chip enable (9) & slave select (UNO:10; MEGA:53)
RF24 radio(9, SS);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

// Random variables
bool done = false; // are we done sending?

// Radio packet structures (packet size must match total size of struct/union)
#define DATA_SZ 28
struct DataMembers_t {
  unsigned long time;
  double pitchPsi;
  double pitchPhi;
  double pitchVa;
  double rollPsi;
  double rollPhi;
  double rollVa;
};
union DataPacket_t {
  DataMembers_t members;
  byte bytes[DATA_SZ];
} dataPacket;

void setup(void) {
  // We assume that all serial communications are at 250 kbaud and uninterrupted
  Serial.begin(250000);
  Serial.setTimeout(1);

  radio.begin();
  radio.setPayloadSize(DATA_SZ);
  radio.setRetries(0, 0);
  radio.setCRCLength(RF24_CRC_16);
  radio.setChannel(53);
  // radio.setDataRate(RF24_1MBPS);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_HIGH);

  #if TX
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
    radio.startListening();
  #else
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
  #endif
}

unsigned char vals[128] = {};

void loop(void) {
  #if TX

    // for (int k = 0; k < 128; ++k) {
    //   radio.setChannel(k);
    //   delay(10);
    //   vals[k] += radio.testRPD();
    //   Serial.print(k);
    //   Serial.print(" ");
    //   Serial.println(vals[k]);
    // }

    // If there's a data packet ready to get
    if (radio.available()) {
      // Try reading data until it's successful
      done = false;
      while (!done) {
        done = radio.read(dataPacket.bytes, DATA_SZ);
      }
      // Write binary data packet to Serial
      Serial.write(dataPacket.bytes, DATA_SZ);

      // Write bytes of data packet to Serial as ASCII
      // for (int i = 0; i < DATA_SZ; ++i) {
      //   Serial.print(dataPacket.bytes[i]);
      //   Serial.print(" ");
      // }
      // Serial.println();

      // Display timestamp of data packet we just received
      // Serial.print("Data received from ");
      // Serial.println(dataPacket.members.time);

      // Display (as ASCII) data (received as binary)
      // Serial.print("Data ");
      // Serial.print(dataPacket.members.pitchPsi);
      // Serial.print(" ");
      // Serial.print(dataPacket.members.pitchPhi);
      // Serial.print(" ");
      // Serial.print(dataPacket.members.pitchVa);
      // Serial.print(" ");
      // Serial.print(dataPacket.members.rollPsi);
      // Serial.print(" ");
      // Serial.print(dataPacket.members.rollPhi);
      // Serial.print(" ");
      // Serial.print(dataPacket.members.rollVa);
      // Serial.print(" received from ");
      // Serial.println(dataPacket.members.time);
    }

  #else

    // If there is data ready to be sent
    if (Serial.available()) {
      // Pull binary data from Serial and put it into packet
      Serial.readBytes(dataPacket.bytes, DATA_SZ);

      // Pull ASCII data from Serial and put it w/timestamp into packet
      // dataPacket.members.time = millis();
      // dataPacket.members.pitchPsi = Serial.parseFloat();
      // Serial.read();
      // dataPacket.members.pitchPhi = Serial.parseFloat();
      // Serial.read();
      // dataPacket.members.pitchVa = Serial.parseFloat();
      // Serial.read();
      // dataPacket.members.rollPsi = Serial.parseFloat();
      // Serial.read();
      // dataPacket.members.rollPhi = Serial.parseFloat();
      // Serial.read();
      // dataPacket.members.rollVa = Serial.parseFloat();

      // Wake up radio so it sends quickly
      radio.powerUp();
      // Try sending data until it's successful
      done = false;
      while (!done) {
        done = radio.write(dataPacket.bytes, DATA_SZ);
      }
    }

  #endif
}