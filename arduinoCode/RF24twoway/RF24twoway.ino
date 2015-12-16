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
bool newCommand = false; // is there a new command to send?
unsigned long started_waiting_at = 0; // time started waiting (for timeout detection)

// Radio packet structures (packet size must match total size of struct/union)
#define CMD_SZ 5
struct CommandMembers_t {
  char name;
  double value;
};
union CommandPacket_t {
  CommandMembers_t members;
  byte bytes[CMD_SZ];
} commandPacket;

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

void loop(void) {
  #if TX

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

      // Send command only if there's a new one (otherwise, send nothing and let RX timeout)
      if (newCommand) {
        radio.stopListening();

        // Wait a bit for RX to start listening (may not be necessary)
        // delay(1);

        // Wake up radio so it sends quickly
        radio.powerUp();
        // Try sending command until it's successful
        done = false;
        while (!done) {
          done = radio.write(commandPacket.bytes, CMD_SZ);
        }
        newCommand = false;
        // Serial.print("Sent command ");
        // Serial.print(commandPacket.members.name);
        // Serial.println(commandPacket.members.value);

        // Now, resume listening so we catch the next data packet
        radio.startListening();
      }
    }
    // There's a command packet ready to be sent
    if (Serial.available()) {
      // Pull data from Serial and put it into packet
      commandPacket.members.name = Serial.read();
      commandPacket.members.value = Serial.parseFloat();
      newCommand = true;
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

      // Now, resume listening so we catch the next command packet
      radio.startListening();

      // Wait here until we get a response, or timeout (3 ms)
      started_waiting_at = millis();
      bool timeout = false;
      while ( !radio.available() && !timeout )
        if (millis() - started_waiting_at > 3)
          timeout = true;

      // Receive command as bytes, then send as ASCII
      if (!timeout) {
        radio.read(commandPacket.bytes, CMD_SZ);
        // Serial.print("Received command ");
        Serial.print(commandPacket.members.name);
        Serial.println(commandPacket.members.value);
      }

      // Stop listening so we can send quickly when new data comes in
      radio.stopListening();
    }

  #endif
}

      // int i = 0;
      // long start = millis();
      // long end = millis();
      // Serial.print(end-start); Serial.print(" "); Serial.println(i);
      // Serial.println(end-start);