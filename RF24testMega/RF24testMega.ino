#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// Become the primary transmitter (ping out)
#define TX false

// Set up nRF24L01 radio on SPI bus plus chip enable (9) & slave select (UNO:10; MEGA:53)
RF24 radio(9,SS);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

void setup(void) {
  Serial.begin(250000);

  radio.begin();

  #if TX
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
  #else
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
  #endif

  radio.startListening();
}

void loop(void) {
  #if TX
  {
    // First, stop listening so we can talk.
    radio.stopListening();

    // Take the time, and send it.  This will block until complete

    float time = millis();
    Serial.print("Now sending "); Serial.println(time/1.0);
    bool ok = radio.write( &time, sizeof(float) );

    // Now, continue listening
    radio.startListening();

    // Wait here until we get a response, or timeout (250ms)
    float started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > 200)
        timeout = true;

    // Describe the results
    if ( timeout )
      Serial.println("Response timed out");
    else {
      // Grab the response, compare, and send to debugging spew
      float got_time;
      radio.read( &got_time, sizeof(float) );
      Serial.print("Got response "); Serial.println(got_time);
    }

    // Try again 1s later
    // delay(100);
  }
  #else
  {
    // if there is data ready
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      float got_time;
      bool done = false;
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
        done = radio.read( &got_time, sizeof(float) );
        Serial.print("Got payload "); Serial.println(got_time);

        // Delay just a little bit to let the other unit make the transition to receiver
        delay(20);
      }
      // First, stop listening so we can talk
      radio.stopListening();

      // Send the final one back.
      radio.write( &got_time, sizeof(float) );
      Serial.println("Sent response");

      // Now, resume listening so we catch the next packets.
      radio.startListening();
    }
  }
  #endif
}
