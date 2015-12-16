
HardwareSerial & pcSer = Serial;
// HardwareSerial & XBeeSer = Serial1;

void setup() {
  // initialize serial communication
  pcSer.begin(250000);
  Serial3.begin(125000);
  // XBeeSer.begin(125000);
}

void loop() {
	while (pcSer.available()) { Serial3.write(pcSer.read()); }
	while (Serial3.available()) { pcSer.write(Serial3.read()); }

	// while (Serial1.available()) { Serial1.write(Serial1.read()); delay(5); }

	// while (pcSer.available()) { XBeeSer.write(pcSer.read()); delay(5); }

	// while (XBeeSer.available()) { pcSer.write(XBeeSer.read()); delay(5); }

}
