
HardwareSerial & pcSer = Serial;
HardwareSerial & XBeeSer = Serial1;

void setup() {
  // initialize serial communication
  pcSer.begin(115200);
  XBeeSer.begin(125000);
}

void loop() {
	while (pcSer.available()) { pcSer.write(pcSer.read()); delay(5); }

	// while (pcSer.available()) { XBeeSer.write(pcSer.read()); delay(5); }

	// while (XBeeSer.available()) { pcSer.write(XBeeSer.read()); delay(5); }

}
