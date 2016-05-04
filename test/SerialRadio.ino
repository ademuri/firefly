#include "Arduino.h"
#include "../cc1101/cc1101.h"

CC1101 cc;

//The setup function is called once at startup of the sketch
void setup() {
	Serial.begin(9600);
	//Serial.println("begin");

	cc.init();
	cc.setCarrierFreq(CFREQ_433);

	// Receive packets to all addresses
	cc.disableAddressCheck();

	// Go for maximum range
	cc.setTxPowerAmp(PA_LongDistance);

	/*Serial.print("CC1101_PARTNUM "); //cc1101=0
	Serial.println(cc.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
	Serial.print("CC1101_VERSION "); //cc1101=4
	Serial.println(cc.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
	Serial.print("CC1101_MARCSTATE ");
	Serial.println(cc.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f);*/

	cc.setRxState();
}

CCPACKET packet;
byte readArray[256];
// The loop function is called in an endless loop
void loop() {
	if (cc.receiveData(&packet)) {
		for (int i=0; i<packet.length; i++) {
			Serial.print(packet.data[i], DEC);
			if (i < packet.length-1) {
				Serial.print(",");
			}
		}
		Serial.println();
	}

	char in = 0;
	String readString = "";
	byte numRead = 0;
	if (Serial.available()) {
		byte totalRead = Serial.readBytesUntil('\n', readArray, 256);

		for (int i=0; i<totalRead; i++) {
			in = readArray[i];
			if (isDigit(in)) {
				readString += in;
			} else if (in == ',' || isSpace(in)) {
				if (readString.length()) {
					packet.data[numRead] = (byte)readString.toInt();
					packet.length = ++numRead;
					readString = "";
				}
			} else {
				Serial.print("Got invalid digit: ");
				Serial.print(in);
				Serial.println();
			}
		}
		if (readString.length()) {
			packet.data[numRead] = (byte)readString.toInt();
			packet.length = ++numRead;
			readString = "";
		}

		/*Serial.print("Sending: ");
		for (int i=0; i<packet.length; i++) {
			Serial.print(packet.data[i], DEC);
			if (i < packet.length-1) {
				Serial.print(",");
			}
		}
		Serial.println();*/
		cc.sendData(packet);
		cc.sendData(packet);
	}
}
