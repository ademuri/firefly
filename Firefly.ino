/*
 *  Firefly - Arduino software that drives an LED. It uses a CC1101 to sync between nodes,
 *  and a microphone to sync to music.
 *
 *  Copyright (C) 2016 Adam Demuri
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Arduino.h"

#include "FreeRTOS/Arduino_FreeRTOS.h"
#include "cc1101/cc1101.h"
#include "queue.h"

#include "Led.h"

#define DEBUG

// Use an unconnected analog pin to seed the random number generator
#define ANALOG_RANDOM_PIN 1
#define LED_R 3 // ATMega328P pin 5
#define LED_G 5 // ATMega328P pin 11
#define LED_B 6 // ATMega328P pin 12


const unsigned long INIT_SEARCH_TIME_MILLIS = 5000;
const unsigned long MASTER_HEARTBEAT_INTERVAL = 1000;
const unsigned long HEARTBEAT_DURATION = 100;
const unsigned long MASTER_SEARCH_TIME = 2000;
const unsigned long PING_WAIT = 1000;

// Convert the constants above to ticks
const unsigned long HEARTBEAT_DURATION_TICKS = HEARTBEAT_DURATION / portTICK_PERIOD_MS;

// How long in slave mode without any heartbeats before becoming a master
const unsigned long SLAVE_TIMEOUT = 5000;

enum State {
	INIT,
	SLAVE,
	MASTER,
	MASTER_SELECTION,
};

enum PacketType {
	UNKNOWN,
	HEARTBEAT,
	PING,
	PING_RESPONSE,
	CLAIM_MASTER,
	MASTER_NEGOTIATE_ANNOUNCE,	// Master brags about how many visible nodes
};

CC1101 cc;
QueueHandle_t ledQueue;

State state = INIT;

// Amount of time we've been in the init state. When we start, listen for a few
// seconds to see if someone else is broadcasting. If not, start broadcasting
// (i.e. become a master). Once we're a master, every so often check to see if
// someone else is a master.
unsigned long initStarted = 0;

// In the master state, millis since we last broadcast a heartbeat
unsigned long masterLastHeartbeatTime = 0;

unsigned long slaveLastHeartbeat = 0;


// Outgoing-ping-related variables
unsigned long pingStartedTime = 0;
byte lastNonceLower = 0;
byte lastNonceUpper = 0;
byte lastPingCount = 0;
byte lastReceivedBrag = 0;
bool bragReceived = false;
bool bragSent = false;

byte lastIdLower = 0;
byte lastIdUpper = 0;

// Appends random ids and sends
void sendData(CCPACKET packet) {
	byte oldLength = packet.length;
	packet.length += 2;

	lastIdLower = random(0xFF);
	lastIdUpper = random(0xFF);

	packet.data[oldLength] = lastIdLower;
	packet.data[oldLength+1] = lastIdUpper;

	cc.sendData(packet);
	cc.sendData(packet);
}


byte receiveData(CCPACKET *packet) {
	if (cc.receiveData(packet) > 0) {
		if (packet->length < 2) {
#ifdef DEBUG
			Serial.println("Invalid packet!");
#endif
			return 0;
		}
		byte origLength = packet->length - 2;
		if (packet->data[origLength] != lastIdLower
				&& packet->data[origLength+1] != lastIdUpper) {
			packet ->length -= 2;
			lastIdLower = packet->data[origLength];
			lastIdUpper = packet->data[origLength+1];

#ifdef DEBUG
			// Dump the packet
			for (int i=0; i<packet->length; i++) {
				Serial.print(packet->data[i], DEC);
				if (i<packet->length-1) {
					Serial.print(",");
				}
			}
			Serial.println();
#endif

			return packet->length;
		}
	}
	return 0;
}

// Blinks a heartbeat for a preset amount of time.
void processHeartbeat(CCPACKET packet) {
	// TODO: actually use the packet
	static LedMsg onMsg = {255, 255, 255, HEARTBEAT_DURATION_TICKS};
	static LedMsg offMsg = {0, 0, 0, 1};
	xQueueSendToBack(ledQueue, &onMsg, 0);
	xQueueSendToBack(ledQueue, &offMsg, 0);
}

void processOwnHeartbeat(CCPACKET packet) {
	// TODO: actually use the packet
	static LedMsg onMsg = {255, 255, 255, 50};
	static LedMsg offMsg = {0, 0, 0, 50};

	xQueueSendToBack(ledQueue, &onMsg, 0);
	xQueueSendToBack(ledQueue, &offMsg, 0);
	xQueueSendToBack(ledQueue, &onMsg, 0);
	xQueueSendToBack(ledQueue, &offMsg, 0);
}

CCPACKET toSend;

// Heartbeat packet structure:
// 0: type (HEARTBEAT)
// 1: Red intensity
// 2: Green intensity
// 3: Blue intensity
void broadcastHeartbeat() {
	toSend.length = 4;
	toSend.data[0] = HEARTBEAT;

	// White
	toSend.data[1] = 0xFF;
	toSend.data[2] = 0xFF;
	toSend.data[3] = 0xFF;

	sendData(toSend);
	masterLastHeartbeatTime = millis();

	processOwnHeartbeat(toSend);
}

// Causes all other masters in range to become slaves. Structure:
// 0: type (CLAIM_MASTER)
void broadcastClaimMaster() {
	toSend.length = 1;
	toSend.data[0] = CLAIM_MASTER;

	sendData(toSend);
}

// Causes all reachable nodes to respond. Used to identify how many other nodes
// are reachable from this node (for master election). Uses a nonce to uniquely
// identify responses to this specific ping request. This kicks off the master
// selection process.
// 0: type (PING)
// 1: nonce 0
// 2: nonce 1
void broadcastPing() {
	toSend.length = 3;

	toSend.data[0] = PING;
	lastNonceLower = toSend.data[1] = random(0xFF);
	lastNonceUpper = toSend.data[2] = random(0xFF);

	sendData(toSend);

	pingStartedTime = millis();
	lastPingCount = 0;
	lastReceivedBrag = 0;
	bragReceived = false;
	bragSent = false;
}

// Responds to a ping, i.e. signals that mutual communication is possible.
// 0: type (PING_RESPONSE)
// 1: nonce 0
// 2: nonce 1
void respondToPing(CCPACKET originalPing) {
	toSend.length = 3;
	toSend.data[0] = PING_RESPONSE;
	toSend.data[1] = originalPing.data[1];
	toSend.data[2] = originalPing.data[2];

	sendData(toSend);
}

bool isMyPing(CCPACKET pingResponse) {
	return pingResponse.data[1] == lastNonceLower
			&& pingResponse.data[2] == lastNonceUpper;
}

// Probably ends master selection. Announces how many nodes we can see. Whoever has the higher
// number becomes master. If the numbers are equal, the last person to respond wins.
// 0: type (MASTER_NEGOTIATE_ANNOUNCE)
// 1: node count
void broadcastMasterNegotiateAnnounce() {
	toSend.length = 2;
	toSend.data[0] = MASTER_NEGOTIATE_ANNOUNCE;
	toSend.data[1] = lastPingCount;

	sendData(toSend);
}

byte getNegotiateAnnounceCount(CCPACKET brag) {
	return brag.data[1];
}

CCPACKET packet;
State prevState = INIT;

// The loop function is called in an endless loop
void mainLoop(void* params) {
	while (1) {
		switch (state) {
		case INIT:
			if (receiveData(&packet) > 0) {
				// Process all available packets
				do {
					switch(packet.data[0]) {
					case HEARTBEAT:
						state = SLAVE;
						slaveLastHeartbeat = millis();
						processHeartbeat(packet);
						break;
					case CLAIM_MASTER:
						state = SLAVE;
						break;
					case PING:
						respondToPing(packet);
						break;
					case PING_RESPONSE:
					case UNKNOWN:
					default:
						break;
					}
				} while (receiveData(&packet) > 0);
			}
			if (millis() > (initStarted + INIT_SEARCH_TIME_MILLIS)) {
#ifdef DEBUG
				Serial.println("No packets received - becoming master.");
#endif
				state = MASTER;
				broadcastClaimMaster();
				broadcastHeartbeat();
			}
			break;

		case MASTER:
			if (millis() > (masterLastHeartbeatTime + MASTER_HEARTBEAT_INTERVAL)) {
				broadcastHeartbeat();
			}
			if (receiveData(&packet) > 0) {
				switch(packet.data[0]) {
				case HEARTBEAT:
					// Two masters - time to do master selection!
					state = MASTER_SELECTION;
					broadcastPing();
					break;
				case PING:
					respondToPing(packet);
					break;
				case PING_RESPONSE:
					// If we're not doing master selection, we don't care about ping responses, so
					// ignore this.
				case UNKNOWN:
				case CLAIM_MASTER:
				case MASTER_NEGOTIATE_ANNOUNCE:
					// This case means we have two masters, but we want the timings to line up, so
					// wait for a heartbeat to start master selection.
				default:
					break;
				}
			}
			break;

		// TODO: this will probably break horribly if there are more than 2 nodes doing this at once
		case MASTER_SELECTION:
			if (receiveData(&packet) > 0) {
				switch(packet.data[0]) {
				case CLAIM_MASTER:
					// If someone else thinks they see more nodes than us, trust them and become a slave.
#ifdef DEBUG
					Serial.println("CLAIM_MASTER received, becoming slave");
#endif
					state = SLAVE;
					slaveLastHeartbeat = millis();
					break;
				case PING:
					respondToPing(packet);
					break;
				case PING_RESPONSE:
					// Note: we don't check whether the PING_WAIT time has expired here - we assume that
					// this loop is fast and it will get picked up below
					if (isMyPing(packet)) {
#ifdef DEBUG
						Serial.print("Received ping: ");
						lastPingCount++;
						Serial.println(lastPingCount, DEC);
#endif
					}
					break;
				case MASTER_NEGOTIATE_ANNOUNCE:
					bragReceived = true;
					lastReceivedBrag = getNegotiateAnnounceCount(packet);
					// TODO: if our count is already greater than this, don't wait for PING_WAIT to elapse,
					// and just become the master already.
					break;
				case HEARTBEAT:
				case UNKNOWN:
				default:
					break;
				}
			}

			if (state == SLAVE) {
				break;
			}

			// Still do the heartbeat when negotiating for a single master
			if (millis() > (masterLastHeartbeatTime + MASTER_HEARTBEAT_INTERVAL)) {
				broadcastHeartbeat();
			}

			if (millis() > (pingStartedTime + PING_WAIT)) {
				// If we've already received another brag, check it
				if (bragReceived) {
					if (lastReceivedBrag > lastPingCount) {
						// Become the slave
#ifdef DEBUG
						Serial.println("Received brag - becoming slave");
#endif
						state = SLAVE;
						slaveLastHeartbeat = millis();
					} else {
						// Become the master
						// Note: if the node counts are equal, the node to finish last (i.e. us, here)
						// becomes the master. This makes things simpler.
#ifdef DEBUG
						Serial.print("Received brag - becoming master: (");
						Serial.print(lastPingCount);
						Serial.print(",");
						Serial.print(lastReceivedBrag);
						Serial.println(")");
#endif
						state = MASTER;
						broadcastClaimMaster();
					}
				} else {
					if (!bragSent) {
						// Done collecting ping responses
						broadcastMasterNegotiateAnnounce();
						bragSent = true;
					}
				}
			}

			// If the timeout expires, become the master and broadcast that we're doing so
			if (state == MASTER_SELECTION && millis() > (pingStartedTime + MASTER_SEARCH_TIME)) {
#ifdef DEBUG
				Serial.println("Master search timed out - becoming master.");
#endif
				state = MASTER;
				broadcastClaimMaster();
			}
			break;
		case SLAVE:
			if (receiveData(&packet) > 0) {
				switch(packet.data[0]) {
				case HEARTBEAT:
					processHeartbeat(packet);
					slaveLastHeartbeat = millis();
					break;
				case PING:
					respondToPing(packet);
					break;
				case UNKNOWN:
				case PING_RESPONSE:
				case CLAIM_MASTER:
				default:
					break;
				}
			} else {
				if (millis() > slaveLastHeartbeat + SLAVE_TIMEOUT) {
#ifdef DEBUG
					Serial.println("No heartbeats - becoming master");
#endif
					state = MASTER;
					broadcastClaimMaster();
				}
			}
			break;
		default:
			break;
		}
#ifdef DEBUG
		if (state != prevState) {
			Serial.print("State: ");
			Serial.print(prevState);
			Serial.print(" -> ");
			Serial.println(state);
			prevState = state;
		}
#endif
	// Yield to the LED task
	vTaskDelay(1);
	} // must never exit
}

//The setup function is called once at startup of the sketch
void setup() {
	Serial.begin(57600);
	randomSeed(analogRead(ANALOG_RANDOM_PIN));
	cc.init();
	cc.setCarrierFreq(CFREQ_433);

	// Receive packets to all addresses
	cc.disableAddressCheck();

	// Setup some config values.
	// 10: clear channel "Unless currently receiving a packet"
	// 1111: default to RX mode after sending or receiving a packet
	//cc.writeReg(CC1101_MCSM1, 0b101111);
	cc.writeReg(CC1101_MCSM1, 0b100000);

	// Go for maximum range
	cc.setTxPowerAmp(PA_LongDistance);

	cc.setRxState();

	Serial.println("Starting search...");
	initStarted = millis();

	// TODO: remove this
	state = MASTER;

	BaseType_t xReturned = xTaskCreate(
	                    &mainLoop,       /* Function that implements the task. */
	                    "MAIN",          /* Text name for the task. */
	                    200,      /* Stack size in words, not bytes. */
	                    NULL,    /* Parameter passed into the task. */
	                    tskIDLE_PRIORITY + 2,/* Priority at which the task is created. */
	                    NULL );      /* Used to pass out the created task's handle. */
	if (!xReturned == pdPASS) {
		Serial.println("Couldn't create main!");
	}

	ledQueue = xQueueCreate(5, sizeof(LedMsg));
	Led* led = new Led(ledQueue);

	xReturned = xTaskCreate(
		                    &(Led::cast),       /* Function that implements the task. */
		                    "LED",          /* Text name for the task. */
		                    200,      /* Stack size in words, not bytes. */
		                    (void*) led,    /* Parameter passed into the task. */
		                    tskIDLE_PRIORITY + 1,/* Priority at which the task is created. */
		                    NULL );      /* Used to pass out the created task's handle. */
	if (!xReturned == pdPASS) {
		Serial.println("Couldn't create LED!");
	}

	Serial.println("Starting scheduler");
	vTaskStartScheduler();
}

void loop() {
	// Unused because of FreeRTOS
}
