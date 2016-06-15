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

#include "Arduino_FreeRTOS.h"
#include "cc1101/cc1101.h"
#include "queue.h"

#include "BeatDetector.h"
#include "Led.h"
#include "PatternController.h"

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//#define DEBUG_STATE

#define DEBUG

// Whether to check the version of other nodes on power-on
#define DEBUG_VERSION

// Whether to perform a Power-On System Test
#define POST

// Use an unconnected analog pin to seed the random number generator
const byte ANALOG_RANDOM_PIN = 1;

// Nondecreasing number indicating the software version. Hopefully I'll remember to increment this
// every time I update things :)
const byte OUR_VERSION = 2;
const byte VERSION_BRIGHTNESS = 100;

const unsigned long INIT_SEARCH_TIME_MILLIS = 2000;
const unsigned long MASTER_HEARTBEAT_INTERVAL = 800;

// How long to wait for a beat before resuming steady beats
const unsigned long BEAT_HEARTBEAT_INTERVAL = 1500;
const unsigned long HEARTBEAT_DURATION = 100;
const unsigned long MASTER_SEARCH_TIME = 200;
const unsigned long PING_WAIT = 50;

// Convert the constants above to ticks
const unsigned long HEARTBEAT_DURATION_TICKS = HEARTBEAT_DURATION / portTICK_PERIOD_MS;

// How long in slave mode without any heartbeats before becoming a master
const unsigned long SLAVE_TIMEOUT = 2000;

// Min and max times between color changes
const unsigned long COLOR_CHANGE_MIN = 4000;
const unsigned long COLOR_CHANGE_MAX = 10000;

// When generating a random color, the minimum (sum) brightness of the colors allowed.
const unsigned int MIN_SUM_BRIGHTNESS = 90;

// When generating a random color, at least one color must be at least this bright.
const unsigned int MIN_IND_BRIGHTNESS = 60;

// When generating a random color, the individual max brightness allowed (fed to random()).
const unsigned int MAX_BRIGHTNESS = 80;

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
	CHECK_VERSION,
};

CC1101 cc;
Led* led;
QueueHandle_t ledQueue;
PatternController* ctrl;
TaskHandle_t beatTask;

// Tells us that a beat was detected and we should broadcast a heartbeat
boolean beatDetected = false;

// Keeps state of whether we were detected beats or not. This way, when we transition from hearing
// beats to not (or vice versa), we change color.
boolean onBeat = false;

State state = INIT;

// Amount of time we've been in the init state. When we start, listen for a few
// seconds to see if someone else is broadcasting. If not, start broadcasting
// (i.e. become a master). Once we're a master, every so often check to see if
// someone else is a master.
unsigned long initStarted = 0;

// In the master state, millis since we last broadcast a heartbeat
unsigned long masterNextHeartbeatTime = 0;

// When we're a master, the time when we'll next change the heartbeat color.
unsigned long nextColorChange = 0;

// When we're a slave, the last time we received a heartbeat. If we don't receive one for long
// enough, we'll become master.
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
			return 0;
		}
		byte origLength = packet->length - 2;
		if (packet->data[origLength] != lastIdLower
				&& packet->data[origLength+1] != lastIdUpper) {
			packet ->length -= 2;
			lastIdLower = packet->data[origLength];
			lastIdUpper = packet->data[origLength+1];

			return packet->length;
		}
	}
	return 0;
}

// Blinks a heartbeat
void processHeartbeat(CCPACKET packet) {
	ctrl->setPattern((Pattern)packet.data[1], packet.data[2], packet.data[3], packet.data[4]);
}

void processOwnHeartbeat(CCPACKET packet) {
	ctrl->setPattern(BLINK_ONCE, packet.data[2], packet.data[3], packet.data[4]);
}

CCPACKET toSend;

byte heartbeatR = 63;
byte heartbeatG = 63;
byte heartbeatB = 63;

void setHeartbeatColor() {
	do {
		heartbeatR = random(MAX_BRIGHTNESS);
		heartbeatG = random(MAX_BRIGHTNESS);
		heartbeatB = random(MAX_BRIGHTNESS);
	} while ((heartbeatR < MIN_IND_BRIGHTNESS || heartbeatG < MIN_IND_BRIGHTNESS || heartbeatB < MIN_IND_BRIGHTNESS)
			&& (heartbeatR + heartbeatG + heartbeatB < MIN_SUM_BRIGHTNESS));
	nextColorChange = millis() + random(COLOR_CHANGE_MIN, COLOR_CHANGE_MAX);
}

// Heartbeat packet structure:
// 0: type (HEARTBEAT)
// 1: Pattern
// 2: Red intensity
// 3: Green intensity
// 4: Blue intensity
void broadcastHeartbeat() {
	if (beatDetected) {
		masterNextHeartbeatTime = millis() + BEAT_HEARTBEAT_INTERVAL;
		if (!onBeat || millis() > nextColorChange) {
			setHeartbeatColor();
			onBeat = true;
		}
	} else {
		masterNextHeartbeatTime = millis() + MASTER_HEARTBEAT_INTERVAL;
		if (onBeat || millis() > nextColorChange) {
			setHeartbeatColor();
			onBeat = false;
		}
	}
	beatDetected = false;
	toSend.length = 5;
	toSend.data[0] = HEARTBEAT;

	toSend.data[1] = 1;
	// Randomly choose a single or continuous blink. Upper bound is exclusive.
//	toSend.data[1] = random(1, 3);

	// White
	toSend.data[2] = heartbeatR;
	toSend.data[3] = heartbeatG;
	toSend.data[4] = heartbeatB;

	sendData(toSend);

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

// Displays GREEN if we're at or above the specified software version, RED otherwise.
// 0: type (CHECK_VERSION)
// 1: version
void respondToVersion(CCPACKET versionPacket) {
	if (versionPacket.data[1] > OUR_VERSION) {
		// BAD - we're not at the specified version
		ctrl->setPattern(LONG_BLINK, VERSION_BRIGHTNESS, 0, 0);
	} else {
		// GOOD - we're at least at the specified version
		ctrl->setPattern(LONG_BLINK, 0, VERSION_BRIGHTNESS, 0);
	}

	// POTENTIALLY DANGEROUS: this will be in production code and may cause problems. However,
	// well-formed clients will not send the check version command during normal use.
	vTaskDelay(1000);
}

#ifdef DEBUG_VERSION
void broadcastCheckVersion() {
	toSend.length = 2;
	toSend.data[0] = CHECK_VERSION;
	toSend.data[1] = OUR_VERSION;
	sendData(toSend);
}
#endif

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

/**
 * Become master. Sets the state, broadcasts a claim master, and turns on beat detection.
 */
void becomeMaster() {
	state = MASTER;
	broadcastClaimMaster();
	vTaskResume(beatTask);
	nextColorChange = millis() + random(COLOR_CHANGE_MIN, COLOR_CHANGE_MAX);
}

/**
 * Become slave.
 */
void becomeSlave() {
	state = SLAVE;
	slaveLastHeartbeat = millis();
	vTaskSuspend(beatTask);
}

byte getNegotiateAnnounceCount(CCPACKET brag) {
	return brag.data[1];
}

CCPACKET packet;
State prevState = INIT;

// The loop function is called in an endless loop
void mainLoop(void* params) {
	Serial.println("done.");

#ifdef DEBUG_VERSION
	broadcastCheckVersion();
	// Also blink the "GOOD" pattern ourselves
	ctrl->setPattern(LONG_BLINK, 0, VERSION_BRIGHTNESS, 0);
	vTaskDelay(1000);
#endif

	while (1) {
//		timing[timeIndex] = millis();
//		timeIndex++;
//		if (timeIndex > 19) {
//			timeIndex = 0;
//			Serial.print("Timing: ");
//			for (byte i=1; i<19; i++) {
//				Serial.print(timing[i] - timing[i-1], DEC);
//				Serial.print(", ");
//			}
//			Serial.println();
//		}

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
#ifdef DEBUG_STATE
				Serial.println("No packets received");
#endif
				becomeMaster();
				broadcastHeartbeat();
			}
			break;

		case MASTER:
			if (beatDetected || millis() > masterNextHeartbeatTime) {
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
				case CLAIM_MASTER:
					becomeSlave();
#ifdef DEBUG_STATE
					Serial.println("CLAIM_MASTER received");
#endif
					break;
				case CHECK_VERSION:
					respondToVersion(packet);
					break;
				case PING_RESPONSE:
					// If we're not doing master selection, we don't care about ping responses, so
					// ignore this.
				case UNKNOWN:
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
#ifdef DEBUG_STATE
					Serial.println("CLAIM_MASTER received, becoming slave");
#endif
					becomeSlave();
					break;
				case PING:
					respondToPing(packet);
					break;
				case PING_RESPONSE:
					// Note: we don't check whether the PING_WAIT time has expired here - we assume that
					// this loop is fast and it will get picked up below
					if (isMyPing(packet)) {
						lastPingCount++;
#ifdef DEBUG_STATE
						Serial.print("Received ping: ");
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
			if (beatDetected || millis() > masterNextHeartbeatTime) {
				broadcastHeartbeat();
			}

			if (millis() > (pingStartedTime + PING_WAIT)) {
				// If we've already received another brag, check it
				if (bragReceived) {
					if (lastReceivedBrag > lastPingCount) {
						// Become the slave
#ifdef DEBUG_STATE
						Serial.println("Received brag - becoming slave");
#endif
						becomeSlave();
					} else {
						// Become the master
						// Note: if the node counts are equal, the node to finish last (i.e. us, here)
						// becomes the master. This makes things simpler.
#ifdef DEBUG_STATE
						Serial.print("Received brag - becoming master");
#endif
						becomeMaster();
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
			if (state == MASTER_SELECTION && (millis() > (pingStartedTime + MASTER_SEARCH_TIME))) {
#ifdef DEBUG_STATE
				Serial.println("Master search timed out");
#endif
				becomeMaster();
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
				case CHECK_VERSION:
					respondToVersion(packet);
					break;
				case UNKNOWN:
				case PING_RESPONSE:
				case CLAIM_MASTER:
				default:
					break;
				}
			} else {
				if (millis() > slaveLastHeartbeat + SLAVE_TIMEOUT) {
#ifdef DEBUG_STATE
					Serial.println("No heartbeats - becoming master");
#endif
					becomeMaster();
				}
			}
			break;
		default:
			break;
		}
#ifdef DEBUG_STATE
		if (state != prevState) {
			Serial.print("State: ");
			Serial.print(prevState);
			Serial.print(" -> ");
			Serial.println(state);
			prevState = state;
		}
#endif
	} // must never exit
}

//The setup function is called once at startup of the sketch
void setup() {
	Serial.begin(57600);
	randomSeed(analogRead(ANALOG_RANDOM_PIN));

	// Set ADC to 77khz, max for 10bit
	sbi(ADCSRA,ADPS2);
	cbi(ADCSRA,ADPS1);
	cbi(ADCSRA,ADPS0);


	Serial.println("Starting... ");
	initStarted = millis();

	BaseType_t xReturned = xTaskCreate(
	                    &mainLoop,       /* Function that implements the task. */
	                    "MAIN",          /* Text name for the task. */
	                    200,      /* Stack size in words, not bytes. */
	                    NULL,    /* Parameter passed into the task. */
	                    1,/* Priority at which the task is created. */
	                    NULL );      /* Used to pass out the created task's handle. */
#ifdef DEBUG
	if (!xReturned == pdPASS) {
		Serial.println("main");
	}
#endif

	// MYSTERIES OF THE UNIVERSE: these allocations must happen after the main xTaskCreate above,
	// or everything breaks.
	ledQueue = xQueueCreate(5, sizeof(LedMsg));
	led = new Led(ledQueue);

#ifdef POST
	led->writeColor(127, 0, 0);
	delay(200);
	led->writeColor(0, 127, 0);
	delay(200);
	led->writeColor(0, 0, 127);
	delay(200);
	led->writeColor(0, 0, 0);
#endif

	xReturned = xTaskCreate(
			&(Led::cast),       /* Function that implements the task. */
			"LED",          /* Text name for the task. */
			85,      /* Stack size in words, not bytes. */
			(void*) led,    /* Parameter passed into the task. */
			2,/* Priority at which the task is created. */
			NULL );      /* Used to pass out the created task's handle. */
#ifdef DEBUG
	if (!xReturned == pdPASS) {
		Serial.println("LED");
	}
#endif

	ctrl = new PatternController(led, ledQueue);
	xReturned = xTaskCreate(
			&(PatternController::cast),       /* Function that implements the task. */
			"CTRL",          /* Text name for the task. */
			150,      /* Stack size in words, not bytes. */
			(void*) ctrl,    /* Parameter passed into the task. */
			2,/* Priority at which the task is created. */
			NULL );      /* Used to pass out the created task's handle. */
#ifdef DEBUG
	if (!xReturned == pdPASS) {
		Serial.println("ctrl");
	}
#endif

	BeatDetector* beat = new BeatDetector();
	xReturned = xTaskCreate(
			&(BeatDetector::cast),       /* Function that implements the task. */
			"BEAT",          /* Text name for the task. */
			150,      /* Stack size in words, not bytes. */
			(void*) beat,    /* Parameter passed into the task. */
			3,/* Priority at which the task is created. */
			&beatTask );      /* Used to pass out the created task's handle. */

	// beatTask gets resumed when we become master
	vTaskSuspend(beatTask);

	// Init the radio
	cc.init();
	cc.setCarrierFreq(CFREQ_433);

	// Receive packets to all addresses
	cc.disableAddressCheck();

	// Setup some config values.
	// 10: clear channel "Unless currently receiving a packet"
	// 1111: default to RX mode after sending or receiving a packet
	//cc.writeReg(CC1101_MCSM1, 0b101111);
	cc.writeReg(CC1101_MCSM1, 0b100000);

#ifdef POST
	if (cc.readConfigReg(CC1101_MCSM1) == 0b100000) {
		led->writeColor(0, 127, 0);
		delay(200);
	} else {
		led->writeColor(127, 0, 0);
		delay(500);
	}
#endif

	// Go for maximum range
	cc.setTxPowerAmp(PA_LongDistance);

	cc.setRxState();

	// Used to calibrate the DC offset of the beat detection circuitry.
//	for(int i=0; i<20; i++) {
//		Serial.println(analogRead(INPUT_PIN));
//		delay(1);
//	}

	vTaskStartScheduler();
}

// Called if a stack overflow is detected
#ifdef DEBUG
void vApplicationStackOverflowHook( TaskHandle_t xTask, portCHAR *pcTaskName ) {
	Serial.println("overflow");
}
#endif // DEBUG

void loop() {
	// Unused because of FreeRTOS
}
