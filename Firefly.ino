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
#include <avr/wdt.h>

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
//#define DEBUG_VERSION

// Whether to perform a Power-On System Test
#define POST

// Use an unconnected analog pin to seed the random number generator
const byte ANALOG_RANDOM_PIN = 1;

// The digital input pin for the beacon switch
const byte BEACON_PIN = 7;

// Built-in LED
const byte STATUS_LED = 8;

// Nondecreasing number indicating the software version. Hopefully I'll remember to increment this
// every time I update things :)
const byte OUR_VERSION = 11;
const byte VERSION_BRIGHTNESS = 100;

const unsigned long INIT_SEARCH_TIME_MILLIS = 2000;
const unsigned long MASTER_HEARTBEAT_INTERVAL = 600;

// How long between heartbeats in beacon mode
const unsigned long BEACON_HEARTBEAT_INTERVAL = 600;

// How frequently to retransmit the beacon mode packet
const unsigned long BEACON_MODE_RETRANSMIT = 1000;

// How long after our last beacon mode packet before beacon mode times out
const unsigned long BEACON_MODE_TIMEOUT = 5000;

// If someone else sends a beacon mode packet, how long to wait after BEACON_MODE_RETRANSMIT
// for them to send it before sending it ourself, so that we don't both send it at the same time.
const unsigned long BEACON_MODE_BACKOFF = 100;

// During beat detection, the longest time between heartbeats. If we haven't detected a beat and
// this runs out, we'll send a keepalive (heartbeat with the OFF pattern).
const unsigned long BEAT_HEARTBEAT_INTERVAL = 1000;

// While doing beat detection, if we don't see a beat for this long, go back to normal mode.
const unsigned long BEAT_DETECTION_TIMEOUT = 5000;

const unsigned long HEARTBEAT_DURATION = 100;
const unsigned long MASTER_SEARCH_TIME_MIN = 100;	 // must be greater than PING_WAIT
const unsigned long MASTER_SEARCH_TIME_MAX = 300;
const unsigned long PING_WAIT = 50;

// Convert the constants above to ticks
const unsigned long HEARTBEAT_DURATION_TICKS = HEARTBEAT_DURATION / portTICK_PERIOD_MS;

// When we receive become a slave, we'll stop trying to be master for a random amount of time
// between these two values.
const unsigned long SLAVE_TIMEOUT_BACKOFF_MIN = 2000;
const unsigned long SLAVE_TIMEOUT_BACKOFF_MAX = 5000;

const unsigned long CLAIM_MASTER_TIMEOUT_MIN = 10000;
const unsigned long CLAIM_MASTER_TIMEOUT_MAX = 20000;

// Min and max times between color changes
const unsigned long COLOR_CHANGE_MIN = 4000;
const unsigned long COLOR_CHANGE_MAX = 10000;

// How frequently to calibrate the radio (by issuing FSTXON)
const unsigned long CALIBRATE_EVERY = 30000;

// When generating a random color, the minimum (sum) brightness of the colors allowed.
const unsigned int MIN_SUM_BRIGHTNESS = 90;

// When generating a random color, at least one color must be at least this bright.
const unsigned int MIN_IND_BRIGHTNESS = 60;

// When generating a random color, the individual max brightness allowed (fed to random()).
const unsigned int MAX_BRIGHTNESS = 100;

// In the heartbeat packet, the most significant bit is the TTL bit. When set, it indicates that
// the receiver should rebroadcast the packet.
const byte TTL_MASK = 0x80;
const byte PATTERN_MASK = 0x7F;

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
	BEACON_MODE,
};

BeatDetector* beat;
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

boolean beaconMode = false;

// When to send the next beacon mode packet
unsigned long nextBeaconModeSend = 0;

// When the current beacon mode will timeout
unsigned long beaconModeTimeout = 0;

State state = INIT;

// Amount of time we've been in the init state. When we start, listen for a few
// seconds to see if someone else is broadcasting. If not, start broadcasting
// (i.e. become a master). Once we're a master, every so often check to see if
// someone else is a master.
unsigned long initStarted = 0;

// In the master state, millis since we last broadcast a heartbeat
unsigned long masterNextHeartbeatTime = 0;

// How long to wait for beats before switching back to non-beat-detection mode.
unsigned long beatDetectionTimeout = 0;

// When we're a master, the time when we'll next change the heartbeat color.
unsigned long nextColorChange = 0;

// When we're a slave, the last time we received a heartbeat. If we don't receive one for long
// enough, we'll become master.
unsigned long slaveNoHeartbeatTimeout = 0;

// When we receive a CLAIM_MASTER, don't try to be master for a while. This is the timer for
// tracking that - when millis() is greater than this, we're clear to try to be master again.
unsigned long claimMasterTimeout = 0;

// During master election, when waiting for ping responses times out.
unsigned long pingTimeout = 0;

// During master election, when our search times out and we become the de facto master.
unsigned long masterSearchTimeout = 0;

// Debug: print the status of the radio every so often
#ifdef DEBUG_STATE
unsigned long printStatusAt = 0;
#endif

unsigned long calibrateAt = 0;

// Outgoing-ping-related variables
byte lastNonceLower = 0;
byte lastNonceUpper = 0;
byte masterElectionRandomNumber = 0;
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

// Rebroadcasts a packet with the same ids that we received it with. Assumes that the ids are already
// set.
void sendDataWithPreviousId(CCPACKET packet) {
	lastIdLower = packet.data[packet.length];
	lastIdUpper = packet.data[packet.length+1];
	packet.length += 2;

	// Since we're rebroadcasting, only send once
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
			packet->length -= 2;
			lastIdLower = packet->data[origLength];
			lastIdUpper = packet->data[origLength+1];

			return packet->length;
		}
	}
	return 0;
}

// Blinks a heartbeat
void processHeartbeat(CCPACKET packet) {
	// Mask out the TTL bit
	Pattern pattern = (Pattern) (packet.data[1] & PATTERN_MASK);

	if (beaconMode && digitalRead(BEACON_PIN)) {
		ctrl->setPattern(pattern, 80, 80, 0);
	} else {
		ctrl->setPattern(pattern, packet.data[2], packet.data[3], packet.data[4]);
	}

	// If the TTL bit is set, clear it (!) and retransmit the packet
	if (packet.data[1] & TTL_MASK) {
		packet.data[1] &= PATTERN_MASK;
		sendDataWithPreviousId(packet);
	}
}

void processOwnHeartbeat(CCPACKET packet) {
	Pattern pattern = (Pattern) (packet.data[1] & PATTERN_MASK);
	if (beaconMode && digitalRead(BEACON_PIN)) {
		ctrl->setPattern(pattern, 80, 80, 0);
	} else {
		ctrl->setPattern(pattern, packet.data[2], packet.data[3], packet.data[4]);
	}
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
			&& ((unsigned int)heartbeatR + heartbeatG + heartbeatB < MIN_SUM_BRIGHTNESS));
	nextColorChange = millis() + random(COLOR_CHANGE_MIN, COLOR_CHANGE_MAX);
}

// Heartbeat packet structure:
// 0: type (HEARTBEAT)
// 2: Pattern
// 3: Red intensity
// 4: Green intensity
// 5: Blue intensity
void broadcastHeartbeat() {
	if (beaconMode) {
		masterNextHeartbeatTime = millis() + BEACON_HEARTBEAT_INTERVAL;
		if (millis() > nextColorChange) {
			setHeartbeatColor();
		}
	} else if (beatDetected) {
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

	if (beaconMode) {
		//toSend.data[1] = BLINK_TWICE | TTL_MASK;
		toSend.data[1] = BLINK_TWICE;
	} else {
		//toSend.data[1] = BLINK_ONCE | TTL_MASK;
		toSend.data[1] = BLINK_ONCE;
	}
	toSend.data[2] = heartbeatR;
	toSend.data[3] = heartbeatG;
	toSend.data[4] = heartbeatB;

	sendData(toSend);
	processOwnHeartbeat(toSend);
}

// Heartbeat packet structure:
// 0: type (HEARTBEAT)
// 2: Pattern
// 3: Red intensity
// 4: Green intensity
// 5: Blue intensity
void broadcastKeepalive() {
	masterNextHeartbeatTime = millis() + MASTER_HEARTBEAT_INTERVAL;

	toSend.length = 5;
	toSend.data[0] = HEARTBEAT;
	toSend.data[1] = OFF;
	sendData(toSend);
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

	pingTimeout = millis() + PING_WAIT;
	masterSearchTimeout = millis() + random(MASTER_SEARCH_TIME_MIN, MASTER_SEARCH_TIME_MAX);
	masterElectionRandomNumber = 0;
	lastReceivedBrag = 0;
	bragReceived = false;
	bragSent = false;
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

// Broadcasts that the network is in beacon mode. Beacon mode disables beat detection.
// 0: type (BEACON_MODE)
// 1: TTL (0 or 1)
void broadcastBeaconMode() {
	toSend.length = 2;
	toSend.data[0] = BEACON_MODE;
	toSend.data[1] = 1;
	sendData(toSend);
}

void receiveBeaconMode(CCPACKET packet) {
	beaconMode = true;
	// If we receive a beacon mode packet, don't send one for a little while
	nextBeaconModeSend = millis() + BEACON_MODE_RETRANSMIT + BEACON_MODE_BACKOFF;
	beaconModeTimeout = millis() + BEACON_MODE_TIMEOUT;
	// If TTL is 1, rebroadcast. Otherwise, do nothing.
	if (packet.data[1]) {
		packet.data[1] = 0;
		sendDataWithPreviousId(packet);
	}
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
	toSend.data[1] = masterElectionRandomNumber = random(0, 255);

	sendData(toSend);
}

/**
 * Become master. Sets the state, broadcasts a claim master, and turns on beat detection.
 */
void becomeMaster() {
	state = MASTER;
	broadcastClaimMaster();
	beat->resetLastWakeTime();
	vTaskResume(beatTask);
	nextColorChange = millis() + random(COLOR_CHANGE_MIN, COLOR_CHANGE_MAX);
	digitalWrite(STATUS_LED, HIGH);
	broadcastHeartbeat();
}

/**
 * Become slave.
 */
void becomeSlave() {
	state = SLAVE;
	setSlaveNoHeartbeatTimeout();
	vTaskSuspend(beatTask);
	digitalWrite(STATUS_LED, LOW);
	beatDetected = false;
}

void setSlaveNoHeartbeatTimeout() {
	slaveNoHeartbeatTimeout = millis() + random(SLAVE_TIMEOUT_BACKOFF_MIN, SLAVE_TIMEOUT_BACKOFF_MAX);
}

void setClaimMasterTimeout() {
	claimMasterTimeout = millis() + random(CLAIM_MASTER_TIMEOUT_MIN, CLAIM_MASTER_TIMEOUT_MAX);
}

byte getNegotiateAnnounceCount(CCPACKET brag) {
	return brag.data[1];
}

CCPACKET packet;
State prevState = INIT;

// The loop function is called in an endless loop
void mainLoop(void* params) {
	Serial.println("done.");

	wdt_enable(WDTO_2S);

#ifdef DEBUG_VERSION
	broadcastCheckVersion();
	// Also blink the "GOOD" pattern ourselves
	ctrl->setPattern(LONG_BLINK, 0, VERSION_BRIGHTNESS, 0);
	vTaskDelay(1000);
#endif

	while (1) {

		// Beacon mode: if the beacon mode switch is on, turn on beacon mode.
		if (digitalRead(BEACON_PIN)) {
			beaconMode = true;
			if (millis() > nextBeaconModeSend) {
				broadcastBeaconMode();
				nextBeaconModeSend = millis() + BEACON_MODE_RETRANSMIT;
			}
		} else {
			if (millis() > beaconModeTimeout) {
				beaconMode = false;
			}
		}

		switch (state) {
		case INIT:
			if (receiveData(&packet) > 0) {
				switch(packet.data[0]) {
				case HEARTBEAT:
					state = SLAVE;
					setSlaveNoHeartbeatTimeout();
					processHeartbeat(packet);
					break;
				case CLAIM_MASTER:
					state = SLAVE;
					setClaimMasterTimeout();
					break;
				case PING:
					break;
				case PING_RESPONSE:
				case BEACON_MODE:
				case UNKNOWN:
				default:
					break;
				}
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
			if (beaconMode) {
				// ignore beat detection
				if (millis() > masterNextHeartbeatTime) {
					broadcastHeartbeat();
				}
			} else if (beatDetected) {
				broadcastHeartbeat();
				beatDetectionTimeout = millis() + BEAT_DETECTION_TIMEOUT;
			} else if (millis() > masterNextHeartbeatTime) {
				if (millis() > beatDetectionTimeout) {
					broadcastHeartbeat();
				} else {
					broadcastKeepalive();
				}
			}
			if (receiveData(&packet) > 0) {
				switch(packet.data[0]) {
				case HEARTBEAT:
					// Two masters - time to do master selection!
					state = MASTER_SELECTION;
#ifdef DEBUG_STATE
					Serial.println("HEARTBEAT received - master selection");
#endif
					//broadcastPing();
					break;
				case PING:
					break;
				case CLAIM_MASTER:
					becomeSlave();
					setClaimMasterTimeout();
#ifdef DEBUG_STATE
					Serial.println("CLAIM_MASTER received");
#endif
					break;
				case CHECK_VERSION:
					respondToVersion(packet);
					break;
				case BEACON_MODE:
					receiveBeaconMode(packet);
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
					setClaimMasterTimeout();
					break;
				case PING:
					break;
				case PING_RESPONSE:
					break;
				case MASTER_NEGOTIATE_ANNOUNCE:
					bragReceived = true;
					lastReceivedBrag = getNegotiateAnnounceCount(packet);
					if (lastReceivedBrag > masterElectionRandomNumber) {
						state = SLAVE;
					}
					break;
				case HEARTBEAT:
				case BEACON_MODE:
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

			// If the timeout expires, become the master and broadcast that we're doing so
			if (state == MASTER_SELECTION && (millis() > masterSearchTimeout)) {
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
					setSlaveNoHeartbeatTimeout();
					break;
				case PING:
					break;
				case CHECK_VERSION:
					respondToVersion(packet);
					break;
				case BEACON_MODE:
					receiveBeaconMode(packet);
					break;
				case CLAIM_MASTER:
					setSlaveNoHeartbeatTimeout();
					setClaimMasterTimeout();
					break;
				case UNKNOWN:
				case PING_RESPONSE:
				default:
					break;
				}
			} else {
				if (millis() > slaveNoHeartbeatTimeout && millis() > claimMasterTimeout) {
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

	if (millis() > calibrateAt) {
		calibrateAt = millis() + CALIBRATE_EVERY;
		cc.cmdStrobe(CC1101_SFSTXON);
		vTaskDelay(1);
		cc.cmdStrobe(CC1101_SRX);
	}
#ifdef DEBUG_STATE
		if (millis() > printStatusAt) {
			printStatusAt = millis() + 5000;
			Serial.println(cc.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER));
			//cc.cmdStrobe(CC1101_SCAL);
		}
#endif

		wdt_reset();
	} // must never exit
}

//The setup function is called once at startup of the sketch
void setup() {
	wdt_disable();
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
	                    250,      /* Stack size in words, not bytes. */
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
			120,      /* Stack size in words, not bytes. */
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
			200,      /* Stack size in words, not bytes. */
			(void*) ctrl,    /* Parameter passed into the task. */
			2,/* Priority at which the task is created. */
			NULL );      /* Used to pass out the created task's handle. */
#ifdef DEBUG
	if (!xReturned == pdPASS) {
		Serial.println("ctrl");
	}
#endif

	beat = new BeatDetector();
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
	//Serial.println("overflow");
	analogWrite(3, 0);
	analogWrite(5, 0);
	analogWrite(6, 0);

	switch(pcTaskName[0]) {
	case 'M': // MAIN
		analogWrite(3, 50);
		break;

	case 'L': // LED
		analogWrite(5, 50);
		break;

	case 'C': // CTRL
		analogWrite(6, 50);
		break;

	case 'B': // BEAT
		analogWrite(3, 50);
		analogWrite(6, 50);
		break;

	default:
		analogWrite(3, 50);
		analogWrite(5, 50);
		analogWrite(6, 50);
	}

	Serial.println(pcTaskName);
}
#endif // DEBUG

void loop() {
	// Unused because of FreeRTOS
}
