#include "Arduino.h"

#include "cc1101/cc1101.h"

// Use an unconnected analog pin to seed the random number generator
#define ANALOG_RANDOM_PIN 1
#define LED_R 3 // ATMega328P pin 5
#define LED_G 5 // ATMega328P pin 11
#define LED_B 6 // ATMega328P pin 12


const unsigned long INIT_SEARCH_TIME_MILLIS = 5000;
const unsigned long MASTER_SEARCH_TIME = 5000;
const unsigned long MASTER_HEARTBEAT_INTERVAL = 1000;
const unsigned long HEARTBEAT_DURATION = 100;
const unsigned long PING_WAIT = 1000;

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
	MASTER_NEGIOTATE_ANNOUNCE,	// Master brags about how many visible nodes
};

CC1101 cc;

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

//The setup function is called once at startup of the sketch
void setup() {
	Serial.begin(9600);
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
	//cc.setTxPowerAmp(PA_LongDistance);

	cc.setRxState();

	Serial.println("Starting search...");
	initStarted = millis();
}

void writeColor(byte r, byte g, byte b) {
	analogWrite(LED_R, r);
	analogWrite(LED_G, g);
	analogWrite(LED_B, b);
}


// Blinks a heartbeat for a preset amount of time. Blocks for HEARTBEAT_DURATION or so.
void processHeartbeat(CCPACKET packet) {
	writeColor(packet.data[1], packet.data[2], packet.data[3]);
	delay(HEARTBEAT_DURATION);
	writeColor(0, 0, 0);
}

// Heartbeat packet structure:
// 0: type (HEARTBEAT)
// 1: Red intensity
// 2: Green intensity
// 3: Blue intensity
void broadcastHeartbeat() {
	CCPACKET heartbeat;
	heartbeat.length = 4;
	heartbeat.data[0] = HEARTBEAT;

	// White
	heartbeat.data[1] = 0xFF;
	heartbeat.data[2] = 0xFF;
	heartbeat.data[3] = 0xFF;

	cc.sendData(heartbeat);
	masterLastHeartbeatTime = millis();

	processHeartbeat(heartbeat);
}

// Causes all other masters in range to become slaves. Structure:
// 0: type (CLAIM_MASTER)
void broadcastClaimMaster() {
	CCPACKET claimMaster;
	claimMaster.length = 1;
	claimMaster.data[0] = CLAIM_MASTER;

	cc.sendData(claimMaster);
}

// Causes all reachable nodes to respond. Used to identify how many other nodes
// are reachable from this node (for master election). Uses a nonce to uniquely
// identify responses to this specific ping request. This kicks off the master
// selection process.
// 0: type (PING)
// 1: nonce 0
// 2: nonce 1
void broadcastPing() {
	CCPACKET ping;
	ping.length = 3;

	ping.data[0] = PING;
	lastNonceLower = ping.data[1] = random(0x100);
	lastNonceUpper = ping.data[2] = random(0x100);

	cc.sendData(ping);

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
	CCPACKET pingResponse;
	pingResponse.length = 3;
	pingResponse.data[0] = PING_RESPONSE;
	pingResponse.data[1] = originalPing.data[1];
	pingResponse.data[2] = originalPing.data[2];

	cc.sendData(pingResponse);
}

bool isMyPing(CCPACKET pingResponse) {
	Serial.print("isMyPing: (");
	Serial.print(pingResponse.data[1], DEC);
	Serial.print(",");
	Serial.print(lastNonceLower, DEC);
	Serial.print(") (");

	Serial.print(pingResponse.data[2], DEC);
	Serial.print(",");
	Serial.print(lastNonceUpper, DEC);
	Serial.println(")");
	return pingResponse.data[1] == lastNonceLower
			&& pingResponse.data[2] == lastNonceUpper;
}

// Probably ends master selection. Announces how many nodes we can see. Whoever has the higher
// number becomes master. If the numbers are equal, we flip a coin until someone wins.
// 0: type (MASTER_NEGIOTATE_ANNOUNCE)
// 1: node count
void broadcastMasterNegotiateAnnounce() {
	CCPACKET brag;
	brag.length = 2;
	brag.data[0] = MASTER_NEGIOTATE_ANNOUNCE;
	brag.data[1] = lastPingCount;

	cc.sendData(brag);
}

byte getNegotiateAnnounceCount(CCPACKET brag) {
	return brag.data[1];
}

CCPACKET packet;
State prevState = INIT;

// The loop function is called in an endless loop
void loop() {
	switch (state) {
	case INIT:
		if (cc.receiveData(&packet) > 0) {
			// Process all available packets
			do {
				switch(packet.data[0]) {
				case HEARTBEAT:
					state = SLAVE;
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
				delay(1);
			} while (cc.receiveData(&packet) > 0);
		}
		if (millis() > (initStarted + INIT_SEARCH_TIME_MILLIS)) {
			Serial.println("No packets received - becoming master.");
			state = MASTER;
			broadcastClaimMaster();
			broadcastHeartbeat();
		}
		break;
	case MASTER:
		if (millis() > (masterLastHeartbeatTime + MASTER_HEARTBEAT_INTERVAL)) {
			broadcastHeartbeat();
		}
		if (cc.receiveData(&packet) > 0) {
			switch(packet.data[0]) {
			case HEARTBEAT:
			case MASTER_NEGIOTATE_ANNOUNCE:
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
			default:
				break;
			}
		}
		break;
	// TODO: this will probably break horribly if there are more than 2 nodes doing this at once
	case MASTER_SELECTION:
		if (cc.receiveData(&packet) > 0) {
			switch(packet.data[0]) {
			case CLAIM_MASTER:
				// If someone else thinks they see more nodes than us, trust them and become a slave.
				state = SLAVE;
				break;
			case PING:
				respondToPing(packet);
				break;
			case PING_RESPONSE:
				// Note: we don't check whether the PING_WAIT time has expired here - we assume that
				// this loop is fast and it will get picked up below
				if (isMyPing(packet)) {
					lastPingCount++;
				}
				break;
			case MASTER_NEGIOTATE_ANNOUNCE:
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

		// Still do the heartbeat when negotiating for a single master
		if (millis() > (masterLastHeartbeatTime + MASTER_HEARTBEAT_INTERVAL)) {
			broadcastHeartbeat();
		}

		if (millis() > pingStartedTime + PING_WAIT) {
			// If we've already received another brag, check it
			if (bragReceived) {
				if (lastReceivedBrag > lastPingCount) {
					// Become the slave
					state = SLAVE;
				} else if (lastReceivedBrag <= lastPingCount) {
					// Become the master
					// Note: if the node counts are equal, the node to finish last (i.e. us, here)
					// becomes the master. This makes things simpler.
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
		if (millis() > pingStartedTime + MASTER_SEARCH_TIME) {
			Serial.println("Master search timed out - becoming master.");
			state = MASTER;
			broadcastClaimMaster();
		}
		break;
	case SLAVE:
		if (cc.receiveData(&packet) > 0) {
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
				Serial.println("No heartbeats - becoming master");
				state = MASTER;
			}
		}
		break;
	default:
		break;
	}
	delay(5);
	if (state != prevState) {
		Serial.print("State: ");
		Serial.print(prevState);
		Serial.print(" -> ");
		Serial.println(state);
		prevState = state;
	}
}
