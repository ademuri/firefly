import argparse
from enum import IntEnum
import os
import random
import serial
import time
import unittest

class PacketType(IntEnum):
	UNKNOWN = 0
	HEARTBEAT = 1
	PING = 2
	PING_RESPONSE = 3
	CLAIM_MASTER = 4
	MASTER_NEGOTIATE_ANNOUNCE = 5
	

class Packet:
	""" Takes a raw list of bytes """
	def __init__(self, data=None):
		if data is not None:
			self.data = data
			self.type = PacketType(int(data[0]))
			self.payload = [int(x) for x in data[1:len(data)-2]]
			# length of the payload
			self.length = len(self.payload)
			self.id = [int(x) for x in data[self.length+1:]]
		
	def __str__(self):
		ret = ""
		ret += "Type: %s\n" % self.type.name
		#ret += "Length: %s\n" % self.length
		ret += "Payload: %s\n" % ",".join(str(x) for x in self.payload)
		#ret += "ID: %s\n" % ",".join(str(x) for x in self.id)
		return ret
	
	""" Whether the packet has the same ID as this one """
	def has_same_id(self, other):
		if other is None:
			return False
		if len(self.id) != 2 or len(other.id) != 2:
			return False
		return (self.id[0] == other.id[0]) and (self.id[1] == other.id[1])
		
	def generate(type, payload):
		packet = Packet()
		packet.type = type
		packet.payload = payload
		packet.length = len(payload) + 3
		packet.id = [random.randrange(0, 255), random.randrange(0, 255)]
		packet.data = [type.value] + payload + packet.id
		return packet
		
		
class Radio:
	def __init__(self, port):
		self.s = serial.Serial(port, 9600, timeout=0.1)
		self.last_packet = None
		self.packet_log = []
	
	def read_packet(self, timeout=1):
		""" Default (None) timeout actually uses a 1s timeout :/ """
		end_time = time.time() + timeout
		while time.time() < end_time:
			packet = self.read_packet_()
			if packet is not None:
				return packet
		
	def read_packet_(self):
		read_string = ""
		b = self.s.read(1)
			
		while not b.isspace():
			# if we got nothing back, then we hit the timeout
			if b is None or len(b) is 0:
				return None
			read_string += b.decode("ascii")
			b = self.s.read(1)
			
		# eat whitespace
		if read_string.isspace() or len(read_string) is 0:
			return self.read_packet()

		packet = Packet(read_string.split(","))
		
		# eat duplicate packets
		if packet.has_same_id(self.last_packet):
			return self.read_packet()
			
		self.last_packet = packet
		self.packet_log.append(packet)
		return packet
	
	""" Reads until the next 'interesting' packet, with the given (rough) timeout """
	def read_packet_swallow_heartbeat(self, timeout=None):
		end_time = time.time() + timeout
		while time.time() < end_time:
			packet = self.read_packet_()
			if (packet is not None) and (packet.type is not PacketType.HEARTBEAT):
				return packet
		return None
		
	def swallow_pending_packets(self):
		if self.s.in_waiting is 0:
			return
		read = self.s.read(self.s.in_waiting)
		if not str(read[len(read)-1]).isspace():
			# read until the next space character
			b = self.s.read(1).decode("ascii")
			while not (b.isspace() or b is None):
				b = self.s.read(1).decode("ascii")
		
		
	def send_packet(self, packet):
		#print("Sending: %s" % packet)
		send_string = ",".join(str(x) for x in packet.data) + "\n"
		self.s.write(send_string.encode("ascii"))
					

parser = argparse.ArgumentParser(description='Test Firefly functionality')
parser.add_argument('port')
port = parser.parse_args().port					
					
radio = Radio(port)		
#read = 0
# read 2 heartbeats
#while read < 2:
#	packet = radio.read_packet()
#	if packet is not None:
#		print(packet)
#		read += 1

class SmallTests(unittest.TestCase):
	def setUp(self):
		radio.swallow_pending_packets()
	
	def test_emits_heartbeat(self):
		packet = radio.read_packet(10)
		self.assertIsNotNone(packet)
		self.assertEqual(packet.type, PacketType.HEARTBEAT)
	
	def test_responds_to_ping(self):
		ping = Packet.generate(PacketType.PING, [12, 34])
		radio.send_packet(ping)
		
		received = radio.read_packet_swallow_heartbeat(10)
		self.assertIsNotNone(received)
		self.assertEqual(received.type, PacketType.PING_RESPONSE)
		
class MediumTests(unittest.TestCase):
	def setUp(self):
		# Let the node under test reset
		time.sleep(10)
		radio.swallow_pending_packets()
		
	def test_master_selection_on_heartbeat(self):
		heartbeat = Packet.generate(PacketType.HEARTBEAT, [255, 255, 255])
		radio.send_packet(heartbeat)
		
		received = radio.read_packet_swallow_heartbeat(10)
		self.assertIsNotNone(received)
		self.assertEqual(received.type, PacketType.PING)
		
		received = radio.read_packet_swallow_heartbeat(10)
		self.assertIsNotNone(received)
		self.assertEqual(received.type, PacketType.MASTER_NEGOTIATE_ANNOUNCE)
		self.assertEqual(received.payload, [0])
		
		received = radio.read_packet_swallow_heartbeat(10)
		self.assertIsNotNone(received)
		self.assertEqual(received.type, PacketType.CLAIM_MASTER)
		
	def test_counts_ping(self):
		heartbeat = Packet.generate(PacketType.HEARTBEAT, [255, 255, 255])
		radio.send_packet(heartbeat)
		
		ping = radio.read_packet_swallow_heartbeat(10)
		self.assertIsNotNone(ping)
		self.assertEqual(ping.type, PacketType.PING)
		
		ping_response = Packet.generate(PacketType.PING_RESPONSE, ping.payload)
		radio.send_packet(ping_response)
		
		received = radio.read_packet_swallow_heartbeat(10)
		self.assertIsNotNone(received)
		self.assertEqual(received.type, PacketType.MASTER_NEGOTIATE_ANNOUNCE)
		self.assertEqual(received.payload, [1])
		
	def test_counts_multiple_pings(self):
		heartbeat = Packet.generate(PacketType.HEARTBEAT, [255, 255, 255])
		radio.send_packet(heartbeat)
		
		ping = radio.read_packet_swallow_heartbeat(10)
		self.assertIsNotNone(ping)
		self.assertEqual(ping.type, PacketType.PING)
		
		ping_response = Packet.generate(PacketType.PING_RESPONSE, ping.payload)
		radio.send_packet(ping_response)
		ping_response = Packet.generate(PacketType.PING_RESPONSE, ping.payload)
		radio.send_packet(ping_response)
		
		received = radio.read_packet_swallow_heartbeat(10)
		self.assertIsNotNone(received)
		self.assertEqual(received.type, PacketType.MASTER_NEGOTIATE_ANNOUNCE)
		self.assertEqual(received.payload, [2])
		
		received = radio.read_packet(10)
		self.assertIsNotNone(received)
		self.assertEqual(received.type, PacketType.HEARTBEAT)
		
	def test_becomes_slave(self):
		heartbeat = Packet.generate(PacketType.HEARTBEAT, [255, 255, 255])
		radio.send_packet(heartbeat)
		
		ping = radio.read_packet_swallow_heartbeat(10)
		self.assertIsNotNone(ping)
		self.assertEqual(ping.type, PacketType.PING)
		
		ping_response = Packet.generate(PacketType.PING_RESPONSE, ping.payload)
		radio.send_packet(ping_response)
		
		packet = Packet.generate(PacketType.MASTER_NEGOTIATE_ANNOUNCE, [3])
		radio.send_packet(packet)
		
		packet = Packet.generate(PacketType.CLAIM_MASTER, [])
		radio.send_packet(packet)
		
		packet = Packet.generate(PacketType.HEARTBEAT, [255, 255, 255])
		radio.send_packet(packet)
		radio.swallow_pending_packets()
		
		end_time = time.time() + 10
		while time.time() < end_time:
			packet = Packet.generate(PacketType.HEARTBEAT, [255, 255, 255])
			radio.send_packet(packet)
			received = radio.read_packet(2)
			self.assertIsNone(received)
			time.sleep(0.2)
			
		# if we don't heartbeat, it should become master again
		time.sleep(5)
		received = radio.read_packet_swallow_heartbeat(5)
		self.assertIsNotNone(received)
		self.assertEqual(received.type, PacketType.CLAIM_MASTER)
	
print("Running small tests...")
small_suite = unittest.TestLoader().loadTestsFromTestCase(SmallTests)
unittest.TextTestRunner(verbosity=2).run(small_suite)

print("\n\nRunning medium tests...")
medium_suite = unittest.TestLoader().loadTestsFromTestCase(MediumTests)
unittest.TextTestRunner(verbosity=2).run(medium_suite)

#packet = Packet.generate(PacketType.HEARTBEAT, [255, 255, 255])
#radio.send_packet(packet)
#start_time = time.time()


#time.sleep(.1)
#print("Time: %d" % (time.time() - start_time))
#packet = Packet.generate(PacketType.MASTER_NEGIOTATE_ANNOUNCE, [3])
#radio.send_packet(packet)

#time.sleep(.1)
#print("Time: %d" % (time.time() - start_time))
#packet = Packet.generate(PacketType.CLAIM_MASTER, [])
#radio.send_packet(packet)

#time.sleep(.1)
#print("Time: %d" % (time.time() - start_time))
#packet = Packet.generate(PacketType.HEARTBEAT, [255, 255, 255])
#radio.send_packet(packet)


# Send a ping
#packet = Packet.generate(PacketType.PING, [12, 34])
#radio.send_packet(packet)

#received = radio.read_packet()
#while True:
	#if received is not None:
	#	print("Time: %d" % (time.time() - start_time))
	#	print("Received: %s" % received)
	#received = radio.read_packet()
	#time.sleep(.5)
	#heartbeat = Packet.generate(PacketType.HEARTBEAT, [10, 10, 10])
	#radio.send_packet(heartbeat)