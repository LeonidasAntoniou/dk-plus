"""
A simple program that sends/listens broadcast packets through UDP socket
Used to test the system if it is able to send/receive packets
"""
import time, math, sys, socket, threading, select, uuid
from collections import namedtuple
import cPickle as pickle
from params import Params

roundtrip = []
iterations = 0

MAX_STAY = 5 #seconds until entry is removed from structure
simple_msg = namedtuple("simple_msg", "time text")
self_params = Params(dummy=True)

# Set the socket parameters
address = ('192.168.1.255', 54545)  # host, port
sock_broad = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock_broad.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Create socket and bind to address
sock_listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_listen.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock_listen.bind(address)

def broad():
	global iterations
	while True:
		msg = simple_msg(time.time(), self_params)
		assert sock_broad.sendto(pickle.dumps(msg), address), "Message failed to send"
		iterations += 1
		time.sleep(0.5)

def listen():
	global iterations
	print "Waiting for message"
	while True:
		try:
			ready = select.select([sock_listen], [], [], 1.0)
			if ready[0]:
				d = sock_listen.recvfrom(4096)
				raw_msg = d[0]

				try:
					msg = pickle.loads(raw_msg)
					roundtrip.append(time.time() - msg.time) 
					if msg.text.ID == self_params.ID:
						pass
						#print "received from ourselves..."
					else:
						print "From addr: '%s', msg: '%s'" % (d[1], msg.text)

				except Exception, e:
					print "Error in receiving: ", e

		except socket.timeout:
			print "Reached timeout. Closing..."
			t_listen.cancel()
			sock_listen.close()

t_listen = threading.Thread(target=listen)
t_broad = threading.Thread(target=broad)

t_listen.daemon = True
t_broad.daemon = True

t_listen.start()
t_broad.start()

time.sleep(100) #test for 100s
print "Average roundtrip: ", (sum(roundtrip)/iterations)*1000, "ms"
print "Max roundtrip: ", max(roundtrip)*1000, "ms"
print "Closing beacon"

