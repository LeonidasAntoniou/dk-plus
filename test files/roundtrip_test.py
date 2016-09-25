"""
A simple program that calculates the time for a message to be sent to a node and then received back.

"""
import time, math, sys, socket, threading, select, uuid, logging, datetime
from collections import namedtuple
import cPickle as pickle
from params import Params

roundtrip = []
iterations = 0
timeout = 40 #s
frequency = 100 #messages/s

MAX_STAY = 5 #seconds until entry is removed from structure
simple_msg = namedtuple("simple_msg", "roundtrip time text")
self_params = Params(dummy=True)

logging.basicConfig(filename='roundtrip_timing', level=logging.INFO)
logging.info("Date and time: %s", datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S'))

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
		msg = simple_msg(False, time.time(), self_params)
		assert sock_broad.sendto(pickle.dumps(msg), address), "Message failed to send"
		iterations += 1
		print_progress()
		time.sleep(1/float(frequency))

def listen():
	while True:
		try:
			ready = select.select([sock_listen], [], [], 1.0)
			if ready[0]:
				d = sock_listen.recvfrom(4096)
				raw_msg = d[0]

				try:
					#Unpickle
					msg = pickle.loads(raw_msg)

					"""
					#Determine source
					if msg.text.ID == self_params.ID:
						pass
						#print "received from ourselves..."
					else:
						print "From addr: '%s', msg: '%s'" % (d[1], msg.text)
					"""
					#If it comes from a roundtrip, calculate time. 
					#Else send again for roundtrip.
					if msg.roundtrip:
						roundtrip.append(time.time() - msg.time) 

					else:
						msg = simple_msg(True, msg.time, msg.text)
						assert sock_broad.sendto(pickle.dumps(msg), address), "Roundtrip message failed to send"
					
				except Exception, e:
					print "Error in receiving: ", e

		except socket.timeout:
			print "Reached timeout. Closing..."
			t_listen.cancel()
			sock_listen.close()

def print_progress():
	sys.stdout.write("\r")
	progress = 100*iterations/(frequency*timeout)
	remaining = timeout-iterations/frequency
	sys.stdout.write("%s%%\t%ss remaining" % (progress, remaining))
	sys.stdout.flush()

t_listen = threading.Thread(target=listen)
t_broad = threading.Thread(target=broad)

t_listen.daemon = True
t_broad.daemon = True

logging.info("Opening beacon, counting...")
t_listen.start()
t_broad.start()

time.sleep(timeout) #test for 100s

logging.info("\nCalculating roundtrip timing after %s iterations with %s frequency", iterations, frequency)
logging.info("Average roundtrip: %s ms", (sum(roundtrip)/iterations)*1000) 
logging.info("Max roundtrip: %s ms", max(roundtrip)*1000)
logging.info("Closing beacon")

