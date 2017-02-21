"""
A simple program that sends/listens broadcast packets through UDP socket
Used to test the system if it is able to send/receive packets
"""
import time, math, sys, socket, threading, select, uuid
from collections import namedtuple
import cPickle as pickle
from params import Params

MAX_STAY = 5  # seconds until entry is removed from structure
Geo = namedtuple("Geo", "lat lon")
simple_msg = namedtuple("simple_msg", "ID text")
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
    while True:
        # msg = simple_msg(self_id,"I am here")
        msg = self_params
        assert sock_broad.sendto(pickle.dumps(msg), address), "Message failed to send"
        time.sleep(1)


def listen():
    print "Waiting for message"
    while True:
        try:
            ready = select.select([sock_listen], [], [], 1.0)
            if ready[0]:
                d = sock_listen.recvfrom(4096)
                raw_msg = d[0]

                try:
                    msg = pickle.loads(raw_msg)
                    if msg.ID == self_params.ID:
                        pass
                    else:
                        print "From addr: '%s', msg: '%s'" % (d[1], msg)

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

time.sleep(100)  # test for 100s
print "Closing beacon"
