"""
This piece of code is DEPRECATED
"""

"""
I will try to create a UDP socket for broadcasting messages.
In the same time, there is going to be a listener for messages as well.
"""

import time, socket, json, threading 

def broad(params):
	threading.Timer(1.0, broad, params).start()
	address = ('<broadcast>', 54545)

	s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

	while True:
		try:
			if type(params)!='string':
				data = json.dumps(params)
			else:
				data = params
		except Exception, e:
			data = " "
			print "Error in thread: ", e

		if self._stop.isSet():
			break
		if s.sendto(data, address):
			print("Message sent")
			time.sleep(1)

	s.close()
	print("Socket closed")

"""
class broadcastThread (threading.Thread):
    def __init__(self, threadID, name, counter, params):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
        self._stop = threading.Event()
        self.params = params

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

    def run(self):
		address = ('<broadcast>', 54545)

		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

		while True:
			try:
				if type(params)!='string':
					data = json.dumps(params)
				else:
					data = params
			except Exception, e:
				data = " "
				print "Error in thread: ", e

			if self._stop.isSet():
				break
			if s.sendto(data, address):
				print("Message sent")
				time.sleep(1)

		s.close()
		print("Socket closed")
"""

"""
#For testing purposes
thread1 = broadcastThread(1, "1", 1)
thread1.start()

for i in range(0, 3):
	print "while broadcasting"
	time.sleep(2)

print thread1.stopped()
thread1.stop()
print thread1.stopped()
"""


