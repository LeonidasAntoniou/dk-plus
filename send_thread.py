import threading, time, hashlib, socket, logging
import cPickle as pickle

class SendThread(threading.Thread):

	def __init__(self, network, address):
		threading.Thread.__init__(self)
		self.daemon = True
		self.network = network
		self.address = address

	def run(self):
		#These functions will be running concurrently until the end of the main process 
		#This is achieved by creating a daemon thread for each one of the broad and listen functions

		while True:
			try:
				data = pickle.dumps(self.network.vehicle_params)
				checksum = self.create_md5_checksum(data)

				msg = (data, checksum)
				pickled_msg = pickle.dumps(msg)

			except pickle.UnpicklingError, e:
				data = " "
				logging.debug("Pickling Error: %s",e)
			
			try:
				self.network.sock_send.sendto(pickled_msg, self.address)

			except socket.error, e:
				logging.debug("Failed to broadcast: %s", e)
				#Failsafe
				break

			time.sleep(self.network.POLL_RATE) #broadcast every POLL_RATE seconds

	def create_md5_checksum(self, data):
		#Create MD5 checksum for message verification
		m = hashlib.md5()
		m.update(data)
		return m.digest()