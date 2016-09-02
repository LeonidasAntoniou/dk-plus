import threading, time, hashlib
import cPickle as pickle

class SendThread(threading.Thread):


	def __init__(self, network):
		threading.Thread.__init__(self)
		self.daemon = True
		self.network = network

	def run(self):
		#These functions will be running concurrently until the end of the main process 
		#This is achieved by creating a daemon thread for each one of the broad and listen functions

		while True:
			try:
				data = pickle.dumps(self.network.vehicle_params)
				checksum = create_md5_checksum(data)
				message = self.network.params_message(data, checksum)

			except Exception, e:
				data = " "
				print "Pickling Error: ", e
			
			try:
				self.network.sock_send.sendto(message, self.network.address)

			except Exception, e:
				print "Failed to broadcast: ", e
				#Failsafe
				break

			time.sleep(self.network.POLL_RATE) #broadcast every POLL_RATE seconds

	def create_md5_checksum(data):
		#Create MD5 checksum for message verification
		m = hashlib.md5()
		m.update(data)
		return m.digest()