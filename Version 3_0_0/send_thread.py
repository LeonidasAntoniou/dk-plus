import multiprocessing
import time
import cPickle as pickle

class SendProcess(multiprocessing.Process):
	def __init__(self, network):
		multiprocessing.Process.__init__(self)
		self.daemon = True
		self.network = network

	def run(self):
		#These functions will be running concurrently until the end of the main process 
		#This is achieved by creating a daemon thread for each one of the broad and listen functions

		while True:
			try:
				data = pickle.dumps(self.network.vehicle_params, pickle.HIGHEST_PROTOCOL)

			except Exception, e:
				data = " "
				print "Pickling Error: ", e
			
			try:
				self.network.sock_send.sendto(data, self.network.address)

			except Exception, e:
				print "Failed to broadcast: ", e
				#Failsafe
				break

			time.sleep(self.network.POLL_RATE) #broadcast every POLL_RATE seconds