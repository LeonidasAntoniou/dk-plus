import threading, time, hashlib, socket, logging
import cPickle as pickle

class SendThread(threading.Thread):

	def __init__(self, network):
		threading.Thread.__init__(self)
		self.daemon = True
		self.network = network
		self.t_timing = []
		self.t_iterations = 1

	def run(self):
		#These functions will be running concurrently until the end of the main process 
		#This is achieved by creating a daemon thread for each one of the broad and listen functions

		while True:

			t_start = time.time()

			try:
				data = pickle.dumps(self.network.vehicle_params)
				checksum = self.create_md5_checksum(data)

				msg = (data, checksum)
				pickled_msg = pickle.dumps(msg)

			except pickle.UnpicklingError, e:
				data = " "
				logging.debug("Pickling Error: %s",e)
			
			try:
				self.network.sock_send.sendto(pickled_msg, self.network.address)

			except socket.error, e:
				logging.debug("Failed to broadcast: %s", e)
				#Failsafe
				break

			self.t_timing.append(time.time() - t_start)
			self.t_iterations += 1

			time.sleep(self.network.POLL_RATE) #broadcast every POLL_RATE seconds

	def create_md5_checksum(self, data):
		#Create MD5 checksum for message verification
		m = hashlib.md5()
		m.update(data)
		return m.digest()

	def get_timing(self):
		try:
			average_timing = 1000*sum(self.t_timing)/self.t_iterations
			max_timing = 1000*max(self.t_timing)

			logging.info("\nPrinting execution times of send_thread")
			logging.info("Average: %s ms\nMax: %s ms", average_timing, max_timing)

		except:
			logging.debug("Not enough data to calculate execution times")