"""
Spawns a daemon thread that:
1.Receives data from a socket
2.Unpickles if necessary 
3.Adds message to a Queue for further processing.

Socket data and message Queue are provided by a drone_network.Network object

Author: Leonidas Antoniou 
mail: leonidas.antoniou@gmail.com
"""

import threading, select, hashlib, time, socket, logging
import cPickle as pickle

class ReceiveThread(threading.Thread):

	def __init__(self, network, q):
		threading.Thread.__init__(self)
		self.daemon = True
		self.network = network
		self.msg_queue = q
		self.count = 0
		self.t_timing = []
		self.t_checksum = []
		self.t_iterations = 1

	def run(self):
		while True:
			t_start = time.time()

			try:
				ready = select.select([self.network.sock_receive], [], [], 2.0) #wait until a message is received - timeout 2s

				if ready[0]:
					
					d = self.network.sock_receive.recvfrom(4096)
					raw_msg = d[0]
					sender_addr = d[1]
					sender_ip = (d[1])[0]

					#Keep verified messages, ignore the rest
					try:
						msg = pickle.loads(raw_msg)
						if self.verify_md5_checksum(msg):
							data = pickle.loads(msg[0])
							
							if data.ID == self.network.vehicle_params.ID: #Ignore messages from self. ID is given based on uuid at initialization time
								pass

							else:
								self.msg_queue.put((data, sender_addr, sender_ip))
								self.count = self.count + 1
						else:
							logging.warning("Received wrong data, ignoring")
						
					except pickle.UnpicklingError:
						pass
					
			except (select.error, socket.error),  e:
				logging.debug("Error in receive_thread: %s", e)
				#Failsafe
				break

			self.t_timing.append(time.time() - t_start)
			self.t_iterations += 1

	def verify_md5_checksum(self, raw_msg):

		t_start = time.time()

		if type(raw_msg) is tuple:

			#Get MD5 of received message
			m = hashlib.md5()
			m.update(raw_msg[0])
			received_data_hashed = m.digest()

			#Get received checksum
			received_checksum = raw_msg[1]

			if received_data_hashed == received_checksum:
				#print 'Message verified!'
				self.t_checksum.append(time.time() - t_start)
				self.t_iterations += 1
				return True

			else:
				self.t_checksum.append(time.time() - t_start)
				self.t_iterations += 1
				return False

		else:
			self.t_checksum.append(time.time() - t_start)
			self.t_iterations += 1
			return None

	def get_timing(self):
		try:
			average_timing = sum(self.t_timing)/self.t_iterations
			average_checksum = sum(self.t_checksum)/self.t_iterations

			max_timing = max(self.t_timing)
			max_checksum = max(self.t_checksum)

			logging.info("\nPrinting average execution times of receive_thread")
			logging.info("Thread: %s\nChecksum: %s", average_timing, average_checksum)

			logging.info("\nPrinting max execution times of receive_thread")
			logging.info("Thread: %s\nChecksum: %s", max_timing, max_checksum)

		except:
			logging.debug("Not enough data to calculate execution times")

