"""
Spawns a daemon thread that:
1.Receives data from a socket
2.Unpickles if necessary 
3.Adds message to a Queue for further processing.

Socket data and message Queue are provided by a drone_network.Network object

Author: Leonidas Antoniou 
mail: leonidas.antoniou@gmail.com
"""

import threading, select, hashlib
import cPickle as pickle

class ReceiveThread(threading.Thread):

	def __init__(self, network, q):
		threading.Thread.__init__(self)
		self.daemon = True
		self.network = network
		self.msg_queue = q


	def run(self):
		while True:

			try:
				ready = select.select([self.network.sock_receive], [], [], 2.0) #wait until a message is received - timeout 2s

				if ready[0]:
					d = self.network.sock_receive.recvfrom(4096)
					raw_msg = d[0]
					sender_addr = d[1]
					sender_ip = (d[1])[0]

					#Keep verified messages, ignore the rest
					try:
						if verify_md5_checksum(raw_msg):
							msg = pickle.loads(raw_msg.data)
							
							if msg.ID == self.network.vehicle_params.ID: #Ignore messages from self. ID is given based on uuid at initialization time
								pass

							else:
								self.msg_queue.put((msg, sender_addr, sender_ip))
									
						else:
							print "Received wrong data, ignoring"
						
					except pickle.UnpicklingError:
						pass
					
			except Exception, e:
				print "Error in socket (most probably because socket is closed by another thread): ", e
				#Failsafe
				break
			
	def verify_md5_checksum(self, raw_msg):

		if type(raw_msg) is self.network.params_message:

			#Get MD5 of received message
			m = hashlib.md5()
			m.update(raw_msg.data)
			received_data_hashed = m.digest()

			#Get received checksum
			received_checksum = raw_msg.checksum

			if received_data_hashed == received_checksum:
				return True

			else:
				return False

		else:
			return None
