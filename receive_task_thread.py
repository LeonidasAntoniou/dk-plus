"""
Spawns a daemon thread that:
1.Reads a message from the message queue
2.If it is drone data, it updates the vehicle's drone list accordingly
3.A timestamp, priority and distance between drones is given, too

Author: Leonidas Antoniou 
mail: leonidas.antoniou@gmail.com
"""

import threading, Queue, time, socket, logging
import geo_tools as geo
from collections import namedtuple

remote_action = namedtuple("remote_action", "ID action params")


class ReceiveTaskThread(threading.Thread):

	def __init__(self, network, q):
		threading.Thread.__init__(self)
		self.daemon = True
		self.network = network
		self.msg_queue = q
		self.count = 0
		self.t_iterations = 1
		self.t_timing = []

	def run(self):
		while True:
			t_start = time.time()

			try:
				(message, sender_addr, sender_ip) = self.msg_queue.get()
				self.network.add_delay()
				self.count = self.count + 1
				
				#print "Received drone info" 

				#Calculate distance between drones
				lat1 = self.network.vehicle_params.global_lat
				lon1 = self.network.vehicle_params.global_lon
				lat2 = message.global_lat
				lon2 = message.global_lon
				message.distance_from_self = geo.get_distance_metres(lat1, lon1, lat2, lon2)

				#Ignore drones that are beyond safety zone
				if message.distance_from_self > self.network.SAFETY_ZONE:
					#print "Drone ", message.ID, " not dangerous"
					pass

				else:
					#Add timestamp
					message.last_recv = time.time()

					#Add extra fields for collision avoidance purposes
					message.priority = None

					#Update drones list
					found = False
					if len(self.network.drones) == 1: #first element apart from us
						self.network.drones.append(message)

					#Update values if message comes from the same sender or insert new sender
					else:
						for i in range(0, len(self.network.drones)):
							if self.network.drones[i].ID == message.ID:	#Registered entry
								self.network.drones[i] = message
								found = True
								break
						if found == False:	#New entry
							self.network.drones.append(message)

				self.msg_queue.task_done()
			
			except Queue.Empty:
				logging.debug("All messages processed")
	
			self.t_timing.append(time.time() - t_start)
			self.t_iterations += 1

	def get_timing(self):
		try:
			average_timing = 1000*sum(self.t_timing)/self.t_iterations
			max_timing = 1000*max(self.t_timing)

			logging.info("Printing average and max execution times of receive_task_thread")
			logging.info("Average: %s ms", average_timing)
			logging.info("Max: %s ms", max_timing)

		except:
			logging.debug("Not enough data to calculate execution times")

	


