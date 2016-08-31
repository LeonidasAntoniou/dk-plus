"""
Spawns a daemon thread that:
1.Reads a message from the message queue
2.If it is drone data, it updates the vehicle's drone list accordingly
3.A timestamp, priority and distance between drones is given, too

Author: Leonidas Antoniou 
mail: leonidas.antoniou@gmail.com
"""

import threading, Queue, time, itertools
import geo_tools as geo
from collections import namedtuple
from operator import attrgetter

simple_msg = namedtuple("simple_msg", "ID text")

class ReceiveTaskThread(threading.Thread):

	def __init__(self, network):
		threading.Thread.__init__(self)
		self.daemon = True
		self.network = network

	def run(self):
		while True:

			try:
				(message, sender_addr, sender_ip) = self.network.msg_queue.get()

				if isinstance(message, simple_msg):
					print "Received from:", sender_addr, "Message: ", message.text

				else:
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

						#Add priority field
						message.priority = None

						#Update drones list
						found = False
						if len(self.network.drones) == 0: #first element
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

				self.network.msg_queue.task_done()
			
			except Queue.Empty:
				print "All messages processed"

			finally:
				#Remove entries that have not been updated for the last MAX_STAY seconds
				#self.network.drones = [item for item in self.network.drones if (time.time() - item.last_recv <= self.network.MAX_STAY)]

				#Update own vehicle parameter entry in the right position
				for i in range(0, len(self.network.drones)):
					if self.network.drones[i].ID == self.network.vehicle_params.ID:
						self.network.drones[index] = self.network.vehicle_params
						break				

				#Give priorities 
				self.give_priorities()
				time.sleep(1)

	"""
	-If drone is grounded then its priority is low, exempting mode 'ARMED'
	-System status needs to be checked first, if it's critical it has the greatest capability of all
	-----If status is OK then highest priority is given to highest mission importance first
	---------Among same mission importances, highest priority is given to the ones with less capabilities
	------------If they have the same status, mission importance, capabilities fine-tune with battery level
	"""

	def give_priorities(self):

		#Temporary drone list
		drones = self.network.drones

		#Temporary priority lists
		top = []
		high = []
		medium = []
		low = []

		#Need to find the specific code block in the project for these...
		grounded_state = ['UNINIT', 'BOOT', 'POWEROFF', 'STANDBY', 'CALIBRATING', 'LOCKED']
		auto_modes = ['RTL', 'LOITER', 'AUTO', 'GUIDED', 'CIRCLE', 'LAND', 'BRAKE', 'LIFTOFF']
		manual_modes = ['ALT_HOLD', 'STABILIZE', 'MANUAL', 'ACRO', 'SPORT', 'DRIFT', 'POSHOLD', 'SIMPLE', 'SUPER_SIMPLE']
		

		#Assign each drone to its priority list
		for i in range(0, len(drones)):
			has_capabilities = drones[i].set_global_alt or drones[i].set_attitude
			has_mayday = (drones[i].system_status == 'CRITICAL') or (drones[i].system_status == 'EMERGENCY')

			#Manual flying drones without capabilities 
			#Drones in Emergency/Critical state
			if (drones[i].mode in manual_modes and not has_capabilities and drones[i].system_status not in grounded_state) or has_mayday:
				top.append(drones[i])

			#Top importance drones in flying state or ready to fly				
			elif drones[i].mission_importance == 2 and drones[i].system_status not in grounded_state or\
				drones[i].mission_importance == 2 and drones[i].system_status not in grounded_state:
				high.append(drones[i])

			#Drones not in level-2 importance
			#Drones in flying state or armed with remote capabilities 
			#Drones flying or armed in one of the automatic flying modes
			elif (drones[i].mode in auto_modes and drones[i].system_status not in grounded_state ) or \
				(drones[i].mode in manual_modes and has_capabilities and drones[i].system_status not in grounded_state):
				medium.append(drones[i])

			#Grounded drones with low importance
			elif drones[i].system_status in grounded_state:
				low.append(drones[i])

		#Sort each sublist based on mission importance (top and high priority lists don't need that)
		medium.sort(key=attrgetter('mission_importance'), reverse=True)
		low.sort(key=attrgetter('mission_importance'), reverse=True)

		#Fine-tune sorting with battery level
		top.sort(key=attrgetter('battery_level'))
		high.sort(key=attrgetter('battery_level'))
		medium.sort(key=attrgetter('battery_level'))
		low.sort(key=attrgetter('battery_level'))

		#Combine everything back to drones list
		drones = [top, high, medium, low]
		self.network.drones = list(itertools.chain.from_iterable(drones))


