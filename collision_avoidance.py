
"""
Handler for the collision avoidance part of the module.
A specific drone_network.Networking object needs to be running
Custom algorithms can be specified
Spawns a thread by instantiation

Author: Leonidas Antoniou 
mail: leonidas.antoniou@gmail.com
"""
import threading, time
import geo_tools as geo


class CollisionThread(threading.Thread):
	def __init__(self, network, algorithm=None):
		threading.Thread.__init__(self)
		self.daemon = True
		self.network = network
		self.near = []
		self.critical = []
		self.algorithm = algorithm

	def run(self):
		while True:
			self.near = []
			self.critical = []
			drone_list = self.network.drones			

			#No drones in range
			if len(drone_list) == 0:
				pass
			
			else: 
				#This value is slightly not concurrent 
				own = self.network.vehicle_params

				#From the detected drones, add any within a SAFETY-to-CRITICAL-metre range
				self.near = [item for item in drone_list if \
					(geo.get_distance_metres(own.global_lat, own.global_lon, item.global_lat, item.global_lon) <= self.network.SAFETY_ZONE)\
					&(geo.get_distance_metres(own.global_lat, own.global_lon, item.global_lat, item.global_lon) > self.network.CRITICAL_ZONE)\
					& (abs(own.global_alt - item.global_alt) <= self.network.SAFETY_ZONE)]


				#From the near drones, add any within a CRITICAL-metre range (ignore ourselves)
				self.critical = [item for item in drone_list if \
					(item.ID != own.ID) \
					&(geo.get_distance_metres (own.global_lat, own.global_lon, item.global_lat, item.global_lon) <= self.network.CRITICAL_ZONE) \
					& (abs(own.global_alt - item.global_alt) <= self.network.CRITICAL_ZONE)]

				
				#Print info
				for i in range(0, len(self.near)):
					print "Drone approaching! ID: ", (self.near[i]).ID

				for i in range(0, len(self.critical)):
					print "Drone too close!!!! ID: ", (self.critical[i]).ID

				#Run collision avoidance algorithm
				self.run_algorithm()				

	#Deploy your collision avoidance algorithm here
	def run_algorithm(self):
		if self.algorithm == None:
			time.sleep(1)
		else:
			pass
