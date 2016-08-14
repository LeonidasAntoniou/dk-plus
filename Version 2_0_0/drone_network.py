"""
The absolutely necessary class to interface your drone with the outer world. 
-Responsible for opening/closing sockets according to a chosen protocol (currently only broadcast messaging through UDP).
-Invokes the send/receive/task threads.
-Contains the vehicle's drone list and message queue.

Author: Leonidas Antoniou 
mail: leonidas.antoniou@gmail.com
"""

import socket, select, Queue, select, time, threading
import cPickle as pickle

#Custom modules
import geo_tools as geo
from params import Params

from send_thread import SendThread
from receive_thread import ReceiveThread
from receive_task_thread import ReceiveTaskThread

class Networking:
	MAX_STAY = 5 #seconds until entry is removed from structure
	SAFETY_ZONE = 40 #metres
	CRITICAL_ZONE = 10 #metres

	def __init__(self, address, protocol, vehicle):
		self.address = address
		self.vehicle = vehicle
		self.protocol = protocol
		self.msg_queue = Queue.Queue()

		self.vehicle_params = Params(vehicle=vehicle)
		self.drones = []

		#First entry in drones list will be our own vehicle
		self.drones.append(self.vehicle_params)

		#Normally it is kept as is until the first receive_thread
		self.drones = self.populate_drones()

		self.sock_send = None
		self.sock_receive = None

		#Create transceiver and worker threads
		self.t_send = SendThread(self)
		self.t_receive = ReceiveThread(self)
		self.t_task = ReceiveTaskThread(self)

	def run(self):
		#Start networking protocol
		if self.protocol == "UDP_BROADCAST":
			if self.create_udp_broadcast(self.address):
				print "Network interface started succesfully"
			else:
				print "Network interface not started, exit..."
				#status -> critical
				#run failsafe
		else:
			#You can use whatever protocol you want here
			print "Unsupported protocol"
			#raise signal
		
		#Start threads
		self.t_receive.start()
		self.t_send.start()
		self.t_task.start()

	def stop(self):
		self.sock_receive.close()	#Release the resource
		self.sock_send.close()	#Release the resource


	def create_udp_broadcast(self, address):
		try:
			#Setting up a UDP socket for sending broadcast messages
			self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
			self.sock_send.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

			#Setting up a UDP socket for receiving broadcast messages
			#Set to non-blocking because we have select() for that reason
			self.sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
			self.sock_receive.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
			self.sock_receive.setblocking(0)
			self.sock_receive.bind(self.address)
			exit_status = 1

		except Exception, e:
			print "Error in socket operation: ", e
			exit_status = 0

		finally:
			return exit_status

	def populate_drones(self):
		"""
		Gives dummy values to test priorities
		Correct priority is: 9 7 4 5 2 3 1 8 0 6
		"""
		drones = []
		entries = []

		for i in range(0, 10):
			entries.append(Params(dummy=True))

		entries[0].ID = 0
		entries[0].mission_importance = 1
		entries[0].system_status = 'CALIBRATING'
		entries[0].mode = 'MANUAL'
		entries[0].set_global_alt = False
		entries[0].set_attitude = False
		entries[0].battery_level = 90
		entries[0].last_recv = time.time() + 50

		entries[1].ID = 1
		entries[1].mission_importance = 1
		entries[1].system_status = 'POWEROFF'
		entries[1].mode = 'AUTO'
		entries[1].set_global_alt = True
		entries[1].set_attitude = True
		entries[1].battery_level = 43
		entries[1].last_recv = time.time() + 50

		entries[2].ID = 2
		entries[2].mission_importance = 0
		entries[2].system_status = 'ACTIVE'
		entries[2].mode = 'MANUAL'
		entries[2].set_global_alt = True
		entries[2].set_attitude = False
		entries[2].battery_level = 43
		entries[2].last_recv = time.time() + 50

		entries[3].ID = 3
		entries[3].mission_importance = 0
		entries[3].system_status = 'ACTIVE'
		entries[3].mode = 'GUIDED'
		entries[3].set_global_alt = True
		entries[3].set_attitude = True
		entries[3].battery_level = 95
		entries[3].last_recv = time.time() + 50

		entries[4].ID = 4
		entries[4].mission_importance = 0
		entries[4].system_status = 'ACTIVE'
		entries[4].mode = 'RTL'
		entries[4].set_global_alt = False
		entries[4].set_attitude = False
		entries[4].battery_level = 20
		entries[4].last_recv = time.time() + 50

		entries[5].ID = 5
		entries[5].mission_importance = 1
		entries[5].system_status = 'ACTIVE'
		entries[5].mode = 'POSHOLD'
		entries[5].set_global_alt = True
		entries[5].set_attitude = True
		entries[5].battery_level = 43
		entries[5].last_recv = time.time() + 50

		entries[6].ID = 6
		entries[6].mission_importance = 2
		entries[6].system_status = 'POWEROFF'
		entries[6].mode = 'MANUAL'
		entries[6].set_global_alt = False
		entries[6].set_attitude = False
		entries[6].battery_level = 100
		entries[6].last_recv = time.time() + 50

		entries[7].ID = 7
		entries[7].mission_importance = 1
		entries[7].system_status = 'ACTIVE'
		entries[7].mode = 'MANUAL'
		entries[7].set_global_alt = False
		entries[7].set_attitude = False
		entries[7].battery_level = 83
		entries[7].last_recv = time.time() + 50

		entries[8].ID = 8
		entries[8].mission_importance = 1
		entries[8].system_status = 'CALIBRATING'
		entries[8].mode = 'MANUAL'
		entries[8].set_global_alt = False
		entries[8].set_attitude = False
		entries[8].battery_level = 60
		entries[8].last_recv = time.time() + 50

		entries[9].ID = 9
		entries[9].mission_importance = 0
		entries[9].system_status = 'EMERGENCY'
		entries[9].mode = 'AUTO'
		entries[9].set_global_alt = True
		entries[9].set_attitude = True	
		entries[9].battery_level = 43
		entries[9].last_recv = time.time() + 50
		
		for i in range(0, 10):
			drones.append(entries[i])

		return drones
