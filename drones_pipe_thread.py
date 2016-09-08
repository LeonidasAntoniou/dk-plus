from multiprocess import Pipe
import threading

class PipeThread(threading.Thread):
	def __init__(self, network):
		threading.Thread.__init__(self)
		self.daemon = True
		self.network = network
		self.drone_conn, self.collision_conn = Pipe()

	def run(self):
		while True:

			#send every POLL_RATE the drone parameters
			self.drone_conn.send(self.network.drones)
