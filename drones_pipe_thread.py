from multiprocessing import Pipe
import threading, select, time

class PipeThread(threading.Thread):
	def __init__(self, network):
		threading.Thread.__init__(self)
		self.daemon = True
		self.network = network
		self.collision_conn, self.drone_conn = Pipe(duplex=False)

	def run(self):

		feed_interval = self.network.POLL_RATE

		while True:

			#send the conflicting drone parameters, the running instance and the current parameters
			msg = (self.network.drones, self.network.vehicle.mode, self.network.vehicle_params)
			self.drone_conn.send(msg)
			time.sleep(feed_interval)