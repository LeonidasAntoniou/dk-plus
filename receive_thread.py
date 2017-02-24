"""
Spawns a daemon thread that:
1.Receives data from a socket
2.Unpickles if necessary 
3.Adds message to a Queue for further processing.

Socket data and message Queue are provided by a drone_network.Network object

Author: Leonidas Antoniou 
mail: leonidas.antoniou@gmail.com
"""

import threading, select, hashlib, socket, logging
import cPickle as pickle


class ReceiveThread(threading.Thread):
    def __init__(self, network, msg_queue, debug=False):
        threading.Thread.__init__(self)
        self.daemon = True
        self.network = network
        self.msg_queue = msg_queue
        self.count = 0
        self.debug = debug

    def run(self):
        while True:
            try:
                ready = select.select([self.network.sock_receive], [], [],
                                      1.0)  # wait until a message is received - timeout 2s

                if ready[0]:

                    d = self.network.sock_receive.recvfrom(4096)
                    raw_msg = d[0]
                    sender_addr = d[1]
                    sender_ip = (d[1])[0]

                    # Keep verified messages, ignore the rest
                    try:
                        msg = pickle.loads(raw_msg)
                        if self.verify_md5_checksum(msg):
                            data = pickle.loads(msg[0])

                            if data.ID == self.network.vehicle_params.ID:  # Ignore messages from self. ID is given based on uuid at initialization time
                                pass

                            else:
                                self.msg_queue.put((data, sender_addr, sender_ip))
                                logging.info("Received msg from: %s", sender_addr)
                                logging.info("SYSID_THISMAV", data.SYSID_THISMAV)

                                self.count += 1
                        else:
                            logging.warning("Received wrong data, ignoring")

                    except pickle.UnpicklingError:
                        pass

            except (select.error, socket.error), e:
                logging.debug("Error in receive_thread: %s", e)
                # Failsafe
                break

    def verify_md5_checksum(self, raw_msg):

        if type(raw_msg) is tuple:
            # Get MD5 of received message
            m = hashlib.md5()
            m.update(raw_msg[0])
            received_data_hashed = m.digest()

            # Get received checksum
            received_checksum = raw_msg[1]

            if received_data_hashed == received_checksum:
                # print 'Message verified!'
                return True

            else:
                return False

        else:
            return None
