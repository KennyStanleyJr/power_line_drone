import threading
from pymavlink import mavutil

class MavHeartbeatThread(threading.Thread):
    def __init__(self, interval=1.):
        super(MavHeartbeatThread, self).__init__()
        self.interval = interval
        self.running = False

    def run(self):
        self.running = True
        self.mavlink_connection = mavutil.mavlink_connection('0.0.0.0:14550', autoreconnect=True, baud=57600)
        self.mavlink_connection.wait_heartbeat()
        while self.running:
            self.mavlink_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
            time.sleep(self.interval)

    def stop(self):
        self.running = False
