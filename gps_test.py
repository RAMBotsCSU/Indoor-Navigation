import os
import time
import threading
from gps import *

gpsd = None  # Global GPS object


class GpsPoller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        global gpsd
        gpsd = gps(mode=WATCH_ENABLE | WATCH_NEWSTYLE)
        self.running = True

    def run(self):
        global gpsd
        while self.running:
            gpsd.next()  # Grab the next GPS packet


if __name__ == "__main__":
    gpsp = GpsPoller()
    gpsp.start()

    try:
        while True:
            os.system("clear")

            print("")
            print(" GPS Reading")
            print("----------------------------------------")
            print("Latitude:     ", gpsd.fix.latitude)
            print("Longitude:    ", gpsd.fix.longitude)
            print("Altitude (m): ", gpsd.fix.altitude)
            print("Speed (m/s):  ", gpsd.fix.speed)
            print("Track:        ", gpsd.fix.track)
            print("Climb:        ", gpsd.fix.climb)
            print("Mode:         ", gpsd.fix.mode)

            # Time handling
            print("Time UTC:     ", gpsd.utc, "+", gpsd.fix.time)

            # Satellites (depends on gpsd version)
            try:
                print("Satellites:   ", len(gpsd.satellites))
            except:
                print("Satellites:   unavailable")

            time.sleep(1)  # Update every seco
    except (KeyboardInterrupt, SystemExit):
        gsps.running = False
        gsps.join()
        print("Exiting")
