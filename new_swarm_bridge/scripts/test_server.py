#!/usr/bin/env python

from Bridge import BridgeServer, ServerDataPackage

import sys
import signal
import threading
import time
import gzip
import pickle

import rospy
from sensor_msgs.msg import Imu

def signal_handler(signal, frame):
    print("Forcing to exit")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    server = BridgeServer()
    server.init()

    rospy.init_node('redirect')
    pub = rospy.Publisher('temp_imu_redirected', Imu, queue_size=1)
    t = threading.Thread(target=server.run, args=(8004,))

    try: 
        t.start()
        time.sleep(1)

        while True: 
            packages = server.get_latest_data_packages()
            if len(packages) > 0: 
                # print("ready to read")
                # print(packages[0].get_data())
                data = packages[0].get_data()
                # print(len(packages))
                # print(len(data))
                # print(packages[0].get_size())
                data = gzip.decompress(data)
                data = pickle.loads(data)
                msg = data[0]()
                msg.deserialize(data[1])
                pub.publish(msg)

        t.join()
    except KeyboardInterrupt: 
        print("[test_server.py] Gracefully shutting down")
        server.stop()
        if t.is_alive():
            t.join()


