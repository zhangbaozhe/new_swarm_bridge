#!/usr/bin/env python

from Bridge import BridgeClient, ClientDataPackage

import sys
import threading
import time
import gzip 
import pickle
from io import BytesIO
import signal

import rospy
from sensor_msgs.msg import Imu

def signal_handler(signal, frame):
    print("Forcing to exit")
    sys.exit(0)

if __name__ == "__main__": 
    signal.signal(signal.SIGINT, signal_handler)



    client = BridgeClient()
    client.init()
    t = threading.Thread(target=client.run, args=('127.0.0.1', 8004,))

    try: 
        rospy.init_node('pub')
        pub = rospy.Publisher('temp_imu', Imu, queue_size=1)
        t.start()
        time.sleep(1)

        rate = rospy.Rate(100)
        while True: 
            package = ClientDataPackage()         
            # print(time.localtime())
            # data = bytes(str(time.time()) + ' hello\0', encoding='utf-8')
            msg = Imu()
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            buff = BytesIO()
            msg.serialize(buff)
            data = (Imu, buff.getvalue())
            data = pickle.dumps(data)
            data = gzip.compress(data)
            print(len(data))
            package.set_size(len(data))
            package.set_data(data)
            client.set_data_packages([package])
            rate.sleep()

        t.join()
    except KeyboardInterrupt: 
        print("[test_server.py] Gracefully shutting down")
        client.stop()
        if t.is_alive():
            t.join()
