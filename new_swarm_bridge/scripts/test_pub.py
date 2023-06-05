#! /usr/bin/python

import sys

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('test_pub')
    robot_id = sys.argv[1]

    string_pubs = []
    imu_pubs = []

    string_pubs.append(rospy.Publisher('/'+robot_id+'/string0',
                       String, tcp_nodelay=True, queue_size=1))
    string_pubs.append(rospy.Publisher('/'+robot_id+'/string1',
                       String, tcp_nodelay=True, queue_size=1))
    imu_pubs.append(rospy.Publisher('/'+robot_id+'/imu0',
                    Imu, tcp_nodelay=True, queue_size=1))
    imu_pubs.append(rospy.Publisher('/'+robot_id+'/imu1',
                    Imu, tcp_nodelay=True, queue_size=1))

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.z = 9.8
        string_msg = String()
        string_msg.data = 'I am robot ' + robot_id

        for i in string_pubs:
            i.publish(string_msg)

        for j in imu_pubs:
            j.publish(imu_msg)
