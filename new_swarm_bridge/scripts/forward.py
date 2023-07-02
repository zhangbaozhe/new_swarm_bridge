#! /usr/bin/python

# Author: Baozhe Zhang
# Note: All in one script for message forwarding in robot swarm applications

import copy
import socket
import sys, signal
from io import BytesIO
import subprocess
from threading import Thread, Lock

import zmq
import rostopic
import rospy

PORT = 5555

def signal_handler(signal, frame):
    print("Program finished")
    sys.exit(0)

# TODO: make this in another thread
# TODO: check ip format
class PeerIpTracker:
    def __init__(self, my_ip, broadcast_ip, port, max_peer_num):
        self._my_ip = my_ip
        self._broadcast_ip = broadcast_ip
        self._port = port
        self._max_peer_num = max_peer_num

        self._peer_ips = []
        self._broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # UDP
        self._broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self._recv_socket.bind(('', self._port))
        self._recv_socket.setblocking(False)
    
    @property
    def my_ip(self):
        return self._my_ip

    @property
    def broadcast_ip(self):
        return self._broadcast_ip
    
    @property
    def port(self):
        return self._port
    
    @property
    def max_peer_num(self):
        return self._max_peer_num

    @property
    def peer_ips(self):
        return copy.deepcopy(self._peer_ips)

    def update(self):
        """update peer ips"""
        for i in range(self._max_peer_num):
            # self._recv_socket.settimeout(0.001)
            try: 
                ip, _ = self._recv_socket.recvfrom(128)
            except:
                print("[PeerIpTracker] [Warn]")
                continue
            ip = ip.decode('ascii')
#             ip_split = []
#             try:
#                 ip_split = ip.split('.')
#             except ValueError as e:
#                 print(e)
#                 print("Ip ill-formatted in the given broadcast ip: \
# expected x.x.x.x")
#                 sys.exit(1)
#             for item in ip_split:
#                 if item.isnumeric():
#                     continue
#                 else:
#                     print("Ip ill-formatted in the given broadcast ip: \
# expected number x.x.x.x")
#                     sys.exit(1)
            # now we are good 
            if ip in self._peer_ips or ip == self._my_ip:
                continue
            # now we are good to append
            self._peer_ips.append(ip)

    def broadcast(self):
        """broadcast my ip"""
        m = bytes(self._my_ip, 'ascii')
        self._broadcast_socket.sendto(m, (self._broadcast_ip, self._port))
                
class PeerMsgForwarder:
    def __init__(self, my_ip, robot_id, topics):
        self._my_ip = my_ip
        self._robot_id = robot_id
        self._topics = topics
        self._peer_ips = []
        self._zmq_context = zmq.Context()
        self._zmq_pub_socket = self._zmq_context.socket(zmq.SocketType.PUB)
        self._zmq_pub_socket.bind("tcp://{}:{}".format(self._my_ip, PORT))
        self._zmq_sub_sockets = {}
        # { ip : { t1 : pub1, ...}, ...} lazy update
        self._ros_publishers = {}
        # { t1 : sub1, ...}
        self._ros_subscribers = {}
        # { t1: thread1, ...}
        self._threads = {}
        # mutex
        self._mutex = Lock()
        

    @property
    def my_ip(self):
        return self._my_ip

    @property
    def robot_id(self):
        return self._robot_id

    @property
    def topics(self):
        return copy.deepcopy(self.topics)

    @property
    def peer_ips(self):
        return copy.deepcopy(self._peer_ips)
    
    @property
    def threads(self):
        return self._threads
    
    @peer_ips.setter
    def peer_ips(self, val):
        """update peer ips and update their sockets"""
        self._peer_ips = copy.deepcopy(val)
        for ip in self._peer_ips:
            if ip in self._zmq_sub_sockets:
                continue
            self._zmq_sub_sockets[ip] = \
                self._zmq_context.socket(zmq.SocketType.SUB)
            print("{} connecting to tcp://{}:{}".format(self._my_ip, ip, PORT))
            self._zmq_sub_sockets[ip].connect("tcp://{}:{}".format(ip, PORT))
            self._zmq_sub_sockets[ip].setsockopt(zmq.SUBSCRIBE, b'')
            self._threads[ip] = Thread(target=self.listen_and_publish_one, args=(ip, ))
            self._threads[ip].start()

    def generic_callback(self, data, args):
        self._mutex.acquire()
        print("[s_a_f] sent")
        t = args[0]
        msg_class = args[1]
        buff = BytesIO()
        data.serialize(buff)
        forward_msg = (self._robot_id, t, msg_class, buff.getvalue())
        self._zmq_pub_socket.send_pyobj(forward_msg)
        self._mutex.release()
    
    def subscribe_and_forward(self):
        """get the topics from local ROS network and pub to other robots"""
        for t in self._topics:
            # topic should be e.g., /0/imu or /1/drone/imu ... starting with robot id
            robot_topic = '/' + str(self._robot_id) + '/' + t
            # print(robot_topic)
            msg_class, _, _ = rostopic.get_topic_class(robot_topic)
            if msg_class == None:
                print("Topic name not properly set")
                sys.exit(1)
            if robot_topic not in self._ros_subscribers:
                self._ros_subscribers[robot_topic] = rospy.Subscriber(
                    robot_topic, msg_class, self.generic_callback, callback_args=(t, msg_class), tcp_nodelay=True, queue_size=1)

    def listen_and_publish_one(self, peer_ip):
        while True: 
            # event.wait(1.0 / len(self._topics) / 100) # assuming the topics are pub at 100Hz each
            # print("I'm in thread")
            # rospy.Rate(100).sleep()
            if peer_ip not in self._ros_publishers:
                self._ros_publishers[peer_ip] = {}
            m = self._zmq_sub_sockets[peer_ip].recv_pyobj()
            ros_msg = m[2]()
            ros_msg.deserialize(m[3])
            robot_topic = '/' + str(m[0]) + '/' + m[1]
            # if not set, set
            if robot_topic not in self._ros_publishers[peer_ip]:
                self._ros_publishers[peer_ip][robot_topic] = \
                    rospy.Publisher(robot_topic, m[2], queue_size=1, tcp_nodelay=True)
            self._ros_publishers[peer_ip][robot_topic].publish(ros_msg)
        

            
    
    def listen_and_publish(self):
        """iterate the sockets, update the publishers, and forward"""
        # FIXME: don't if this has performance issue, since it cannot handle concurrent pub
        # for ip in self._peer_ips:
        #     if ip not in self._ros_publishers:
        #         self._ros_publishers[ip] = {}
        #     for i in range(len(self._topics)):
        #         # TODO: this is a blocking statement, use a timeout
        #         # self._zmq_sub_sockets[ip].RCVTIMEO = 1000
        #         try:
        #             m = self._zmq_sub_sockets[ip].recv_pyobj(zmq.NOBLOCK)
        #         except:
        #             continue
        #         ros_msg = m[2]()
        #         ros_msg.deserialize(m[3])
        #         robot_topic = '/' + str(m[0]) + '/' + m[1]
        #         # if not set, set
        #         if robot_topic not in self._ros_publishers[ip]:
        #             self._ros_publishers[ip][robot_topic] = \
        #                 rospy.Publisher(robot_topic, m[2], queue_size=1)
        #         self._ros_publishers[ip][robot_topic].publish(ros_msg)
        if (len(self._peer_ips) == 0):
            return
        # with futures.ThreadPoolExecutor(max_workers=len(self._peer_ips) * len(self._topics)) as exe:
            # exe.map(self.listen_and_publish_one, self._peer_ips)

        print('[l_a_p] published')
    

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('forward')
    my_ip = sys.argv[1]
    # my_ip = '192.168.51.2'
    proc = subprocess.Popen('ifconfig', stdout=subprocess.PIPE)
    while True:
        line = proc.stdout.readline()
        if my_ip.encode() in line:
            break
    broadcast_ip = line.decode('ascii').split('broadcast')[-1].replace('\n', '').strip()
    print(broadcast_ip)
    peer_ip_tracker = PeerIpTracker(my_ip, broadcast_ip, 5005, 10)
    forwarding_topics = []
    forwarding_topics = str(sys.argv[3]).split()
    forwarder = PeerMsgForwarder(my_ip, int(sys.argv[2]), forwarding_topics)
    # forwarder = PeerMsgForwarder(my_ip, 0, ['string0', 'string1', 'imu0', 'imu1'])
    forwarder.subscribe_and_forward()

    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        rate.sleep()
        for j in range(10):
            peer_ip_tracker.broadcast()
        peer_ip_tracker.update()
        peer_ips = peer_ip_tracker.peer_ips
        print(peer_ip_tracker.peer_ips)
        forwarder.peer_ips = peer_ips
        

    
    rospy.spin()




    
