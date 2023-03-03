#! /usr/bin/env python2
# -*- coding: utf-8 -*-

import time, threading
from datetime import datetime
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from ros_node_manager import ProcessWrapper, ProcessMonitor

G_NODE_NAME_ = "daemon_node"
G_WORKER_LIST_ = [
    {"idx": 0, "name": "apa_node_local", "cmdline": "roslaunch apa_node apa_carla_local.launch"},
    {"idx": 1, "name": "apa_data_local", "cmdline": "rosbag play --loop /home/hugoliu/github/catkin_ws/src/VIW-Fusion/data/apa_microlino_720p_20221227.bag"},
    {"idx": 2, "name": "apa_node_remote", "cmdline": "roslaunch apa_node apa_carla_remote.launch"},
    # {"idx": 0, "name": "apa_node_local", "cmdline": "python /home/hugoliu/github/catkin_ws/src/VIW-Fusion/daemon_node/tools/loop_test.py"},
    # {"idx": 1, "name": "apa_data_local", "cmdline": "python /home/hugoliu/github/catkin_ws/src/VIW-Fusion/daemon_node/tools/loop_test.py"},
    # {"idx": 2, "name": "apa_node_remote", "cmdline": "python /home/hugoliu/github/catkin_ws/src/VIW-Fusion/daemon_node/tools/loop_test.py"},
    {"idx": 3, "name": "l2_node_local", "cmdline": "sleep 300"},
    {"idx": 4, "name": "l2_data_local", "cmdline": "sleep 300"},
    {"idx": 5, "name": "l2_node_remote", "cmdline": "sleep 300"},
]

class RosDaemonNode(object):
    def __init__(self):
        self.init_node(G_NODE_NAME_)
        full_data_source_name = rospy.search_param('data_source_apa')
        self.data_source_apa = rospy.get_param(full_data_source_name, default="local")
        rospy.loginfo("param {}: {}".format(full_data_source_name, self.data_source_apa))
        self.pmon = ProcessMonitor()
        self.process_list_apa = []
        if self.data_source_apa == "local":
            self.process_list_apa.append(G_WORKER_LIST_[0])
            self.process_list_apa.append(G_WORKER_LIST_[1])
        elif self.data_source_apa == "remote":
            self.process_list_apa.append(G_WORKER_LIST_[2])

        full_data_source_name = rospy.search_param('data_source_l2')
        self.data_source_l2 = rospy.get_param(full_data_source_name, default="local")
        rospy.loginfo("param {}: {}".format(full_data_source_name, self.data_source_l2))
        self.process_list_l2 = []
        if self.data_source_l2 == "local":
            self.process_list_l2.append(G_WORKER_LIST_[3])
            self.process_list_l2.append(G_WORKER_LIST_[4])
        elif self.data_source_l2 == "remote":
            self.process_list_l2.append(G_WORKER_LIST_[5])

    def run(self):
        self.start_apa()
        rospy.spin()
        self.stop_apa()
        self.stop_l2()

    def start_apa(self):
        for dict in self.process_list_apa:
            rospy.loginfo("try start process: {}".format(dict))
            pw = ProcessWrapper(command_line=dict["cmdline"], name=dict["name"])
            if self.pmon.register(pw):
                pw.start()
                time.sleep(2.0)

    def stop_apa(self):
        for dict in self.process_list_apa:
            rospy.loginfo("try stop process: {}".format(dict))
            self.pmon.stop(kill_name=dict["name"])
        self.pmon.check_cleanup()

    def start_l2(self):
        for dict in self.process_list_l2:
            rospy.loginfo("try start process: {}".format(dict))
            pw = ProcessWrapper(command_line=dict["cmdline"], name=dict["name"])
            if self.pmon.register(pw):
                pw.start()
                time.sleep(2.0)

    def stop_l2(self):
        for dict in self.process_list_l2:
            rospy.loginfo("try stop process: {}".format(dict))
            self.pmon.stop(kill_name=dict["name"])
        self.pmon.check_cleanup()

    # cmd from remote, 0: start l2 && stop apa, 1: start apa && stop l2
    def sub_callback(self, data):
        rospy.logwarn("{} recv cmd: {}".format(rospy.get_caller_id(), data.data))
        if data.data > 0:
            self.stop_l2()
            time.sleep(1.0)
            self.start_apa()
        elif data.data < 1:
            self.stop_apa()
            time.sleep(1.0)
            self.start_l2()

    def heartbeat(self):
        while not rospy.is_shutdown():
            now = datetime.now()
            dt_string = now.strftime("%Y-%m-%d-%H-%M-%S")
            rospy.loginfo("{} heartbeat: {}".format(G_NODE_NAME_, dt_string))
            self.pub_heartbeat.publish(dt_string)
            time.sleep(3.0)

    def start_heartbeat(self):
        topic_name = "{}/heartbeat".format(G_NODE_NAME_)
        self.pub_heartbeat = rospy.Publisher(name=topic_name, data_class=String, queue_size=10)
        th = threading.Thread(target=self.heartbeat, args=())
        th.setDaemon(True)
        th.start()

    def init_node(self, node_name):
        rospy.init_node(node_name)
        self.start_heartbeat()
        topic_name = "{}/cmd".format(G_NODE_NAME_)
        self.sub_cmd = rospy.Subscriber(name=topic_name, data_class=Int32, callback=self.sub_callback, queue_size=10)
        

if __name__ == "__main__":
    node = RosDaemonNode()
    node.run()
    print("{} exit".format(G_NODE_NAME_))
