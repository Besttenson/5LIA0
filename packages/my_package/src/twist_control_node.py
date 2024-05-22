#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Int16


# Twist command for controlling the linear and angular velocity of the frame
VELOCITY = 0.03  # linear vel    , in m/s    , forward (+)
OMEGA = 0  # angular vel   , rad/s     , counter clock wise (+)


class TwistControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        # form the message
        self._v = VELOCITY
        self._omega = OMEGA
        # construct publisher
        self._publisher = rospy.Publisher("duckie_control", Int16, queue_size=1)
        # construct subscriber
        self._subscriber = rospy.Subscriber('chatter', Int16, self.callback)

    def callback(self, msg):
        # publish 10 messages every second (10 Hz)
        print("received msg: ", msg)
        rate = rospy.Rate(10)
        ref_angle = 0.5
        while not rospy.is_shutdown():
            self._publisher.publish(ref_angle)
            rate.sleep()

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = TwistControlNode(node_name='twist_control_node')
    # run node
    # node.run()
    # keep the process from terminating
    rospy.spin()