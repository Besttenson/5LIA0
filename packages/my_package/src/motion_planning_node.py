import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct subscriber
        self.camera_to_mp = rospy.Subscriber('MotionPlanning', Float32MultiArray, self.motion_planning)
        self.mo_to_control = rospy.Publisher('duckie_control', Float32MultiArray, queue_size=1)
    def motion_planning(self, data):
        distance = data.data[0]
        angle = data.data[1]
        object_length = data.data[2]
        type = data.data[3]

        moving_dist = 0
        rotating_angle = 0

        if type == 0:
            moving_dist = distance - object_length
        else:
            rotating_angle = angle

        control_msg = Float32MultiArray()
        control_msg.data = [moving_dist, rotating_angle]
        self.mo_to_control.publish(control_msg)
        
        rospy.loginfo("Motion planning data: ", data.data)

if __name__ == '__main__':
    # create the node
    motion_planning_node = MySubscriberNode(node_name='motion_planning_node')
    # keep spinning
    rospy.spin()