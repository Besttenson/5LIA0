
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Int16
from duckietown_msgs.msg import Twist2DStamped, CompressedImage
from duckietown.dtros import DTROS, NodeType, TopicType

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct subscriber
        self.sub = rospy.Subscriber('duckie_control', Int16, self.pid)
        car_cmd_topic = f"/{self.veh}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic,
            Twist2DStamped,
            queue_size=1,
            dt_topic_type=TopicType.CONTROL
        )

        self.sub_image = rospy.Subscriber(
            f"/{self.veh}/camera_node/image/compressed",
            CompressedImage,
            self.get_header,
            buff_size=10000000,
            queue_size=1,
        )

    def get_header(self, data):
        self.header = data.header

    def pid(self, data):
        v_0, omega, e, e_int = self.PIDController(0.3, data, 0.5, 0, 0, 0.1)
        car_control_msg = Twist2DStamped()
        car_control_msg.header = self.header
        car_control_msg.v = v_0

        # always drive straight
        car_control_msg.omega = omega

        self.pub_car_cmd.publish(car_control_msg)

    def PIDController(self, v_0, theta_ref, theta_hat, prev_e, prev_int, delta_t): #add theta_ref as input
        """
        Args:
            v_0 (:double:) linear Duckiebot speed (given).
            theta_ref (:double:) reference heading pose
            theta_hat (:double:) the current estiamted theta.
            prev_e (:double:) tracking error at previous iteration.
            prev_int (:double:) previous integral error term.
            delta_t (:double:) time interval since last call.
        returns:
            v_0 (:double:) linear velocity of the Duckiebot 
            omega (:double:) angular velocity of the Duckiebot
            e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
            e_int (:double:) current integral error (automatically becomes prev_int_y at next iteration).
        """
    
    # Tracking error
        e = theta_ref - theta_hat

        # integral of the error
        e_int = prev_int + e*delta_t

        # anti-windup - preventing the integral error from growing too much
        e_int = max(min(e_int,2),-2)

        # derivative of the error
        e_der = (e - prev_e)/delta_t

        # controller coefficients
        Kp = 5
        Ki = 0.2
        Kd = 0.1

        # PID controller for omega
        omega = Kp*e + Ki*e_int + Kd*e_der
        
        #print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {u} \nTheta hat: {np.rad2deg(theta_hat)} \n")
        
        return v_0, omega, e, e_int
    
if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()