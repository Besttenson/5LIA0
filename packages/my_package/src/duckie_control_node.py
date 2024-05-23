import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Int16
from duckietown_msgs.msg import Twist2DStamped, CompressedImage,WheelEncoderStamped
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import Float32MultiArray

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        self.R = 0.0318
        self.baseline = 0.1
        self.x_prev = 0
        self.y_prev = 0
        self.theta_prev = 0
        self.prev_ticks_left = 0
        self.prev_ticks_right = 0
        self.left_resolution = 0
        self.right_resolution = 0
        self.prev_int = 0
        self.prev_e = 0
        self.vel = 0

        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct subscriber
        self.mp_to_control = rospy.Subscriber('duckie_control', Float32MultiArray, self.pid)

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

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        # store data value
        self._ticks_left = data.data
        self.left_resolution = data.resolution

    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        # store data value
        self._ticks_right = data.data
        self.right_resolution = data.resolution

    def get_header(self, data):
        self.header = data.header

    def pid(self, data):

        dist = data.data[0]
        angle = data.data[1]

        ref_yaw = self.theta_curr+angle
        self.vel = dist / 2

        delta_phi_left = self.delta_phi(self._ticks_left, self.prev_ticks_left, self.left_resolution)
        delta_phi_right = self.delta_phi(self._ticks_right, self.prev_ticks_right, self.right_resolution)

        self.prev_ticks_left = self._ticks_left
        self.prev_ticks_right = self._ticks_right

        self.x_curr, self.y_curr, self.theta_curr = self.pose_estimation(self.R, self.baseline, self.x_prev, self.y_prev, self.theta_prev, delta_phi_left, delta_phi_right)
        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.theta_prev = self.theta_curr
        
        self.delta_t = 0.1
        v_0, omega, e, e_int = self.PIDController(self.vel, ref_yaw, self.theta_curr, self.prev_e, self.prev_int, self.delta_t)
        self.prev_e = e
        self.prev_int = e_int

        car_control_msg = Twist2DStamped()
        car_control_msg.header = self.header
        car_control_msg.v = v_0
        car_control_msg.omega = omega

        rospy.loginfo_once(f"vehicle vel: {v_0}")
        rospy.loginfo_once(f"vehicle heading: {omega}")

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
    
    def delta_phi(ticks: int, prev_ticks: int, resolution: int):
        """
            Args:
                ticks: Current tick count from the encoders.
                prev_ticks: Previous tick count from the encoders.
                resolution: Number of ticks per full wheel rotation returned by the encoder.
            Return:
                rotation_wheel: Rotation of the wheel in radians.
                ticks: current number of ticks.
        """

        delta_ticks = ticks-prev_ticks

        # Assuming no wheel slipping
        dphi = 2*np.pi*delta_ticks/resolution


        return dphi

    def pose_estimation(
        self,
        R: float,
        baseline: float,
        x_prev: float,
        y_prev: float,
        theta_prev: float,
        delta_phi_left: float,
        delta_phi_right: float,
    ):
        """
        Calculate the current Duckiebot pose using the dead-reckoning approach.

        Args:
            R:                  radius of wheel (assumed identical) - this is fixed in simulation,
                                and will be imported from your saved calibration for the real robot
            baseline:           distance from wheel to wheel; 2L of the theory
            x_prev:             previous x estimate - assume given
            y_prev:             previous y estimate - assume given
            theta_prev:         previous orientation estimate - assume given
            delta_phi_left:     left wheel rotation (rad)
            delta_phi_right:    right wheel rotation (rad)

        Return:
            x:                  estimated x coordinate
            y:                  estimated y coordinate
            theta:              estimated heading
        """

        # x_curr = x_prev + R*(delta_phi_left+delta_phi_right)*np.cos(theta_prev)/2
        # y_curr = y_prev + R*(delta_phi_left+delta_phi_right)*np.sin(theta_prev)/2
        # theta_curr = theta_prev + R*(delta_phi_right-delta_phi_left)/(baseline)

    
        w = [R, 2*R / baseline, 1]

        x = np.array(
            [
                [
                    (delta_phi_left + delta_phi_right) * np.cos(theta_prev) / 2,
                    (delta_phi_left + delta_phi_right) * np.sin(theta_prev) / 2,
                    0,
                ],
                [0, 0, (delta_phi_right - delta_phi_left) / 2],
                [x_prev, y_prev, theta_prev],
            ]
        )

        x_curr, y_curr, theta_curr = np.array(w).dot(x)

        return x_curr, y_curr, theta_curr
if __name__ == '__main__':
    # create the node
    duckie_control_node = MySubscriberNode(node_name='duckie_control_node')
    # keep spinning
    rospy.spin()