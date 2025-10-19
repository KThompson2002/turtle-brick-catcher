from enum import Enum, auto
import rclpy
import math

from rclpy.node import Node
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from turtlesim_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from transforms3d.quaternions import axangle2quat

# Custom msg/srv
from turtle_brick_interfaces.msg import Tilt

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    START = auto(),
    MOVING = auto(),
    STOPPED = auto()


def quatToMsg(quat):
    """
    Convert a four-element sequence to a geometry_msgs/msg/Quaternion.

    Parameters
    ----------
    quat : list
         A four element sequence [w, x, y, z] representing a quaternion

    Returns
    -------
    geometry_msgs.msg.Quaternion
         The corresponding Quaternion message

    """
    return Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])


class TurtleRobot(Node):
    def __init__(self):
        super().__init__('turtle_robot')
        # Static Broadcaster:
        self.static_broadcasters = StaticTransformBroadcaster(self)
        
        # Creating Transforms:
        
        qos_profile = QoSProfile(depth=10) # durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        #joint_state = JointState()
        # Declare parameters
        self.declare_parameter("frequency", 90.0)
        self.declare_parameter("platform_height", 9.0)
        self.declare_parameter("wheel_radius", 2.0)
        self.declare_parameter("max_velocity", 3.0)
        self.declare_parameter("gravity_accel", 9.81)
        
        # Create Parameters
        self.freq = self.get_parameter("frequency").get_parameter_value().double_value
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.gravity = self.get_parameter("gravity_accel").get_parameter_value().double_value
        self.disp = self.wheel_radius*2 + (0.5+0.2)
        
        # Establish initial state
        self.timer = self.create_timer(1/self.freq, self.timer_callback)
        self.state = State.START
        self.pose = None
        self.curr_waypoint = None
        self._velocity = 1.5
        self.stem_angle = 0.0
        self.wheel_angle = 0.0
        
        #Create Tilt Subscriber
        self.platform_tilt = 0.0
        self._tilt = self.create_subscription(Tilt, "/tilt", self.tilt_callback, qos_profile)
        
        self.curr_vel = None
        
        
        # Create subscribers
        self._vel = self.create_subscription(Pose, "/pose", self.vel_callback, qos_profile)
        self._goal = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, qos_profile)
        self._tilt = self.create_subscription(Tilt, "/tilt", self.tilt_callback, qos_profile)
        # Create Publishers
        self._pub = self.create_publisher(Twist, "/cmd_vel", qos_profile)
        self._joint = self.create_publisher(JointState, "/joint_states", qos_profile)
        self._arrive = self.create_publisher(Bool, "/arrive", qos_profile)
        
        # self._odom = self.create_publisher(Odometry, "odometery", qos_profile)
        
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ["platform_joint", "stem_joint", "wheel_joint"]
        joint_msg.position = [0.0, 0.0, 0.0]
        self._joint.publish(joint_msg)
        
        #create the broadcaster
        self.broadcaster = TransformBroadcaster(self)
        
    def timer_callback(self):
        if self.state == State.START and self.pose is not None:
            world_odom_tf = TransformStamped()
            world_odom_tf.header.stamp = self.get_clock().now().to_msg()
            world_odom_tf.header.frame_id = 'world'
            world_odom_tf.child_frame_id = 'odom'
            
            world_odom_tf.transform.translation.x = 5.544
            world_odom_tf.transform.translation.y = 5.544
            world_odom_tf.transform.rotation.w = 1.0
            # time = self.get_clock().now().to_msg()
            self.state = State.STOPPED
            self.static_broadcasters.sendTransform(world_odom_tf)
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = ["platform_joint", "stem_joint", "wheel_joint"]
            joint_msg.position = [0.0, 0.0, 0.0]
            self._joint.publish(joint_msg)
        
            
            
        elif self.state == State.MOVING and self.curr_waypoint is not None:
            twist = self.turtle_twist()
            self._pub.publish(twist)
            self.translate_robot()
            if get_distance([self.curr_waypoint.x, self.curr_waypoint.y], [self.pose.x, self.pose.y]) < 0.01:
                arrive_msg = Bool()
                if self.pose.y < 5.544:
                    arrive_msg.data = True
                else:
                    arrive_msg.data = False
                
                self._arrive.publish(arrive_msg)
                self.state = State.STOPPED
        else:
            odom_trans = TransformStamped()
            odom_trans.header.stamp = self.get_clock().now().to_msg()
            odom_trans.header.frame_id = 'odom'
            odom_trans.child_frame_id = 'base_link'
            odom_trans.transform.translation.z = self.disp
            self.broadcaster.sendTransform(odom_trans)
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = ["platform_joint", "stem_joint", "wheel_joint"]
            joint_msg.position = [self.platform_tilt, 0.0, 0.0]
            self._joint.publish(joint_msg)
        
        
    
    def vel_callback(self, pose):
        self.pose = pose
        
    def goal_callback(self, msg: Pose):
        self.curr_waypoint = msg.pose.position
        if self.state == State.STOPPED or self.state == State.START:
            self.state = State.MOVING
        return
    
    def tilt_callback(self, tilt):
        self.platform_tilt = tilt.tilt_angle
        
    def turtle_twist(self):
        """ Create a twist which moves the turtle proportionally towards the next waypoint """
        if self.state == State.MOVING:
            #Establish initial important variables
            vel_msg = Twist()
            dx = self.curr_waypoint.x - self.pose.x
            dy = self.curr_waypoint.y - self.pose.y
            x_vel = dx * self._velocity
            y_vel = dy * self._velocity
            vel_msg.linear.x = x_vel
            vel_msg.linear.y = y_vel
            self.curr_vel = vel_msg
            return vel_msg
        else:
            return Twist()
    
    def translate_robot(self):
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        
        # if self.curr_vel is not None:
        #     odom_trans.transform.translation.x = self.curr_vel.linear.x
        #     odom_trans.transform.translation.y = self.curr_vel.linear.y
        if self.pose is not None:
            odom_trans.transform.translation.x = self.pose.x - 5.544
            odom_trans.transform.translation.y = self.pose.y - 5.544
            odom_trans.transform.translation.z = self.disp
            odom_trans.transform.rotation = quatToMsg(axangle2quat([0, 0, 1.0], self.pose.theta))
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(odom_trans)
        
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ["platform_joint", "stem_joint", "wheel_joint"]
        
        if self.curr_vel is not None:
            xdot = self.curr_vel.linear.x
            ydot = self.curr_vel.linear.y
            if math.hypot(xdot, ydot) > 0:
                self.stem_angle = math.atan2(xdot, ydot)
            
            ang_vel = math.sqrt(xdot**2 + ydot**2)/self.wheel_radius
            self.wheel_angle += ang_vel * (1/self.freq)
            self.wheel_angle = math.atan2(math.sin(self.wheel_angle), math.cos(self.wheel_angle))
            
                
        
        joint_msg.position = [self.platform_tilt, self.stem_angle, self.wheel_angle]
            
        self._joint.publish(joint_msg)

        
def get_distance(point1, point2):
        """ Calculates straight line distance between two poses
        
        point1 - initial point
        point2 - end point
        """
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
      
        
def main(args=None):
    rclpy.init(args=args)
    node = TurtleRobot()
    rclpy.spin(node) 
    rclpy.shutdown()