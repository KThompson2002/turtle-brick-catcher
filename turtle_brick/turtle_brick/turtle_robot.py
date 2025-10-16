from enum import Enum, auto
import rclpy
import math

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from turtlesim_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# Custom msg/srv
from turtle_brick_interfaces.msg import Tilt

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    START = auto()
    MOVING = auto(),
    STOPPED = auto(),

class TurtleRobot():
    def __init__(self):
        super().__init__('turtle_robot')
        # Static Broadcaster:
        self.static_broadcasters = StaticTransformBroadcaster(self)
        
        # Creating Transforms:
        
        qos_profile = QoSProfile(depth=10) # durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        #self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        
        #joint_state = JointState()
        
        # Establish initial state
        self.timer = self.create_timer(1/self.freq, self.timer_callback)
        self.state = State.START
        self.pose = None
        
        
        #self.odom_trans = None
        
        # Declare parameters
        self.declare_parameter("frequency", 90.0)
        
        # Create Parameters
        self.freq = self.get_parameter("frequency").get_parameter_value().double_value
        
        # Create subscribers
        self._vel = self.create_subscription(Pose, "turtle1/pose", self.vel_callback, qos_profile)
        self._goal = self.create_subscription(PoseStamped, "goal_pose", self.goal_callback, qos_profile)
        # self._tilt = self.create_subscription(Tilt, "", qos_profile)

    def get_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
        # Create Publishers
        self._pub = self.create_publisher(Twist, "turtle1/cmd_vel", qos_profile)
        # self._odom = self.create_publisher(Odometry, "odometery", qos_profile)
        
        #create the broadcaster
        self.broadcaster = TransformBroadcaster(self)
        
    def timer_callback(self):
        if self.state == State.START and self.pose is not None:
            world_base_tf = TransformStamped()
            world_base_tf.header.stamp = self.get_clock().now().to_msg()
            world_base_tf.header.frame_id = 'world'
            
            odom_trans = TransformStamped()
            odom_trans.header.frame_id = 'odom'
            odom_trans.child_frame_id = 'base_link'
            world_base_tf.child_frame_id = 'odom'
            world_base_tf.transform.translation.x = self.pose.x
            world_base_tf.transform.translation.y = self.pose.y
            time = self.get_clock().now().to_msg()
            self.state = State.STOPPED
            self.broadcaster.sendTransform(self.odom_trans)
            
            
        elif self.state == State.MOVING:
            twist = self.turtle_twist()
            self._pub.publish(twist)
            
    
    def vel_callback(self, pose):
        dx, dy = pose.x - self.pose.x, pose.y - self.pose.y
        self.pose = pose
        if self.state == State.MOVING:
            odom_trans = TransformStamped()
            odom_trans.header.frame_id = 'odom'
            odom_trans.child_frame_id = 'base_link'
            odom_trans.transform.translation.x = dx
            odom_trans.transform.translation.y = dy
            
            odom_trans.header.stamp = self.get_clock().now().to_msg()
            self.broadcaster.sendTransform(odom_trans)
            
        
    def goal_callback(self, pose):
        self.curr_waypoint = pose
        if self.state == State.STOPPED:
            self.state = State.MOVING
        return
    
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
            return vel_msg
        else:
            return Twist()
        

        
def main(args=None):
    rclpy.init(args=args)
    node = TurtleRobot()
    rclpy.spin(node) 
    rclpy.shutdown()