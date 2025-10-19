from enum import Enum, auto
import rclpy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from turtle_brick_interfaces.msg import Tilt

class BrickState(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    STATIC = auto(),
    DROPPING = auto(),
    CARRYING = auto()


class Catcher(Node):
    
    def __init__(self):
        super().__init__('catcher')
        qos_profile = QoSProfile(depth=10)
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        
        # Declare Parameters
        self.declare_parameter("frequency", 90.0)
        self.declare_parameter("platform_height", 9.0)
        self.declare_parameter("wheel_radius", 2.0)
        self.declare_parameter("max_velocity", 3.0)
        self.declare_parameter("gravity_accel", 9.81)
        self.declare_parameter("platform_radius", 2.0)

        # Establish Parameters
        self.freq = self.get_parameter("frequency").get_parameter_value().double_value
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.gravity = self.get_parameter("gravity_accel").get_parameter_value().double_value
        self.platform_rad = self.get_parameter("platform_radius").get_parameter_value().double_value

        
        # Create publisher
        self._goal = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)        
        # Create subscribers for arena node
        self.create_subscription(Bool, 'drop_event', self.drop_callback, qos_profile)
        # self.create_subscription(Bool, 'caught_event', self.caught_callback, qos_profile)
        self.create_subscription(Bool, '/arrive', self.arrive_callback, qos_profile)
        
        # Create publisher for turtle-robot node
        self._tilt = self.create_publisher(Tilt, '/tilt', qos_profile)
        
        # Establish initial conditions
        self.brick_state = BrickState.STATIC
        
        # Establish tf2 listener
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        
        # Create Marker Publisher
        self.pub = self.create_publisher(Marker, '/catcher', markerQoS)
        
        # Create timer
    #     self.timer = self.create_timer(1/self.freq, self.timer_callback)
    
    # def timer_callback(self):
    #     if self.brick_state == BrickState.CARRYING:
            # Find brick location
            # if self.platform_tilt > 0.0:
            #     self.world.tilt()
            
        
    def drop_callback(self, msg):
        self.brick_state = BrickState.DROPPING
        
        # Look up base_link location
        t = self.tf_buffer.lookup_transform(
            'odom',
            'base_link',
            rclpy.time.Time())
        base_x = t.transform.translation.x + 5.544
        base_y = t.transform.translation.y + 5.544
        
        # Look up brick location
        b = self.tf_buffer.lookup_transform(
            'odom',
            'brick',
            rclpy.time.Time())
        brick_x = b.transform.translation.x + 5.544
        brick_y = b.transform.translation.y + 5.544
        brick_z = b.transform.translation.z
        
        # Drop time equals sqrt(2*height / gravity)
        drop_time = math.sqrt((2*(brick_z - self.platform_height))/self.gravity)
        self._logger.info(f"Base Location: {base_x}, {base_y}")
        self._logger.info(f"Brick Location: {brick_x}, {brick_y}")
        travel_dist = get_distance([base_x, base_y], [brick_x, brick_y])
        self._logger.info(f"Travel Distance: {travel_dist}")
        travel_time = travel_dist / self.max_velocity
        self._logger.info(f"Travel Time: {travel_time}")
        
        if travel_time < drop_time:
            goal_msg = PoseStamped()
            goal_msg.pose.position.x = brick_x
            goal_msg.pose.position.y = brick_y
            self._logger.info(f"Traveling to: {travel_time}")
            self._goal.publish(goal_msg)
        else:
            mark = Marker()
            mark.header.frame_id = "odom"
            mark.header.stamp = self.get_clock().now().to_msg()
            mark.type = Marker.TEXT_VIEW_FACING
            mark.text = "Brick Cannot be Caught"
            self._logger.info("Brick cannot be traveled to")
            mark.pose.position.x = 0.0
            mark.pose.position.y = 0.0
            mark.pose.position.z = 10.0
            mark.scale.z = 2.0
            self.pub.publish(mark)
    
    def arrive_callback(self, msg):
        if self.brick_state == BrickState.CARRYING:
            if msg.data:
                tilt_msg = Tilt()
                tilt_msg.tilt_angle = (11*math.pi)/6
                self._tilt.publish(tilt_msg)
            else:
                tilt_msg = Tilt()
                tilt_msg.tilt_angle = math.pi/6
                self._tilt.publish(tilt_msg)
            
            self.brick_state == BrickState.STATIC
        else:
            # t = self.tf_buffer.lookup_transform(
            #     'odom',
            #     'base_link',
            #     rclpy.time.Time())
            # base_y = t.transform.translation.y + 5.544
            goal_msg = PoseStamped()
            goal_msg.pose.position.x = 5.544
            if msg.data:
                goal_msg.pose.position.y = 5.544 - self.platform_rad
            else:
                goal_msg.pose.position.y = 5.544 + self.platform_rad
            self._goal.publish(goal_msg)
            
            self.brick_state = BrickState.CARRYING
            
        
        
def get_distance(point1, point2):
        """ Calculates straight line distance between two poses
        
        point1 - initial point
        point2 - end point
        """
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
            
def main(args=None):
    
    """Entry point for the arenaNode."""
    rclpy.init(args=args)
    node = Catcher()
    rclpy.spin(node)
    rclpy.shutdown()
    
    