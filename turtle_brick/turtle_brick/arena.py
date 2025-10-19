from enum import Enum, auto
import rclpy
import math


from geometry_msgs.msg import TransformStamped
# from interactive_markers.interactive_marker_server import (
#     InteractiveMarker, InteractiveMarkerServer)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker

from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from std_srvs.srv import Empty
from std_msgs.msg import Bool
from turtle_brick_interfaces.srv import Place

from turtle_brick import physics
from turtle_brick_interfaces.msg import Tilt

class BrickState(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    NONEXIST = auto()
    STATIC = auto(),
    TILTING = auto(),
    DROPPING = auto(),
    CAUGHT = auto()

class Arena(Node):
    """
    Simulates a turtle-sim sized arena using markers
    
    

    """
    
    def __init__(self):
        super().__init__('arena')
        
        qos_profile = QoSProfile(depth=10)
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        
        # Create marker publishers
        self.pub = self.create_publisher(Marker, 'arena_wall', markerQoS)
        self.brick_pub = self.create_publisher(Marker, 'brick_marker', markerQoS)
        
        # Declare Parameters
        self.declare_parameter("platform_height", 5.0)
        self.declare_parameter("wheel_radius", 1.0)
        self.declare_parameter("max_velocity", 1.5)
        self.declare_parameter("gravity_accel", 9.81)
        self.declare_parameter("platform_radius", 2.0)
        
        # Create Parameters
        self.freq = 250
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.gravity = self.get_parameter("gravity_accel").get_parameter_value().double_value
        self.platform_rad = self.get_parameter("platform_radius").get_parameter_value().double_value
        
        # Establish Timer
        self.timer = self.create_timer(1/self.freq, self.timer_callback)
        
        # Establish variables
        self.brick_state = BrickState.NONEXIST
        self.world = None
        self.publish_walls()
        self.brick_off_x = 0
        self.brick_off_y = 0
        # Instantiate Brick Marker
        self.brick_marker = Marker()
        
        # Create place service
        self._brick = self.create_service(Place, "place", self.place_callback)
        self._drop = self.create_service(Empty, "drop",  self.drop_callback)
        
        # Create tilt subscriber
        self.platform_tilt = 0.0
        self._tilt = self.create_subscription(Tilt, "/tilt", self.tilt_callback, qos_profile)
        
        
        # Create publisher for catcher Node
        self.drop_not = self.create_publisher(Bool, "drop_event", qos_profile)
        # self.catch_not = self.create_publisher(Bool, "caught_event", qos_profile)
        
        # Create the broadcaster
        self.broadcaster = TransformBroadcaster(self)
        
        # Create tf-tree listener
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        
    
    def timer_callback(self):
        if self.brick_state == BrickState.STATIC:
            self.broadcast_brick()
        elif self.brick_state == BrickState.DROPPING:
            self.world.drop()
            self.broadcast_brick()
            self.pub_brick_marker()
            
            # Get Brick location
            brick = self.world._brick
            
            # Establish TF tree listener
            t = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time())
            x = t.transform.translation.x + 5.544
            y = t.transform.translation.y + 5.544
            # self.get_logger().info(f"Listening: {x}, {y}")
            if get_distance([x,y], brick) < self.platform_rad and brick[2] <= self.platform_height:
                self.brick_state = BrickState.CAUGHT
                # msg = Bool()
                # msg.data = True
                # self.catch_not.publish(msg)
                self.brick_off_x = x - brick[0]
                self.brick_off_y = y - brick[1]
            elif brick[2] <= 0.0:
                self.brick_state = BrickState.STATIC
        elif self.brick_state == BrickState.CAUGHT:
            self.broadcast_brick()
            t = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time())
            x = t.transform.translation.x + 5.544
            y = t.transform.translation.y + 5.544
            self.world._brick[0] = x - self.brick_off_x
            self.world._brick[1] = y - self.brick_off_y
            self.broadcast_brick()
            self.pub_brick_marker()
    
    def tilt_callback(self, tilt):
        self.platform_tilt = tilt.tilt_angle
        # self.brick_state = BrickState.TILTING()
        
    
    def place_callback(self, request, response):
        """ Callback function for the custom Place service

        Moves brick to a specificied locations

        Args:
          request: Geometry_msgs/Point place
          response: Empty
          
        Empty Return
        """
        # Establish brick location
        x_loc = request.place.x
        y_loc = request.place.y
        z_loc = request.place.z
        
        # Establish physics world
        self.world = physics.World([x_loc, y_loc, z_loc], self.gravity, self.platform_rad, 1/self.freq)
        
        # Call Odom to brick Transfrom
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'brick'
        odom_trans.transform.translation.x = x_loc - 5.544
        odom_trans.transform.translation.y = y_loc - 5.544
        odom_trans.transform.translation.z = z_loc
        self.broadcaster.sendTransform(odom_trans)
        
        # Create and publish brick marker
        self.brick_marker.header.frame_id = "brick"
        self.brick_marker.header.stamp = self.get_clock().now().to_msg()
        self.brick_marker.ns = "brick_marker"
        self.brick_marker.id = 1
        self.brick_marker.type = Marker.CUBE
        self.brick_marker.action = Marker.ADD
        self.brick_marker.scale.x = 0.5
        self.brick_marker.scale.y = 1.0
        self.brick_marker.scale.z = 0.5
        self.brick_marker.pose.position.x = 0.0
        self.brick_marker.pose.position.y = 0.0
        self.brick_marker.pose.position.z = 0.0
        self.brick_marker.color.r = 1.0
        self.brick_marker.color.g = 1.0
        self.brick_marker.color.b = 0.0
        self.brick_marker.color.a = 1.0
        self.brick_pub.publish(self.brick_marker)
        
        #Change brick state
        self.brick_state = BrickState.STATIC
        return response
    
    def broadcast_brick(self):
        """ Continuously broadcast brick while it is static in the air """
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'brick'
        brick = self.world._brick
        odom_trans.transform.translation.x = brick[0] - 5.544
        odom_trans.transform.translation.y = brick[1] - 5.544
        odom_trans.transform.translation.z = brick[2]
        self.broadcaster.sendTransform(odom_trans)
    
    def pub_brick_marker(self):
        self.brick_marker.header.frame_id = "brick"
        self.brick_marker.header.stamp = self.get_clock().now().to_msg()
        self.brick_marker.ns = "brick_marker"
        self.brick_marker.id = 1
        self.brick_marker.action = Marker.MODIFY
        self.brick_marker.pose.position.x = 0.0
        self.brick_marker.pose.position.y = 0.0
        self.brick_marker.pose.position.z = 0.0
        self.brick_pub.publish(self.brick_marker)
    
    def drop_callback(self, request, response):
        """ Callback function for the drop service

        Causes brick to start falling in gravity using specficied gravity acceleration
        
        Specific Rules:
            If the brick hits the platform or ground it stops falling
            Assume tilt is 0 degrees
            If the brick is on the platform and it tilts, it should slide off the platform

        Args:
          request: Empty
          response: Empty
          
        Empty Return
        """
        self.brick_state = BrickState.DROPPING
        msg = Bool()
        msg.data = True
        self.drop_not.publish(msg)
        return response
    
    
    
    def publish_walls(self):
        self.m1 = Marker()
        self.m1.header.frame_id = "odom"
        self.m1.header.stamp = self.get_clock().now().to_msg()
        self.m1.ns = "arena"
        self.m1.id = 1
        self.m1.type =  Marker.CUBE
        self.m1.action = Marker.ADD
        self.m1.scale.x = 1.0
        self.m1.scale.y = 13.0
        self.m1.scale.z = 1.0
        self.m1.pose.position.x = -6.0
        self.m1.pose.position.y = 0.0
        self.m1.pose.position.z = 0.5
        self.m1.color.r = 1.0
        self.m1.color.g = 0.0
        self.m1.color.b = 0.0
        self.m1.color.a = 1.0
        self.pub.publish(self.m1)
        
        self.m2 = Marker()
        self.m2.header.frame_id = "odom"
        self.m2.header.stamp = self.get_clock().now().to_msg()
        self.m2.ns = "arena"
        self.m2.id = 2
        self.m2.type =  Marker.CUBE
        self.m2.action = Marker.ADD
        self.m2.scale.x = 1.0
        self.m2.scale.y = 13.0
        self.m2.scale.z = 1.0
        self.m2.pose.position.x = 6.0
        self.m2.pose.position.y = 0.0
        self.m2.pose.position.z = 0.5
        self.m2.color.r = 1.0
        self.m2.color.g = 0.0
        self.m2.color.b = 0.0
        self.m2.color.a = 1.0
        self.pub.publish(self.m2)
        
        self.m3 = Marker()
        self.m3.header.frame_id = "odom"
        self.m3.header.stamp = self.get_clock().now().to_msg()
        self.m3.ns = "arena"
        self.m3.id = 3
        self.m3.type =  Marker.CUBE
        self.m3.action = Marker.ADD
        self.m3.scale.x = 11.0
        self.m3.scale.y = 1.0
        self.m3.scale.z = 1.0
        self.m3.pose.position.x = 0.0
        self.m3.pose.position.y = 6.0
        self.m3.pose.position.z = 0.5
        self.m3.color.r = 1.0
        self.m3.color.g = 0.0
        self.m3.color.b = 0.0
        self.m3.color.a = 1.0
        self.pub.publish(self.m3)
        
        self.m4 = Marker()
        self.m4.header.frame_id = "odom"
        self.m4.header.stamp = self.get_clock().now().to_msg()
        self.m4.ns = "arena"
        self.m4.id = 4
        self.m4.type =  Marker.CUBE
        self.m4.action = Marker.ADD
        self.m4.scale.x = 11.0
        self.m4.scale.y = 1.0
        self.m4.scale.z = 1.0
        self.m4.pose.position.x = 0.0
        self.m4.pose.position.y = -6.0
        self.m4.pose.position.z = 0.5
        self.m4.color.r = 1.0
        self.m4.color.g = 0.0
        self.m4.color.b = 0.0
        self.m4.color.a = 1.0
        self.pub.publish(self.m4)
        
        
def get_distance(point1, point2):
        """ Calculates straight line distance between two poses
        
        point1 - initial point
        point2 - end point
        """
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
        
        
def main(args=None):
    
    """Entry point for the arenaNode."""
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()