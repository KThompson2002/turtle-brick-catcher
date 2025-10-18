from geometry_msgs.msg import TransformStamped
from interactive_markers.interactive_marker_server import (
    InteractiveMarker, InteractiveMarkerServer)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import InteractiveMarkerControl, Marker

# from turtle_brick_interfaces.srv import Place

class Arena(Node):
    """
    Simulates a turtle-sim sized arena using markers
    
    

    """
    
    def __init__(self):
        super().__init__('arena')
        
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Marker, 'arena_wall', markerQoS)
        
        self.m1 = Marker()
        self.m1.header.frame_id = "odom"
        self.m1.header.stamp = self.get_clock().now().to_msg()
        self.m1.ns = "arena"
        self.m1.id = 1
        self.m1.type =  Marker.CUBE
        self.m1.action = Marker.ADD
        self.m1.scale.x = 1.0
        self.m1.scale.y = 11.0
        self.m1.scale.z = 1.0
        self.m1.pose.position.x = -6.0
        self.m1.pose.position.y = 0.0
        self.m1.pose.position.z = 0.0
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
        self.m2.scale.y = 11.0
        self.m2.scale.z = 1.0
        self.m2.pose.position.x = 6.0
        self.m2.pose.position.y = 0.0
        self.m2.pose.position.z = 0.0
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
        self.m3.pose.position.z = 0.0
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
        self.m4.pose.position.z = 0.0
        self.m4.color.r = 1.0
        self.m4.color.g = 0.0
        self.m4.color.b = 0.0
        self.m4.color.a = 1.0
        self.pub.publish(self.m4)
        
        self.server = InteractiveMarkerServer(self, 'arena_marker')
        
        
        
        
        
def main(args=None):
    
    """Entry point for the arenaNode."""
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()