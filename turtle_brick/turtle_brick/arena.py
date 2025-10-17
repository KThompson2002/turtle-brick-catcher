from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import InteractiveMarkerControl, Marker

class Arena(Node):
    """
    Simulates a turtle-sim sized arena using markers
    
    

    """
    
    def __init__(self):
        super().__init__('arena')
        
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Marker, 'visualization_marker', markerQoS)
        
        self.m = Marker()
        self.m.header.frame_id = "arena"
        self.m.header.stamp = self.get_clock().now().to_msg()
        # self.m.id = 1
        # self.m.type =  
        # self.m.action = 
        
        
def main(args=None):
    """Entry point for the arenaNode."""
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()