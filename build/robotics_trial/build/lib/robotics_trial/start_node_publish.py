import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class start_node_publisher(Node):
    def __init__(self):
        super().__init__('start_node_publisher')
        self.start_node_publish=self.create_publisher(Marker,'/start',10)
        self.create_timer=self.create_timer(2.0, self.publish_start_node)

    def publish_start_node(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Reference frame, could also be "world"
        marker.type = Marker.SPHERE  # We'll visualize the point as a sphere
        marker.action = Marker.ADD  # Add this marker
        marker.pose.position.x = 0.0  # X coordinate of the point
        marker.pose.position.y = 0.0  # Y coordinate of the point
        marker.pose.position.z = 0.0  # Z coordinate of the point (on the ground)
        
        # Default orientation
        marker.pose.orientation.w = 1.0  # No rotation
        
        # Set the scale of the marker (size in x, y, z directions)
        marker.scale.x = 0.3  # Make it small (0.1 units wide)
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # Set the color (RGBA)
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # No green
        marker.color.b = 0.0  # No blue
        marker.color.a = 1.0

        self.start_node_publish.publish(marker)
        self.get_logger().info(f'Publishing start point: ({marker.pose.position.x}, {marker.pose.position.y})')

def main(args=None):
    rclpy.init(args=args) # Initializes ROS2 client library
    node = start_node_publisher()
    rclpy.spin(node) #Keeps the node running, waiting for the timer to trigger
    node.destroy_node()
    rclpy.shutdown() # Shutdown ROS2 when node stops

if __name__ == '__main__':
    main()
