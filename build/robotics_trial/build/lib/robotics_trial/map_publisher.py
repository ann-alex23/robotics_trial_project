import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

class map_publisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.map_publish=self.create_publisher(OccupancyGrid,'/map',10)
        self.create_timer=self.create_timer(2.0, self.publish_map)

    def publish_map(self):
        grid = [
            [0, 100, 0, 0, 0],
            [0, 100, 0, 100, 0],
            [0, 0, 0, 100, 0],
            [0, 100, 100, 100, 0],
            [0, 0, 0, 0, 0]
        ]
        oc_grid=OccupancyGrid()
        oc_grid.header.stamp = self.get_clock().now().to_msg() # get current time and converts to ROS2 message format
        oc_grid.header.frame_id = 'map'
        oc_grid.info.resolution = 1.0  # meters per cell
        oc_grid.info.width = len(grid[0])
        oc_grid.info.height = len(grid)
        oc_grid.info.origin=Pose()
        oc_grid.data = [cell for row in grid for cell in row]
        
        self.map_publish.publish(oc_grid)

        self.get_logger().info('Publishing: ')

def main(args=None):
    rclpy.init(args=args)
    node = map_publisher()
    rclpy.spin(node) #Keeps the node running, waiting for the timer to trigger
    # node.destroy_node()
    rclpy.shutdown() # Shutdown ROS2 when node stops

if __name__ == '__main__':
    main()
