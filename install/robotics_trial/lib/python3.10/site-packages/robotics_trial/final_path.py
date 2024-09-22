import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import heapq
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class ArgumentsSubscriber(Node):
    def __init__(self):
        super().__init__('arguments_subscriber')
        self.map_data = None
        self.start_point = None
        self.goal_point = None
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_subscriber_fn,
            10
        )
        self.start_subscriber = self.create_subscription(
            Marker,
            '/start',
            self.start_subscriber_fn,
            10
        )
        self.goal_subscriber = self.create_subscription(
            Marker,
            '/goal',
            self.goal_subscriber_fn,
            10
        )
        self.path_publisher = self.create_publisher(Path, '/path', 10)
        self.create_timer=self.create_timer(2.0, self.compute_path)

        

    def map_subscriber_fn(self, msg):
        self.map_data = msg
        self.get_logger().info(f'Received map data with size: {len(self.map_data.data)}')
        # self.compute_path()

    def start_subscriber_fn(self, msg):
        self.start_point = msg
        self.get_logger().info(f'Received start data: ({self.start_point.pose.position.x}, {self.start_point.pose.position.y})')
        # self.compute_path()

    def goal_subscriber_fn(self, msg):
        self.goal_point = msg
        self.get_logger().info(f'Received goal data: ({self.goal_point.pose.position.x}, {self.goal_point.pose.position.y})')
        # self.compute_path()

    def compute_path(self):
        if self.map_data is not None and self.start_point is not None and self.goal_point is not None:
            path = self.a_star(self.map_data, self.start_point.pose.position, self.goal_point.pose.position)
            if path:
                path_msg = Path()
                path_msg.header.stamp = self.get_clock().now().to_msg()
                path_msg.header.frame_id = 'map'
            
                for (x, y) in path:
                    pose = PoseStamped()
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = float(x)
                    pose.pose.position.y = float(y)
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                    path_msg.poses.append(pose)
            self.path_publisher.publish(path_msg)
            self.get_logger().info(f'Path published')

    def a_star(self, map_data, start, goal):
        # self.get_logger().info(f'entered astar')
        width = map_data.info.width
        height = map_data.info.height
        origin_x, origin_y = map_data.info.origin.position.x, map_data.info.origin.position.y
        resolution = map_data.info.resolution

        grid_2d = [map_data.data[i * width:(i + 1) * width] for i in range(height)]

        s_grid_x = int((start.x - origin_x) / resolution) # converting coordinates to grid indices
        s_grid_y = int((start.y - origin_y) / resolution)
        g_grid_x = int((goal.x - origin_x) / resolution)
        g_grid_y = int((goal.y - origin_x) / resolution)

        if self.if_obstacle(s_grid_x, s_grid_y, grid_2d) or not self.within_bounds(s_grid_x, s_grid_y, width, height):
            self.get_logger().error('Start point is an obstacle or out of bounds.')
            return []

        if self.if_obstacle(g_grid_x, g_grid_y, grid_2d) or not self.within_bounds(g_grid_x, g_grid_y, width, height):
            self.get_logger().error('Goal point is an obstacle or out of bounds.')
            return []
        

        open_list=[]
        closed_list=set()
        heapq.heapify(open_list)

        g_cost={(s_grid_x,s_grid_y):0}
        f_cost={(g_grid_x, g_grid_y):self.euclidean_distance(s_grid_x, s_grid_y,g_grid_x, g_grid_y)}
        f_cost[(s_grid_x,s_grid_y)]=self.euclidean_distance(s_grid_x, s_grid_y,g_grid_x, g_grid_y)
        parent={(s_grid_x,s_grid_y):None}
        heapq.heappush(open_list,(f_cost[(s_grid_x,s_grid_y)],(s_grid_x,s_grid_y)))

        while open_list:
            # self.get_logger().info('Probs')
            _,(current_x,current_y)=heapq.heappop(open_list)
            # self.get_logger().info(f'{_}')
            if (current_x,current_y)==(g_grid_x, g_grid_y):
                return self.construct_path(parent,(g_grid_x, g_grid_y))
            closed_list.add((current_x, current_y))
            neighbors_list=self.find_neighbors(current_x,current_y,grid_2d)
            for x,y in neighbors_list:
                if (x,y) not in closed_list:
                    temp_g_cost=self.euclidean_distance(current_x, current_y,x,y) + g_cost[(current_x, current_y)]
                    if (x,y) not in g_cost or temp_g_cost<=g_cost[(x,y)]:
                        g_cost[(x,y)]=temp_g_cost
                        f_cost[(x,y)]=g_cost[(x,y)]+self.euclidean_distance(g_grid_x,g_grid_y,x,y)
                        parent[(x,y)]=(current_x,current_y)
                        heapq.heappush(open_list,(f_cost[(x,y)],(x,y)))

    
    def construct_path(self,parent,goal):
        final_path=[]
        current=goal
        while current is not None:
            final_path.append(current)
            current=parent[current]
        self.get_logger().info(f'{final_path}')
        final_path.reverse()
        return final_path

    def if_obstacle(self, x, y, grid_list):
        return grid_list[y][x] != 0

    def within_bounds(self, x, y, width, height):
        return 0 <= x < width and 0 <= y < height

    def find_neighbors(self, x, y, grid_list):
        neighbors_pos = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        result = []
        width = len(grid_list[0])
        height = len(grid_list)
        
        for i, j in neighbors_pos:
            new_x, new_y = x + i, y + j
            if self.within_bounds(new_x, new_y, width, height) and not self.if_obstacle(new_x, new_y, grid_list):
                result.append((new_x, new_y))
        
        return result

    def euclidean_distance(self, x1, y1, x2, y2):
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

def main(args=None):
    rclpy.init(args=args)
    node = ArgumentsSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
