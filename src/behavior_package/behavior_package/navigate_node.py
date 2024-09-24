import rclpy
from rclpy.node import Node
from interface_package.srv import MoveCommand  # Reutilizando o serviço MoveCommand

class NavigateNode(Node):

    def __init__(self):
        super().__init__('navigate_node')

        # Cliente para o serviço MoveCommand do interface_package
        self.client = self.create_client(MoveCommand, 'move_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveCommand service...')

        self.get_logger().info('Connected to MoveCommand service.')

        
        self.waypoints = [
            (1.0, 1.0, 2.0),
            (2.0, 0.0, 2.0),
            (3.0, 1.0, 2.0)  
        ]
        self.current_waypoint_index = 0
        self.navigate_to_next_waypoint()

    def navigate_to_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            self.get_logger().info(f'Navigating to waypoint: {waypoint}')
            self.send_move_command(waypoint[0], waypoint[1], waypoint[2])
            self.current_waypoint_index += 1
        else:
            self.get_logger().info('Reached all waypoints!')

    def send_move_command(self, x, y, z):
        req = MoveCommand.Request()
        req.x = float(x)  
        req.y = float(y)
        req.z = float(z)

        future = self.client.call_async(req)
        future.add_done_callback(self.waypoint_reached)

    def waypoint_reached(self, future):
        result = future.result()
        if result.success:
            self.get_logger().info('Waypoint reached successfully!')
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().error('Failed to reach waypoint.')

def main(args=None):
    rclpy.init(args=args)

    navigate_node = NavigateNode()

    rclpy.spin(navigate_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
