import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from interface_package.action import Takeoff, Land, NavigateWaypoints
from mission_package.missions import Task1

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.takeoff_action_client = ActionClient(self, Takeoff, 'takeoff')
        self.land_action_client = ActionClient(self, Land, 'land')
        self.navigate_action_client = ActionClient(self, NavigateWaypoints, 'navigate_waypoints')
        self.get_logger().info('Mission node is ready.')
        self.current_mission = Task1(self) 
        self.current_mission.execute()

    def start_mission(self, waypoints):
        # Step 1: Decolagem
        self.get_logger().info("Initiating takeoff...")
        takeoff_goal = Takeoff.Goal()
        takeoff_goal.altitude = 2.0 
        self.send_takeoff_goal(takeoff_goal, waypoints)

    def send_takeoff_goal(self, goal, waypoints):
        self.takeoff_action_client.wait_for_server()
        self._takeoff_future = self.takeoff_action_client.send_goal_async(goal)
        self._takeoff_future.add_done_callback(
            lambda future: self.takeoff_response_callback(future, waypoints))

    def takeoff_response_callback(self, future, waypoints):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Takeoff rejected.")
            return

        self.get_logger().info("Takeoff accepted.")
        self._takeoff_result_future = goal_handle.get_result_async()
        self._takeoff_result_future.add_done_callback(
            lambda future: self.takeoff_result_callback(future, waypoints))

    def takeoff_result_callback(self, future, waypoints):
        result = future.result().result
        if result.success:
            self.get_logger().info("Takeoff succeeded. Proceeding to navigate waypoints...")
            self.navigate_waypoints(waypoints)
        else:
            self.get_logger().info("Takeoff failed.")

    def navigate_waypoints(self, waypoints):
        # Enviando goal para NavigateWaypoints
        navigate_goal = NavigateWaypoints.Goal()
        navigate_goal.waypoints = waypoints

        self.get_logger().info("Navigating through waypoints...")
        self.send_navigate_goal(navigate_goal)

    def send_navigate_goal(self, goal):
        self.navigate_action_client.wait_for_server()
        self._navigate_future = self.navigate_action_client.send_goal_async(goal)
        self._navigate_future.add_done_callback(self.navigate_response_callback)

    def navigate_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Navigate waypoints rejected.")
            return

        self.get_logger().info("Navigate waypoints accepted.")
        self._navigate_result_future = goal_handle.get_result_async()
        self._navigate_result_future.add_done_callback(self.navigate_result_callback)

    def navigate_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Waypoints navigation succeeded. Proceeding to land...")
            self.initiate_landing()
        else:
            self.get_logger().info("Waypoints navigation failed.")

    def initiate_landing(self):
        self.get_logger().info("Initiating landing...")
        land_goal = Land.Goal()
        self.send_land_goal(land_goal)

    def send_land_goal(self, goal):
        self.land_action_client.wait_for_server()
        self._land_future = self.land_action_client.send_goal_async(goal)
        self._land_future.add_done_callback(self.land_response_callback)

    def land_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Land rejected.")
            return

        self.get_logger().info("Land accepted.")
        self._land_result_future = goal_handle.get_result_async()
        self._land_result_future.add_done_callback(self.land_result_callback)

    def land_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Landing succeeded.")
        else:
            self.get_logger().info("Landing failed.")


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
