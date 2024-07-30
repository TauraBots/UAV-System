#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import (
    CommandBool,
    CommandTOL,
    SetMode,
    StreamRate
)
from std_msgs.msg import Empty
from mavros_msgs.msg import State
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped

class BaseController(Node):
    """BaseController is used to call the services from mavlink using mavros.
    
    Keywords arguments:
    services_timeout -- the max time to check if the services are running (default=60 seconds).
    """

    @staticmethod
    def global_from_local_position(local_position: list, pose: PoseStamped) -> list:
        """Get the drone position with the pose to navigate
        
        Keywords arguments:
        local_position  -- the current location of the drone
        pose            -- the pose of the navigation
        """
        global_position = [
            local_position[0] + pose.pose.position.x,
            local_position[1] + pose.pose.position.y,
            local_position[2] + pose.pose.position.z
        ]
        return global_position

    def __init__(self, services_timeout: float = 60):
        super().__init__('base_controller')
        self.services_timeout = services_timeout
        self.state = None
        self.rangefinder = None
        self.current_position = None

        self.__read_parameters()
        self.__init_services()
        self.__init_publishers()
        self.__init_subscribers()

        self.__check_for_services(self.services_timeout)
        assert self.__enable_mavros_topics()

    def __read_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('services.arming', '/mavros/cmd/arming'),
                ('services.takeoff', '/mavros/cmd/takeoff'),
                ('services.setmode', '/mavros/set_mode'),
                ('services.land', '/mavros/cmd/land'),
                ('services.stream_rate', '/mavros/set_stream_rate'),
                ('publishers.setpoint_position_local', '/mavros/setpoint_position/local'),
                ('subscribers.state', '/mavros/state'),
                ('subscribers.rangefinder', '/mavros/rangefinder/rangefinder'),
                ('subscribers.local_position_pose', '/mavros/local_position/pose'),
                ('custom_modes', ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER', 'RTL', 'CIRCLE', 'POSITION', 'LAND', 'OF_LOITER', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE', 'POSHOLD', 'BRAKE', 'THROW', 'AVOID_ADSB', 'GUIDED_NOGPS'])
            ]
        )

        self.__service_arming = self.get_parameter('services.arming').get_parameter_value().string_value
        self.__service_takeoff = self.get_parameter('services.takeoff').get_parameter_value().string_value
        self.__service_setmode = self.get_parameter('services.setmode').get_parameter_value().string_value
        self.__service_land = self.get_parameter('services.land').get_parameter_value().string_value
        self.__service_streamrate = self.get_parameter('services.stream_rate').get_parameter_value().string_value
        self.__setpoint_local = self.get_parameter('publishers.setpoint_position_local').get_parameter_value().string_value
        self.__state = self.get_parameter('subscribers.state').get_parameter_value().string_value
        self.__rangefinder = self.get_parameter('subscribers.rangefinder').get_parameter_value().string_value
        self.__local_position_pose = self.get_parameter('subscribers.local_position_pose').get_parameter_value().string_value
        self.__custom_modes = self.get_parameter('custom_modes').get_parameter_value().string_array_value

    def __init_services(self):
        self.__service_arming_proxy = self.create_client(CommandBool, self.__service_arming)
        self.__service_takeoff_proxy = self.create_client(CommandTOL, self.__service_takeoff)
        self.__service_setmode_proxy = self.create_client(SetMode, self.__service_setmode)
        self.__service_land_proxy = self.create_client(CommandTOL, self.__service_land)
        self.__service_streamrate_proxy = self.create_client(StreamRate, self.__service_streamrate)

    def __init_publishers(self):
        self.__publisher_setpoint_local = self.create_publisher(PoseStamped, self.__setpoint_local, 10)

    def __init_subscribers(self):
        self.create_subscription(State, self.__state, self.__state_callback, 10)
        self.create_subscription(Range, self.__rangefinder, self.__rangefinder_callback, 10)
        self.create_subscription(PoseStamped, self.__local_position_pose, self.__local_position_pose_callback, 10)

    def __state_callback(self, msg):
        self.state = msg

    def __rangefinder_callback(self, msg):
        self.rangefinder = msg

    def __local_position_pose_callback(self, msg):
        self.current_position = msg

    def __check_for_services(self, services_timeout):
        for client in [self.__service_arming_proxy, self.__service_takeoff_proxy, self.__service_setmode_proxy, self.__service_land_proxy, self.__service_streamrate_proxy]:
            if not client.wait_for_service(timeout_sec=services_timeout):
                self.get_logger().error(f'Service {client.srv_name} not available')
                raise Exception(f'Service {client.srv_name} not available')

    def __enable_mavros_topics(self):
        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = 10
        req.on_off = True
        future = self.__service_streamrate_proxy.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if hasattr(future.result(), 'success'):
                return future.result().success
            else:
                self.get_logger().error('Response does not have success attribute')
                return False
        else:
            self.get_logger().error('No response received')
            return False

    def arm(self):
        future = self.__service_arming_proxy.call_async(CommandBool.Request(value=True))
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def disarm(self):
        future = self.__service_arming_proxy.call_async(CommandBool.Request(value=False))
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def takeoff(self, min_pitch=0.0, yaw=0.0, latitude=0.0, longitude=0.0, altitude=0.0):
        self.set_custom_mode("LOITER")
        time.sleep(2)
        future = self.__service_takeoff_proxy.call_async(CommandTOL.Request(min_pitch=min_pitch, yaw=yaw, latitude=latitude, longitude=longitude, altitude=altitude))
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def set_custom_mode(self, custom_mode):
        assert custom_mode in self.__custom_modes
        future = self.__service_setmode_proxy.call_async(SetMode.Request(custom_mode=custom_mode))
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent

    def land(self, min_pitch=0.0, yaw=0.0, latitude=0.0, longitude=0.0, altitude=0.0):
        future = self.__service_land_proxy.call_async(CommandTOL.Request(min_pitch=min_pitch, yaw=yaw, latitude=latitude, longitude=longitude, altitude=altitude))
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def set_position(self, position_x=0.0, position_y=0.0, position_z=0.0):
        self.set_custom_mode("GUIDED")
        pose = PoseStamped()
        pose.pose.position.x = position_x
        pose.pose.position.y = position_y
        pose.pose.position.z = position_z
        self.__publisher_setpoint_local.publish(pose)
        return BaseController.global_from_local_position(self.get_current_position(), pose)

    def get_current_position(self):
        return [self.current_position.pose.position.x, self.current_position.pose.position.y, self.current_position.pose.position.z]

    def get_height(self):
        return self.rangefinder.range

def main(args=None):
    rclpy.init(args=args)
    base_controller = BaseController()
    rclpy.spin(base_controller)
    base_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
