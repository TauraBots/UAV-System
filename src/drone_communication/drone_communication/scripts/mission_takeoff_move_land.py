from .mission import Mission

class TakeoffMoveLandMission(Mission):
    def execute(self, control_node):
        control_node.arm()
        control_node.engage_offboard_mode() 
        control_node.publish_position_setpoint(0.0, 0.0, -5.0)  # Takeoff
        control_node.get_logger().info("Taking off...")
        control_node.publish_position_setpoint(1.0, 0.0, -5.0)  # Move forward 1 meter
        control_node.get_logger().info("Moving forward...")
        control_node.land()  # Land
        control_node.get_logger().info("Landing...")
