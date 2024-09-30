from geometry_msgs.msg import PoseStamped
from .mission import Mission

class Task2(Mission):
    def waypoints(self):
        # Definindo os vértices de um quadrado de lado 2
        wp1 = PoseStamped()
        wp1.pose.position.x = 0.0
        wp1.pose.position.y = 0.0
        wp1.pose.position.z = 2.0  # Altura de 2 metros

        wp2 = PoseStamped()
        wp2.pose.position.x = 2.0
        wp2.pose.position.y = 0.0
        wp2.pose.position.z = 2.0

        wp3 = PoseStamped()
        wp3.pose.position.x = 2.0
        wp3.pose.position.y = 2.0
        wp3.pose.position.z = 2.0

        wp4 = PoseStamped()
        wp4.pose.position.x = 0.0
        wp4.pose.position.y = 2.0
        wp4.pose.position.z = 2.0

        return [wp1, wp2, wp3, wp4]
