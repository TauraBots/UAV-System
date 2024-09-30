from geometry_msgs.msg import PoseStamped
from .mission import Mission

class Task4(Mission):
    def waypoints(self):
        waypoints = []
        points = [
            (2.0, 0.0, 2.0),  # Vértice 1
            (0.0, 2.0, 2.0),  # Vértice 2
            (-2.0, 0.0, 2.0), # Vértice 3
        ]

        for point in points:
            wp = PoseStamped()
            wp.pose.position.x = point[0]
            wp.pose.position.y = point[1]
            wp.pose.position.z = point[2]
            waypoints.append(wp)

        return waypoints
