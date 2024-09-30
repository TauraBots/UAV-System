import math
from geometry_msgs.msg import PoseStamped
from .mission import Mission

class Task3(Mission):
    def __init__(self, node, radius=2.0, num_points=36):
        super().__init__(node)
        self.radius = radius  # Raio do círculo
        self.num_points = num_points  # Número de pontos que formam o círculo

    def waypoints(self):
        waypoints = []
        for i in range(self.num_points):
            angle = (2 * math.pi / self.num_points) * i  # Angulo em radianos
            wp = PoseStamped()
            wp.pose.position.x = self.radius * math.cos(angle)  # Cálculo da coordenada x
            wp.pose.position.y = self.radius * math.sin(angle)  # Cálculo da coordenada y
            wp.pose.position.z = 2.0  # Altura de 2 metros
            waypoints.append(wp)
        
        return waypoints
