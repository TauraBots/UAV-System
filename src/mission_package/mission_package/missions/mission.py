from abc import ABC, abstractmethod

class Mission(ABC):
    def __init__(self, node):
        self.node = node

    @abstractmethod
    def waypoints(self):
        """Deve retornar a lista de waypoints para a missão."""
        pass

    def execute(self):
        self.node.get_logger().info("Starting mission...")
        self.node.start_mission(self.waypoints())
