from abc import ABC, abstractmethod

class Mission(ABC):
    @abstractmethod
    def execute(self, control_node):
        pass
