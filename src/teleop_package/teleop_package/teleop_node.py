import rclpy
from rclpy.node import Node
from interface_package.srv import MoveCommand  
import sys
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # Cliente para o serviço de movimento
        self.client = self.create_client(MoveCommand, 'move_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveCommand service...')

        self.get_logger().info('Connected to MoveCommand service.')
        self.run_teleop()

    def run_teleop(self):
        # Inicializar termios para capturar entrada do teclado
        self.get_logger().info('Use W/A/S/D para mover, Q/E para ajustar altura.')
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

        try:
            while True:
                char = sys.stdin.read(1)
                if char == 'w':
                    self.send_move_command(0.1, 0, 0)  # Move para frente
                elif char == 's':
                    self.send_move_command(-0.1, 0, 0)  # Move para trás
                elif char == 'a':
                    self.send_move_command(0, -0.1, 0)  # Move para esquerda
                elif char == 'd':
                    self.send_move_command(0, 0.1, 0)  # Move para direita
                elif char == 'q':
                    self.send_move_command(0, 0, 0.1)  # Sobe
                elif char == 'e':
                    self.send_move_command(0, 0, -0.1)  # Desce
                elif char == 'x':  
                    self.get_logger().info("Exiting teleop.")
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def send_move_command(self, x, y, z):
        # Enviar comando de movimento ao serviço
        req = MoveCommand.Request()
        req.x = float(x)  # Garantir que seja float
        req.y = float(y)  # Garantir que seja float
        req.z = float(z)  # Garantir que seja float

        self.client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
