import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from interface_package.srv import MoveCommand
from interface_package.action import Takeoff, Land
import sys
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        # Cliente para o serviço de movimento
        self.move_command_client = self.create_client(MoveCommand, 'move_command')
        while not self.move_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveCommand service...')

        # Cliente para a action de Takeoff
        self.takeoff_action_client = ActionClient(self, Takeoff, 'takeoff')
        self.get_logger().info('Waiting for Takeoff action server...')
        self.takeoff_action_client.wait_for_server()

        # Cliente para a action de Land
        self.land_action_client = ActionClient(self, Land, 'land')
        self.get_logger().info('Waiting for Land action server...')
        self.land_action_client.wait_for_server()

        self.get_logger().info('Connected to MoveCommand service, Takeoff, and Land actions.')

        # Executa o Takeoff no início
        self.takeoff(1.0)

        # Iniciar o teleop para controle manual
        self.run_teleop()

    def takeoff(self, altitude):
        self.get_logger().info(f'Sending Takeoff request with altitude: {altitude}')
        goal_msg = Takeoff.Goal()
        goal_msg.altitude = altitude

        self.takeoff_action_client.send_goal_async(goal_msg).add_done_callback(self.takeoff_response)

    def takeoff_response(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Takeoff successful.')
        else:
            self.get_logger().info('Takeoff failed.')

    def land(self):
        self.get_logger().info('Sending Land request...')
        goal_msg = Land.Goal()

        self.land_action_client.send_goal_async(goal_msg).add_done_callback(self.land_response)

    def land_response(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Landing successful.')
        else:
            self.get_logger().info('Landing failed.')

    def run_teleop(self):
        # Inicializar termios para capturar entrada do teclado
        self.get_logger().info('Use W/A/S/D para mover, Q/E para ajustar altura. Pressione X para sair e realizar land.')
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
                    self.get_logger().info("Exiting teleop and initiating landing...")
                    self.land()  # Executa land ao sair
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def send_move_command(self, x, y, z):
        # Enviar comando de movimento ao serviço MoveCommand
        req = MoveCommand.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)

        self.move_command_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
