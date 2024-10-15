import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from interface_package.srv import MoveCommand
from interface_package.action import Takeoff, Land
import sys
import pygame
import termios
import tty
import select

class TeleopNode(Node):
    def __init__(self, use_joystick=False):
        super().__init__('teleop_node')

        self.use_joystick = use_joystick
        self.active_command = (0.0, 0.0, 0.0)  # (x, y, z)

        if self.use_joystick:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() < 1:
                self.get_logger().error("Nenhum controle PS4 conectado. Encerrando.")
                sys.exit(1)
            else:
                self.controller = pygame.joystick.Joystick(0)
                self.controller.init()
                self.get_logger().info("Controle PS4 conectado.")
        else:
            self.controller = None

        self.move_command_client = self.create_client(MoveCommand, 'move_command')
        while not self.move_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando pelo serviço MoveCommand...')

        self.takeoff_action_client = ActionClient(self, Takeoff, 'takeoff')
        self.get_logger().info('Esperando pelo servidor Takeoff...')
        self.takeoff_action_client.wait_for_server()

        self.land_action_client = ActionClient(self, Land, 'land')
        self.get_logger().info('Esperando pelo servidor Land...')
        self.land_action_client.wait_for_server()

        self.get_logger().info('Conectado aos serviços e ações.')
        self.takeoff(1.0)  

        self.run_teleop()

    def takeoff(self, altitude):
        self.get_logger().info(f'Enviando comando de decolagem para {altitude} metros.')
        goal_msg = Takeoff.Goal()
        goal_msg.altitude = altitude
        self.takeoff_action_client.send_goal_async(goal_msg).add_done_callback(self.takeoff_response)

    def takeoff_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Comando de decolagem rejeitado.')
            return
        goal_handle.get_result_async().add_done_callback(self.takeoff_result)

    def takeoff_result(self, future):
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info('Decolagem bem-sucedida.')
            else:
                self.get_logger().info('Falha na decolagem.')
        except Exception as e:
            self.get_logger().error(f'Erro na ação de decolagem: {e}')

    def land(self):
        self.get_logger().info('Iniciando pouso...')
        goal_msg = Land.Goal()
        self.land_action_client.send_goal_async(goal_msg).add_done_callback(self.land_response)

    def land_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Comando de pouso rejeitado.')
            return
        goal_handle.get_result_async().add_done_callback(self.land_result)

    def land_result(self, future):
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info('Pouso bem-sucedido.')
                sys.exit(0) 
            else:
                self.get_logger().info('Falha no pouso.')
        except Exception as e:
            self.get_logger().error(f'Erro na ação de pouso: {e}')

    def run_teleop(self):
        if self.use_joystick:
            self.get_logger().info('Use o controle PS4. Pressione PS para sair.')
        else:
            self.get_logger().info('Use o teclado. Pressione X para sair.')

        if not self.use_joystick:
            settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        try:
            while rclpy.ok():
                if self.use_joystick:
                    self.process_ps4_events()
                else:
                    if self.is_keyboard_input():
                        char = sys.stdin.read(1)
                        self.process_keyboard_input(char)

                self.send_move_command(*self.active_command)
                rclpy.spin_once(self, timeout_sec=0)

        except KeyboardInterrupt:
            self.get_logger().info("Interrompido pelo usuário, iniciando pouso...")
            self.land()
        finally:
            if not self.use_joystick:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            if self.controller:
                pygame.quit()

    def process_ps4_events(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.handle_ps4_axes(event.axis, event.value)
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 10: 
                    self.get_logger().info("PS pressionado, iniciando pouso...")
                    self.land()

    def handle_ps4_axes(self, axis, value):
        threshold = 0.1
        value = round(value, 3)
        if abs(value) < threshold:
            value = 0.0

        if axis == 4:  # Right Stick X (Roll)
            self.active_command = (-value * 0.01, self.active_command[1], self.active_command[2])
        # elif axis == 0:  # Left Stick X (Yaw):
        #     self.active_command = ()  #YAW ainda não implementado
        elif axis == 3:  # Right Stick Y (Pitch)
            self.active_command = (self.active_command[0], -value * 0.01, self.active_command[2])
        elif axis == 1:  # left Stick Y (Altitude)
            self.active_command = (self.active_command[0], self.active_command[1], -value * 0.01)

    def process_keyboard_input(self, char):
        command_map = {
            'w': (0.2, 0.0, 0.0),
            's': (-0.2, 0.0, 0.0),
            'a': (0.0, -0.2, 0.0),
            'd': (0.0, 0.2, 0.0),
            'q': (0.0, 0.0, 0.1),
            'e': (0.0, 0.0, -0.1),
        }

        if char in command_map:
            self.active_command = command_map[char]
            self.send_move_command(*self.active_command)
            self.active_command = (0.0, 0.0, 0.0) 
        elif char == 'x':
            self.get_logger().info("X pressionado, iniciando pouso...")
            self.land()

    def is_keyboard_input(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def send_move_command(self, x, y, z):
        req = MoveCommand.Request()
        req.x = x
        req.y = y
        req.z = z

        future = self.move_command_client.call_async(req)
        future.add_done_callback(self.move_command_response)

    def move_command_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'MoveCommand response: {response}')
        except Exception as e:
            self.get_logger().error(f'MoveCommand call failed: {e}')


def main(args=None):
    rclpy.init(args=args)

    use_joystick = '--joystick' in sys.argv
    node = TeleopNode(use_joystick=use_joystick)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()