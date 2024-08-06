import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from evdev import InputDevice, ecodes

SERIAL_PORT = '/dev/input/event0'  # Substitua pelo dispositivo serial correto
CHECK_INTERVAL = 0.5  # Intervalo de checagem em segundos
TIMEOUT_INTERVAL = 2  # Tempo máximo sem eventos antes de considerar desconexão

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(CHECK_INTERVAL, self.timer_callback)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 12
        self.last_button_release_time = None  # Nova variável para rastrear o tempo de liberação do botão

        self.device = self.connect_device(SERIAL_PORT)

    def connect_device(self, serial_port):
        try:
            device = InputDevice(serial_port)
            self.get_logger().info(f"Conectado ao dispositivo: {serial_port}")
            return device
        except PermissionError as e:
            self.get_logger().error(f"Permissão negada para acessar {serial_port}: {e}")
            raise
        except FileNotFoundError as e:
            self.get_logger().error(f"Dispositivo não encontrado: {serial_port}: {e}")
            raise

    def timer_callback(self):
        current_time = time.time()
        if self.last_button_release_time:
            time_diff = current_time - self.last_button_release_time
            self.get_logger().info(f'Current time: {current_time}, Last button release time: {self.last_button_release_time}, Time difference: {time_diff}')
            if time_diff > TIMEOUT_INTERVAL:
                self.get_logger().info('Nenhum evento detectado em 5 segundos após soltar o botão, zerando eixos e botões...')
                self.reset_joy_msg()
                self.publisher_.publish(self.joy_msg)
                self.reboot_system()

    def reset_joy_msg(self):
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 12

    def reboot_system(self):
        self.get_logger().info('Reiniciando o sistema...')
        os.system('reboot')

    def run(self):
        while rclpy.ok():
            try:
                for event in self.device.read_loop():
                    if event.type == ecodes.EV_KEY and event.code == 304:
                        self.get_logger().info(f"Botão {event.code}: {event.value}")
                        if event.value == 0:  # Botão foi liberado
                            self.last_button_release_time = time.time()
                            self.get_logger().info(f"Botão 304 liberado: {self.last_button_release_time}")
                        else:  # Botão foi pressionado
                            self.last_button_release_time = None  # Reseta o tempo de liberação
                        self.publisher_.publish(self.joy_msg)
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Erro ao ler evento: {e}")

def main(args=None):
    rclpy.init(args=args)
    joy_publisher = JoyPublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(joy_publisher)
    try:
        joy_publisher.run()
    except KeyboardInterrupt:
        pass
    joy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




