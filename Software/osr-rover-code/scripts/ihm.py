#Adicionar o código da outra rasp aqui
import psutil  # Para obter informações do sistema
import subprocess  # Para executar comandos do sistema
from RPi_GPIO_i2c_LCD import lcd
from time import sleep, strftime

def get_battery_percentage():
    # Obtém o percentual da bateria
    battery = psutil.sensors_battery()
    return battery.percent if battery else None

#def get_network_name():
#    # Obtém o nome da rede
#    try:
#        output = subprocess.check_output(["iwgetid", "-r"]).decode("utf-8").strip()
#        return output if output else None
#    except subprocess.CalledProcessError:
#        return None

def update_display(display):
    # Atualiza o display com as informações do sistema
    battery_percentage = get_battery_percentage()
    #network_name = get_network_name()

    display.clear()
    display.set("Bateria: {}%".format(battery_percentage), 1)
    #display.set("Network: {}".format(network_name), 2)

def main():
    # Função principal
    lcdDisplay = lcd.HD44780(0x27)  # Inicializa o display

    # Exibe informações iniciais
    lcdDisplay.backlight("on")
    update_display(lcdDisplay)
    sleep(5)
    lcdDisplay.backlight("off")

    # Loop principal
    #while True:
    #    # Atualiza o display a cada 5 minutos
    #    update_display(lcdDisplay)
    #    lcdDisplay.backlight("off")
    #    sleep(300)  # 5 minutos em segundos

if __name__ == "__main__":
    main()




'''import psutil  # Para obter informações do sistema
import subprocess  # Para executar comandos do sistema
from RPi_GPIO_i2c_LCD import lcd
from time import sleep, strftime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BatteryListener(Node):
    def __init__(self):
        super().__init__('battery_listener')
        self.battery = None
        self.subscription = self.create_subscription(
            String,
            '/status',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.battery = msg.data

def get_battery_percentage():
    rclpy.init()
    battery_listener = BatteryListener()
    rclpy.spin(battery_listener)
    battery_percentage = battery_listener.battery
    rclpy.shutdown()

    return battery_percentage

def update_display(display):
    # Atualiza o display com as informações do sistema
    battery_percentage = get_battery_percentage()

    display.clear()
    display.set("Bateria: {}%".format(battery_percentage), 1)

def main():
    # Função principal
    lcdDisplay = lcd.HD44780(0x27)  # Inicializa o display

    # Exibe informações iniciais
    lcdDisplay.backlight("on")
    update_display(lcdDisplay)
    sleep(5)
    lcdDisplay.backlight("off")

if __name__ == "__main__":
    main()'''

