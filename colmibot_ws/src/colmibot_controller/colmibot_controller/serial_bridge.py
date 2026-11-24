"""Envía los comandos de /cmd_vel a la ESP32 por USB."""
import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        puerto = self.declare_parameter('puerto', '/dev/ttyUSB0').value
        baudios = self.declare_parameter('baudios', 115200).value
        self.serie = serial.Serial(puerto, baudios, write_timeout=0.2)
        self.create_subscription(Twist, '/cmd_vel', self.enviar_velocidad, 1)
        self.get_logger().info(f'Puerto abierto: {puerto} a {baudios} baudios')

    def enviar_velocidad(self, msg):
        # Una línea por comando: velocidad lineal (m/s), angular (rad/s).
        trama = f'{msg.linear.x:.4f},{msg.angular.z:.4f}\n'
        self.serie.write(trama.encode('ascii'))


def main(args=None):
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = SerialBridge()
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except serial.SerialException as error:
        raise SystemExit(f'Error de comunicación serial: {error}')
    finally:
        if nodo is not None:
            try:
                nodo.serie.write(b'0.0000,0.0000\n')
            except serial.SerialException:
                pass  # El puerto puede haberse desconectado.
            nodo.serie.close()
            nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
