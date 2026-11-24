from math import isfinite

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy


class JoyToTraction(Node):
    def __init__(self):
        super().__init__('joy2traction')
        self.eje_lineal = self.declare_parameter('eje_lineal', 1).value
        self.eje_angular = self.declare_parameter('eje_angular', 3).value
        self.r1 = self.declare_parameter('boton_r1', 5).value
        self.v_max = self.declare_parameter('escala_lineal', 1.0).value
        self.w_max = self.declare_parameter('escala_angular', 2.0).value
        self.zona_muerta = self.declare_parameter('zona_muerta', 0.08).value

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Joy, '/joy', self.leer_control, qos)

        # Se activa al recibir un mando con R1 presionado.
        self.paro = self.create_timer(
            0.30, self.detener, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.paro.cancel()

    def leer_control(self, msg):
        if not (0 <= self.r1 < len(msg.buttons) and
                0 <= self.eje_lineal < len(msg.axes) and
                0 <= self.eje_angular < len(msg.axes)):
            self.detener()
            return

        avance = msg.axes[self.eje_lineal]
        giro = msg.axes[self.eje_angular]
        if not msg.buttons[self.r1] or not isfinite(avance) or not isfinite(giro):
            self.detener()
            return

        cmd = Twist()
        if abs(avance) > self.zona_muerta:
            cmd.linear.x = avance * self.v_max
        if abs(giro) > self.zona_muerta:
            cmd.angular.z = giro * self.w_max
        self.publisher.publish(cmd)
        self.paro.reset()  # Cada mensaje habilitado reinicia los 0.30 s.

    def detener(self):
        if not self.paro.is_canceled():
            self.publisher.publish(Twist())
            self.paro.cancel()  # Un solo mensaje de paro.


def main(args=None):
    rclpy.init(args=args)
    nodo = JoyToTraction()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
