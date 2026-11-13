#!/usr/bin/env python3
"""
control_torque.py
=================
Nodo ROS 2 de control de TORQUE para Gazebo (xacro modo:=gazebo_torque).

    /joint_states ──► PID por articulación + G(q) ──► /effort_controller/commands
                      (nucleo_control.ControlTorque)   [riel N, base, hombro, codo N·m]

El lazo se cierra con cada mensaje de /joint_states (250 Hz, tiempo de
simulación tomado del sello del mensaje).

Tópicos:
  sub  /trayectoria/comando   std_msgs/String   home | recta | semiarco | paro |
                                                escalon <i> [amp] | rampa <i> [pend]
  pub  /trayectoria/estado    std_msgs/String   JSON: tipo, fase, s, p, v, error, q, tau...
  pub  /efector/posicion      geometry_msgs/PointStamped   (marco de tarea {0}, m)
  pub  /efector/velocidad     geometry_msgs/TwistStamped   (solo parte lineal, m/s)

Parámetros:
  compensar_gravedad (bool, true)   suma G(q) al PID
  modelar_actuador   (bool, true)   limita el par con el modelo de motor de CD
  incluir_rotor      (bool, false)  ganancias con la inercia de rotor reflejada
"""

import json

import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from modelo_brazo import JOINTS
from nucleo_control import ControlTorque

CADA_ESTADO = 10      # publica el estado cada 10 ciclos (25 Hz)


class NodoControlTorque(Node):
    def __init__(self):
        super().__init__('control_torque')
        self.declare_parameter('compensar_gravedad', True)
        self.declare_parameter('modelar_actuador', True)
        self.declare_parameter('incluir_rotor', False)

        self.ctrl = ControlTorque(
            compensar_gravedad=self.get_parameter('compensar_gravedad').value,
            modelar_actuador=self.get_parameter('modelar_actuador').value,
            incluir_rotor=self.get_parameter('incluir_rotor').value,
        )
        for d in self.ctrl.diseno:
            self.get_logger().info(
                f"{d['etiqueta']:<10} J={d['J']:.4f}  Kp={d['Kp']:.3f}  Ki={d['Ki']:.3f}  "
                f"Kd={d['Kd']:.3f}  b={d['b']:.3f}  límite=±{d['u_max']:.2f}")

        self.pub_tau = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.pub_estado = self.create_publisher(String, '/trayectoria/estado', 10)
        self.pub_pos = self.create_publisher(PointStamped, '/efector/posicion', 10)
        self.pub_vel = self.create_publisher(TwistStamped, '/efector/velocidad', 10)
        self.create_subscription(String, '/trayectoria/comando', self._comando_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._estado_cb, 10)

        self._t_prev = None
        self._q_prev = None
        self._ciclo = 0
        self.get_logger().info('control_torque listo: esperando /joint_states ...')

    def _comando_cb(self, msg):
        self.get_logger().info(f'Comando recibido: "{msg.data}"')
        self.ctrl.comando(msg.data)

    def _estado_cb(self, msg):
        idx = {n: k for k, n in enumerate(msg.name)}
        if any(j not in idx for j in JOINTS):
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = [msg.position[idx[j]] for j in JOINTS]

        if self._t_prev is not None and t < self._t_prev:
            # La simulación se reinició: arrancar de nuevo sin saltos
            self.get_logger().warn('El tiempo retrocedió; reiniciando el controlador.')
            self.ctrl.t = None
            self._q_prev = None
        if len(msg.velocity) == len(msg.name):
            dq = [msg.velocity[idx[j]] for j in JOINTS]
        elif self._q_prev is not None and t > self._t_prev:
            dq = [(a - b) / (t - self._t_prev) for a, b in zip(q, self._q_prev)]
        else:
            dq = [0.0] * 4
        self._t_prev, self._q_prev = t, q

        tau = self.ctrl.paso(t, q, dq)
        self.pub_tau.publish(Float64MultiArray(data=[float(v) for v in tau]))

        self._ciclo += 1
        if self._ciclo % CADA_ESTADO == 0:
            e = self.ctrl.estado()
            self.pub_estado.publish(String(data=json.dumps(e)))
            p = PointStamped()
            p.header.stamp = msg.header.stamp
            p.header.frame_id = 'marco_tarea'
            p.point.x, p.point.y, p.point.z = e['p']
            self.pub_pos.publish(p)
            v = TwistStamped()
            v.header = p.header
            v.twist.linear.x, v.twist.linear.y, v.twist.linear.z = e['v']
            self.pub_vel.publish(v)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoControlTorque()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
