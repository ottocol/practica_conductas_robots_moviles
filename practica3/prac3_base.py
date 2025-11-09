#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

# YASMIN (https://github.com/uleroboticsgroup/yasmin)
from yasmin.state import State
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard

TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = "/scan"            
TOPIC_COLOR = "/color_detected"

ANG_IZQ = 30.0 * math.pi / 180.0
ANG_DER = -ANG_IZQ


class WanderAndDetect(State):
    """
    Estado único: se suscribe a láser y color, publica cmd_vel para deambular
    y termina cuando llega un Int32 en /color_detected (outcome 'color_detected').
    """
    def __init__(self, node):
        super().__init__(outcomes={'color_detected'})
        # runtime
        self._node =  node
        self._pub_cmd =  None
        self._sub_scan = None
        self._sub_color = None
        self._color_detected = False

    def execute(self, blackboard: Blackboard) -> str:

        # Pub/Sub
        self._pub_cmd = self._node.create_publisher(Twist, TOPIC_VEL, 10)
        self._sub_scan = self._node.create_subscription(
            LaserScan, TOPIC_SCAN, self._laser_cb, 10
        )
        self._sub_color = self._node.create_subscription(
            Int32, TOPIC_COLOR, self._color_cb, 10
        )

        # Bucle: procesamos callbacks hasta detectar el color
        while rclpy.ok() and not self._color_detected:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        # Limpieza explícita (buen hábito dentro de estados)
        if self._sub_scan:
            self._node.destroy_subscription(self._sub_scan)
            self._sub_scan = None
        if self._sub_color:
            self._node.destroy_subscription(self._sub_color)
            self._sub_color = None
        # Opcional: parar el robot al salir
        if self._pub_cmd:
            self._pub_cmd.publish(Twist())

        return 'color_detected'

    # --- Callbacks ---

    def _laser_cb(self, msg: LaserScan):
        pos_izq = self._angle_to_index(msg, ANG_IZQ)
        pos_der = self._angle_to_index(msg, ANG_DER)

        d_izq = msg.ranges[pos_izq] if pos_izq is not None else float('inf')
        d_der = msg.ranges[pos_der] if pos_der is not None else float('inf')

        # Control mínimo de ejemplo:
        # - avanza si hay espacio; si no, gira alejándose del obstáculo más cercano
        cmd = Twist()
        min_dist = min(d_izq, d_der)
        safe = 1.5            # umbral “seguro”
        fwd = 0.5            # m/s
        yaw = 0.6             # rad/s

        if min_dist > safe:
            cmd.linear.x = fwd
            cmd.angular.z = 0.0
        else:
            # si izquierda más cerca -> gira a la derecha; y viceversa
            cmd.linear.x = 0.05
            cmd.angular.z = -yaw if d_izq < d_der else yaw

        # Publica
        if self._pub_cmd:
            self._pub_cmd.publish(cmd)

    def _color_cb(self, msg: Int32):
        # Con que llegue un Int32, consideramos “detectado”
        self._color_detected = True

    # --- Utilidades ---

    @staticmethod
    def _angle_to_index(msg: LaserScan, ang: float) -> int | None:
        # Convierte un ángulo en índice dentro del LaserScan (con saturación)
        if msg.angle_increment == 0.0:
            return None
        idx = int(round((ang - msg.angle_min) / msg.angle_increment))
        if 0 <= idx < len(msg.ranges):
            return idx
        return None


def main(args=None):
    rclpy.init(args=args)
    node = Node("practica3")

    # FSM YASMIN
    sm = StateMachine(outcomes={'end'})
    sm.add_state('WanderAndDetect', WanderAndDetect(node),
                 transitions={'color_detected': 'end'})
    sm.set_start_state('WanderAndDetect')
    sm.validate()

    # Ejecuta FSM (bloquea hasta outcome final)
    outcome = sm.execute(Blackboard())
    node.get_logger().info(f"FSM terminada con outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
