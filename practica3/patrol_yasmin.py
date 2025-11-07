#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node



import yasmin
from yasmin.state import State
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard
from yasmin_viewer import YasminViewerPub



def main(args=None):
    rclpy.init(args=args)
    node = Node("patrol_yasmin")

    # FSM YASMIN
    sm = StateMachine(outcomes={'end'})
    sm.validate()

    # Ejecuta FSM (bloquea hasta outcome final)
    outcome = sm.execute(Blackboard())
    node.get_logger().info(f"FSM terminada con outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
