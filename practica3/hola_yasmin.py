#!/usr/bin/env python3
import time

import rclpy


import yasmin
from yasmin.state import State
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard
from yasmin_viewer import YasminViewerPub

import logging



class EstadoA(State):
    def __init__(self):
        super().__init__(outcomes={'to_b', 'finished'})
        self._count = 0  # nº de transiciones A→B realizadas

    def execute(self, blackboard: Blackboard) -> str:
        print(f"→ A (iter {self._count + 1})")
        time.sleep(2.0)  # trabajo simulado 2 s
        self._count += 1
        blackboard.mensaje = f"Hola desde A: contador:{self._count}"
        if self._count >= 3:
            print("✅ Fin: alcanzados 3 cambios A→B")
            return 'finished'
        print("  A → B")
        return 'to_b'

class EstadoB(State):
    def __init__(self):
        super().__init__(outcomes={'to_a'})

    def execute(self, blackboard: Blackboard) -> str:
        print("→ B")
        time.sleep(2.0)
        print("  B → A")
        print(f"Blackboard.mensaje={blackboard.mensaje}")
        return 'to_a'

def main():
    import logging

    #deshabilita mensajes de YASMIN (pero también de ROS o quien sea)
    #logging.disable(logging.INFO)

    # Inicializar ROS 2
    rclpy.init()

    sm = StateMachine(outcomes={'finished'})
    a = EstadoA()
    b = EstadoB()

    # Registrar estados y transiciones basadas en outcomes
    sm.add_state('A', a, transitions={'to_b': 'B', 'finished': 'finished'})
    sm.add_state('B', b, transitions={'to_a': 'A'})
    sm.set_start_state('A')
    sm.validate()  # opcional pero recomendable

    YasminViewerPub("HOLA YASMIN", sm)
    outcome = sm.execute(Blackboard())
    print(f"FSM terminada con outcome: {outcome}")

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
