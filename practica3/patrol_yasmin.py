#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose

import yasmin
from yasmin.state import State
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard
from yasmin_viewer import YasminViewerPub
from yasmin_ros import ActionState


class GotoWPState(ActionState):
    def __init__(self, wp):
        self.wp = wp
        self.gh = None
        super().__init__(
            NavigateToPose,
            "/navigate_to_pose",
            create_goal_handler=self.create_goal_handler,
            feedback_handler=self.feedback_handler
        )

    def create_goal_handler(self, blackboard):
        pose = Pose()
        pose.position.x = self.wp[0]
        pose.position.y = self.wp[1]
        pose.orientation.w = 1.0
        goal = NavigateToPose.Goal()
        goal.pose.pose = pose
        goal.pose.header.frame_id = "map"
        return goal
    
    def feedback_handler(self, blackboard, feedback):
        print(f"Quedan {feedback.distance_remaining} metros")

def main(args=None):
    rclpy.init(args=args)
    node = Node("patrol_yasmin")

    # FSM YASMIN
    sm = StateMachine(outcomes={'end'})
    estado1 = GotoWPState([0.0,0.0])
    estado2 = GotoWPState([-10.0,-9.0])        
    estados_cancelables = [estado1, estado2]     
    sm.add_state('WP1', estado1, transitions={'succeeded':'WP2'})
    sm.add_state('WP2', estado2, transitions={'succeeded':'WP1'})
    sm.set_start_state('WP1')
    sm.validate()
    
 
    # Ejecuta FSM 
    try:
        outcome = sm.execute(Blackboard())
    except KeyboardInterrupt:
        for state in estados_cancelables: 
            state.cancel_state()
        raise
    finally:    
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
