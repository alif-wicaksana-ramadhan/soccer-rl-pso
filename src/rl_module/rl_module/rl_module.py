import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D, Point
from rl_module_interfaces.srv import Inference, UpdateModel
from collections import namedtuple
from typing import List, Tuple
from enum import Enum
import random
import numpy as np

from rl_module import ReplayMemory, SoccerObserverModel


Transition = namedtuple("Transition", ("state", "action", "next_state", "reward"))


class RLModule(Node):
    def __init__(self, lr=5e-4):
        super().__init__('rl_module')
        self.model = SoccerObserverModel()

        self.feed_forward_service = self.create_service(Inference, 'rl_module/feed_forward', self.feed_forward_callback)
        self.update_model_service = self.create_service(UpdateModel, 'rl_module/update_model', self.update_model_callback)
    
    def feed_forward_callback(self, request, response):
        if len(request.enemies) == 0:
            action = Point()
            action.x = 1.0
            action.y = 1.0
            response.action = action
            return response
        
        robot_position = np.array([request.robot_pose.x, request.robot_pose.y], np.float32)
        robot_velocity = np.array([request.robot_vel.x, request.robot_vel.y], np.float32)
        target_position = np.array([request.target_pos.x, request.target_pos.y], np.float32)
        enemies_positions = np.array([np.array([pos.x, pos.y], np.float32) for pos in request.enemies])
        predicted_action = self.model(robot_position, robot_velocity, target_position, enemies_positions)
        print(predicted_action)
        action = Point()
        action.x = float(predicted_action[0] * 10.0)
        action.y = float(predicted_action[1] * 10.0)
        response.action = action
        return response
    
    def update_model_callback(self, request, response):
        new_weights = request.parameters
        self.model.update_all_parameters(new_weights)

        status = Bool()
        status.data = True
        response.status = status
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RLModule()
    node.get_logger().info('This is starting point')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()