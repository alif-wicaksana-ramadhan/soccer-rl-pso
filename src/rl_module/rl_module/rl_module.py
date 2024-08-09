import torch
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from rl_module_interfaces.srv import Inference
from collections import namedtuple
from typing import List, Tuple
from enum import Enum
import random
import numpy as np

from rl_module import ReplayMemory, QNetwork


Transition = namedtuple("Transition", ("state", "action", "next_state", "reward"))

class SystemState(Enum):
    IDLE = 1
    RUNNING = 2
    COMPLETED = 3


class RLModule(Node):
    def __init__(self, lr=5e-4):
        super().__init__('rl_module')
        self.system_state = SystemState.IDLE
        self.lr = lr
        self.policy_net = QNetwork(self.lr)
        self.target_net = QNetwork(self.lr)
        self.target_net.net.load_state_dict(
            self.policy_net.net.state_dict()
        )  # Copy the weight of the policy network
        self.rm = ReplayMemory()
        self.burn_in_memory()
        self.batch_size = 32
        self.gamma = 0.99
        self.c = 0

        self.rl_service = self.create_service(Inference, 'rl_module', self.inference_callback)
    
    def inference_callback(self, request, response):
        robot_position = np.array([request.robot_pose.x, request.robot_pose.y], np.float32)
        robot_velocity = np.array([request.robot_vel.x, request.robot_vel.y], np.float32)
        target_position = np.array([request.target_pos.x, request.target_pos.y], np.float32)
        enemies_positions = np.array([np.array([pos.x, pos.y], np.float32) for pos in request.enemies])
        predicted_action = self.target_net.net(robot_position, robot_velocity, target_position, enemies_positions)
        print(predicted_action)
        action = Point()
        action.x = float(predicted_action[0] * 9.0)
        action.y = float(predicted_action[1] * 6.0)
        response.action = action
        return response

    def burn_in_memory(self):
        # Initialize replay memory with a burn-in number of episodes/transitions.
        cnt = 0
        terminated = False
        truncated = False
        # state, _ = self.env.reset()
        # state = torch.tensor(state, dtype=torch.float32).unsqueeze(0)

        # # Iterate until we store "burn_in" buffer
        # while cnt < self.rm.burn_in:
        #     # Reset environment if terminated or truncated
        #     if terminated or truncated:
        #         state, _ = self.env.reset()
        #         state = torch.tensor(state, dtype=torch.float32).unsqueeze(0)

        #     # Randomly select an action (left or right) and take a step
        #     action = torch.tensor(random.sample([0, 1], 1)[0]).reshape(1, 1)
        #     next_state, reward, terminated, truncated, _ = self.env.step(action.item())
        #     reward = torch.tensor([reward])
        #     if terminated:
        #         next_state = None
        #     else:
        #         next_state = torch.tensor(next_state, dtype=torch.float32).unsqueeze(0)

        #     # Store new experience into memory
        #     transition = Transition(state, action, next_state, reward)
        #     self.rm.memory.append(transition)
        #     state = next_state
        #     cnt += 1

def main(args=None):
    rclpy.init(args=args)
    node = RLModule()
    node.get_logger().info('This is starting point')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()