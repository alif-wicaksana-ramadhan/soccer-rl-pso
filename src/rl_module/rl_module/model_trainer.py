from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Float32, Int32, Bool
from enum import Enum
from rl_module_interfaces.srv import Inference, RunSim
from rl_module import ReplayMemory, SoccerObserverModel
import numpy as np
import rclpy



class ModelTrainer(Node):
    def __init__(self, iteration=100):
        super().__init__('model_trainer')
        # self.timer = self.create_timer(0.5, self.main_loop)

        self.model = SoccerObserverModel()
        self.n_param = len(self.model.get_all_parameters())

    def run_sim_server(self):
        client = self.create_client(RunSim, "run_sim/start")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server..")
        
        request = RunSim.Request()
        state = Bool()
        state.data = True
        request.data = state

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def iterate(self):
        self.run_sim_server()
        


def main(args=None):
    rclpy.init(args=args)
    node = ModelTrainer()
    print(node.iterate())
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()