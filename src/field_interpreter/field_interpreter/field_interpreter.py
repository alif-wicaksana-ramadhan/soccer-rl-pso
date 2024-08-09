import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from field_interpreter_interfaces.srv import GetRobotInRadius
from typing import List, Tuple


class FieldInterpreter(Node):

    def __init__(self, attacker_total:int = 0, defender_total:int = 0):
        super().__init__('get_defender_in_radius')
        self.attacker_service = self.create_service(GetRobotInRadius, 'get_attacker_in_radius', self.get_attacker_callback)
        self.defender_service = self.create_service(GetRobotInRadius, 'get_defender_in_radius', self.get_defender_callback)
        self.attacker1_subber = self.create_subscription(Pose2D, '/attacker1/pose', self.attacker1_pose_callback, 10)
        self.attacker2_subber = self.create_subscription(Pose2D, '/attacker2/pose', self.attacker2_pose_callback, 10)
        self.attacker3_subber = self.create_subscription(Pose2D, '/attacker3/pose', self.attacker3_pose_callback, 10)
        self.defender1_subber = self.create_subscription(Pose2D, '/defender1/pose', self.defender1_pose_callback, 10)
        self.defender2_subber = self.create_subscription(Pose2D, '/defender2/pose', self.defender2_pose_callback, 10)
        self.defender3_subber = self.create_subscription(Pose2D, '/defender3/pose', self.defender3_pose_callback, 10)

        self.attacker_total = attacker_total
        self.defender_total = defender_total
        self.attackerPoses:List[Pose2D] = []
        self.defenderPoses:List[Pose2D] = []

        for _ in range(self.attacker_total):
            self.attackerPoses.append(Pose2D(x=50.0, y=50.0, theta=50.0))
        
        for _ in range(self.defender_total):
            self.defenderPoses.append(Pose2D(x=50.0, y=50.0, theta=50.0))
    
    def get_attacker_callback(self, request, response):
        self.get_logger().info(f"Request received Radius: {request.radius}, Pos: {request.position.x}, {request.position.y}")
        position = request.position
        radius = request.radius
        robots = []

        for attacker in self.attackerPoses:
            if self.euclidean_distance((position.x, position.y), (attacker.x, attacker.y)) < radius:
                robots.append(attacker)
            
        response.points = robots
        return response

    def get_defender_callback(self, request, response):
        self.get_logger().info(f"Request received Radius: {request.radius}, Pos: {request.position.x}, {request.position.y}")
        position = request.position
        radius = request.radius
        robots = []

        for defender in self.defenderPoses:
            if self.euclidean_distance((position.x, position.y), (defender.x, defender.y)) < radius:
                robots.append(defender)
            
        response.points = robots
        return response
    
    def euclidean_distance(self, point1:Tuple, point2:Tuple):
        if len(point1) != len(point2):
            raise ValueError("Both points must have the same number of dimensions.")
        
        distance = math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2)))
        return distance

    def attacker1_pose_callback(self, msg):
        self.attackerPoses[0].x = msg.x
        self.attackerPoses[0].y = msg.y
        self.attackerPoses[0].theta = msg.theta

    def attacker2_pose_callback(self, msg):
        self.attackerPoses[1].x = msg.x
        self.attackerPoses[1].y = msg.y
        self.attackerPoses[1].theta = msg.theta

    def attacker3_pose_callback(self, msg):
        self.attackerPoses[2].x = msg.x
        self.attackerPoses[2].y = msg.y
        self.attackerPoses[2].theta = msg.theta
    
    def defender1_pose_callback(self, msg):
        self.defenderPoses[0].x = msg.x
        self.defenderPoses[0].y = msg.y
        self.defenderPoses[0].theta = msg.theta
    
    def defender2_pose_callback(self, msg):
        self.defenderPoses[1].x = msg.x
        self.defenderPoses[1].y = msg.y
        self.defenderPoses[1].theta = msg.theta

    def defender3_pose_callback(self, msg):
        self.defenderPoses[2].x = msg.x
        self.defenderPoses[2].y = msg.y
        self.defenderPoses[2].theta = msg.theta


def main(args=None):
    rclpy.init(args=args)
    node = FieldInterpreter(attacker_total=1, defender_total=2)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()