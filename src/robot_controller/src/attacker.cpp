#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include <thread>
#include "el_force/el_force.hpp"
#include "field_interpreter_interfaces/srv/get_robot_in_radius.hpp"
#include "rl_module_interfaces/srv/inference.hpp"

enum class SystemState
{
  IDLE,
  RUNNING,
  COMPLETED
};

class Attacker : public rclcpp::Node
{
private:
  SystemState system_state_;
  std::string robot_name_;
  geometry_msgs::msg::Pose2D pose_;
  geometry_msgs::msg::Twist vel_;
  geometry_msgs::msg::Point target_;
  geometry_msgs::msg::Point rl_action_;
  std_msgs::msg::Float32 sim_time_;
  std_msgs::msg::Int32 sim_state_;
  std::vector<geometry_msgs::msg::Pose2D> defenders_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sim_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sim_time_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sim_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sim_stop_pub_;
  rclcpp::Client<field_interpreter_interfaces::srv::GetRobotInRadius>::SharedPtr robot_client_;
  rclcpp::Client<rl_module_interfaces::srv::Inference>::SharedPtr rl_client_;

  ElForce el_force_;

  void pose_callback_(const geometry_msgs::msg::Pose2D::SharedPtr msg)
  {
    pose_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "pose: %f, %f, %f", pose_.position.x, pose_.position.y, pose_.position.z);
  }

  void vel_callback_(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    vel_ = *msg;
  }

  void target_callback_(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    target_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "target: %f, %f, %f", target_.x, target_.y, target_.z);
  }

  void sim_state_callback_(const std_msgs::msg::Int32::SharedPtr msg)
  {
    sim_state_ = *msg;
  }

  void sim_time_callback_(const std_msgs::msg::Float32::SharedPtr msg)
  {
    sim_time_ = *msg;
  }

  double calculateNorm(std::vector<double> vec)
  {
    double sum = 0.0;
    for (int i = 0; i < (int)vec.size(); ++i)
    {
      sum += std::pow(vec[i], 2);
    }
    return std::sqrt(sum);
  }

  void get_robots_request(const float radius)
  {
    // Create a request object and populate it with data
    auto request = std::make_shared<field_interpreter_interfaces::srv::GetRobotInRadius::Request>();
    request->radius = radius;
    request->position.x = pose_.x;
    request->position.y = pose_.y;
    request->position.theta = pose_.theta;

    // Define a callback function that will be called when the service response is received
    using ServiceResponseFuture = rclcpp::Client<field_interpreter_interfaces::srv::GetRobotInRadius>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
    {
      auto response = future.get();
      defenders_.clear();
      for (const auto &pos : response->points)
      {
        // RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, theta=%.2f", pos.x, pos.y, pos.theta);
        defenders_.push_back(pos);
      }
    };

    // Send the request asynchronously and set the callback to handle the response
    auto future_result = robot_client_->async_send_request(request, response_received_callback);
  }

  void rl_inference_reqeust()
  {
    auto request = std::make_shared<rl_module_interfaces::srv::Inference::Request>();
    geometry_msgs::msg::Point vel_msg;
    vel_msg.x = vel_.linear.x;
    vel_msg.y = vel_.linear.y;

    request->robot_pose = pose_;
    request->target_pos = target_;
    request->enemies = defenders_;
    request->robot_vel = vel_msg;

    // Define a callback function that will be called when the service response is received
    using ServiceResponseFuture = rclcpp::Client<rl_module_interfaces::srv::Inference>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
    {
      auto response = future.get();
      rl_action_ = response->action;
    };

    // Send the request asynchronously and set the callback to handle the response
    auto future_result = rl_client_->async_send_request(request, response_received_callback);
  }

  void running_loop_()
  {
    geometry_msgs::msg::Twist msg;
    rl_inference_reqeust();
    get_robots_request(10.0);
    RCLCPP_INFO(this->get_logger(), "Detected Enemy: %ld", defenders_.size());
    for (const geometry_msgs::msg::Pose2D defender : defenders_)
    {
      RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, theta=%.2f", defender.x, defender.y, defender.theta);
      std::vector<double> enemyPos = {defender.x, defender.y};
      el_force_.addEnemy(enemyPos);
    }

    std::vector<double> target_pos = {target_.x, target_.y, target_.z};
    std::vector<double> pos = {pose_.x, pose_.y, pose_.theta};
    el_force_.setTarget(target_pos);
    std::vector<double> forceVec = el_force_.calculateForce(pos);

    forceVec[0] *= rl_action_.x;
    forceVec[1] *= rl_action_.y;
    rl_action_.x = 0.0;
    rl_action_.y = 0.0;

    for (int i = 0; i < (int)forceVec.size(); i++)
    {
      if (forceVec[i] > 100.0)
      {
        forceVec[i] = 100.0;
      }
      else if (forceVec[i] < -100.0)
      {
        forceVec[i] = -100.0;
      }
    }

    msg.linear.x = forceVec[0];
    msg.linear.y = forceVec[1];
    msg.linear.z = forceVec[2];

    cmd_vel_pub_->publish(msg);

    // RCLCPP_INFO(this->get_logger(), "Target pos: %f, %f, %f", target_.x, target_.y, target_.z);
    // RCLCPP_INFO(this->get_logger(), "Robot pos: %f, %f, %f", pose_.x, pose_.y, pose_.theta);
    // RCLCPP_INFO(this->get_logger(), "Vector: %f, %f, %f", target_.x - pose_.x, target_.y - pose_.y, target_.z - pose_.theta);
    // RCLCPP_INFO(this->get_logger(), "Force: %f, %f, %f", forceVec[0], forceVec[1], forceVec[2]);
  }

  void main_loop_()
  {
    std_msgs::msg::Bool sim_msg;
    std::vector<double> targetVec = {target_.x - pose_.x, target_.y - pose_.y};
    std::cout << sim_state_.data << std::endl;

    switch (system_state_)
    {

    case SystemState::IDLE:
      RCLCPP_INFO(this->get_logger(), "IDLE");
      sim_start_pub_->publish(sim_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      if (sim_state_.data)
      {
        system_state_ = SystemState::RUNNING;
      }
      break;

    case SystemState::RUNNING:
      RCLCPP_INFO(this->get_logger(), "RUNNING");
      running_loop_();

      if (calculateNorm(targetVec) < 1.0)
      {
        system_state_ = SystemState::COMPLETED;
      }
      else if (sim_time_.data > 30.0)
      {
        system_state_ = SystemState::COMPLETED;
      }
      break;

    case SystemState::COMPLETED:
      RCLCPP_INFO(this->get_logger(), "COMPLETED");
      sim_stop_pub_->publish(sim_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      if (!sim_state_.data)
      {
        system_state_ = SystemState::IDLE;
      }
    }
  }

public:
  Attacker(const std::string &robot_name) : Node("attacker"), robot_name_(robot_name), el_force_(5.0, 10.0, 2.0, 70.0)
  {
    std::string cmd_vel_topic_ = "/" + robot_name_ + "/cmd_vel";
    std::string pose_topic_ = "/" + robot_name_ + "/pose";
    std::string vel_topic_ = "/" + robot_name_ + "/vel";

    system_state_ = SystemState::IDLE;
    rl_action_.x = 0.0;
    rl_action_.y = 0.0;

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    sim_start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/startSimulation", 10);
    sim_stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/stopSimulation", 10);
    sim_state_sub_ = this->create_subscription<std_msgs::msg::Int32>("/simulationState", 10, std::bind(&Attacker::sim_state_callback_, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(pose_topic_, 10, std::bind(&Attacker::pose_callback_, this, std::placeholders::_1));
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(vel_topic_, 10, std::bind(&Attacker::vel_callback_, this, std::placeholders::_1));
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/target", 10, std::bind(&Attacker::target_callback_, this, std::placeholders::_1));
    sim_time_sub_ = this->create_subscription<std_msgs::msg::Float32>("/simulationTime", 10, std::bind(&Attacker::sim_time_callback_, this, std::placeholders::_1));
    robot_client_ = this->create_client<field_interpreter_interfaces::srv::GetRobotInRadius>("get_defender_in_radius");
    rl_client_ = this->create_client<rl_module_interfaces::srv::Inference>("rl_module");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Attacker::main_loop_, this));
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (argc < 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run robot_controller attacker <robot_name>");
    return 1;
  }

  std::string robot_name = argv[1];
  auto node = std::make_shared<Attacker>(robot_name);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  // rclcpp::spin(std::make_shared<Attacker>(robot_name));
  rclcpp::shutdown();
  return 0;
}
