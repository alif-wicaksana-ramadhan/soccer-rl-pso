cmake_minimum_required(VERSION 3.8)
project(rl_module_interfaces)

# find dependencies
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Inference.srv"
  "srv/UpdateModel.srv"
  "srv/RunSim.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
