cmake_minimum_required(VERSION 3.8)
project(sfm)

# find dependencies
find_package(ament_cmake REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)

# define a library target called sfm
add_library(sfm SHARED src/sfm.cpp)
 
# install the include/sfm directory to the install/include/sfm
install(
  DIRECTORY include/sfm
  DESTINATION include
)
 
install(
  TARGETS sfm
  EXPORT export_sfm
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
 
# this line to exports the library
ament_export_targets(export_sfm HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)
ament_package()