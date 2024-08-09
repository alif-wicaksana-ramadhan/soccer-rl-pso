cmake_minimum_required(VERSION 3.8)
project(el_force)
# find dependencies
find_package(ament_cmake REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)

# define a library target called el_force
add_library(el_force SHARED src/el_force.cpp)
 
# install the include/el_force directory to the install/include/el_force
install(
  DIRECTORY include/el_force
  DESTINATION include
)
 
install(
  TARGETS el_force
  EXPORT export_el_force
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
 
# this line to exports the library
ament_export_targets(export_el_force HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)
ament_package()