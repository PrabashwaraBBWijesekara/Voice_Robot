cmake_minimum_required(VERSION 3.8)
project(voice_control_robot_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Install Python modules
install(
  PROGRAMS 
    src/voice_cmd_vel.py
    src/random_command_publisher.py
    src/google_voice_recognizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(builtin_interfaces)

ament_package()
