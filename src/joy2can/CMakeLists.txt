cmake_minimum_required(VERSION 3.5)
project(joy2can)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(tranceiver
	src/tranceiver.cpp
	src/canbus.cpp
)
ament_target_dependencies(tranceiver rclcpp std_msgs sensor_msgs)

install(TARGETS
  tranceiver
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(kinematics
	src/kinematics.cpp
	src/kinematicsCalculations.cpp
	src/canbus.cpp
	src/ik.cpp
	src/quintic.cpp
	src/trajectoryPlanner.cpp
)
ament_target_dependencies(kinematics rclcpp std_msgs sensor_msgs Eigen3)

install(TARGETS
  kinematics
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/odriveCAN.py
  scripts/cmd_id_enums.py
  scripts/motor.py
  scripts/motorSetpointSub.py
  scripts/odrive-cansimple.dbc
  scripts/app.py
  scripts/default.json
  DESTINATION lib/${PROJECT_NAME}
)

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
