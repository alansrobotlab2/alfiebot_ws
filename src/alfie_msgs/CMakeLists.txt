cmake_minimum_required(VERSION 3.5)
project(alfie_msgs)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GDBServoMemoryMap.msg"
  "msg/GDBServoCmd.msg"
  "msg/GDBServoState.msg"
  "msg/GDBImu.msg"
  "msg/GDBMagneticField.msg"
  "msg/GDBMotorState.msg"
  "msg/GDBState.msg"
  "msg/GDBCmd.msg"
  "msg/SpeechRequest.msg"
  "msg/Speaking.msg"
  "msg/AudioFrame.msg"
  "msg/ASRResult.msg"
  "msg/GDBDiagnostics.msg"
  "msg/RobotLowState.msg"
  "msg/RobotLowCmd.msg"
  "msg/JetsonState.msg"
  "msg/MotorState.msg"
  "msg/ServoState.msg"
  "srv/GDBServoService.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
 )

ament_package()
