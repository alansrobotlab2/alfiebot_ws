cmake_minimum_required(VERSION 3.8)
project(alfie_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages (arm-board subset of the canonical alfie_msgs package)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ServoCmd.msg"
  "msg/ServoState.msg"
  "msg/ArmCmd.msg"
  "msg/ArmState.msg"
  "msg/ServoMemoryMap.msg"
  "srv/ServoService.srv"
  DEPENDENCIES std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
