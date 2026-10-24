cmake_minimum_required(VERSION 3.5)
project(alfie_foxglove_extension)

find_package(ament_cmake REQUIRED)

# The extension itself is an npm project built outside colcon (see README.md).
# Whatever `npm run package` last produced is installed to share/ so nodes on
# the robot can hand it out to laptops for drag-and-drop install.
file(GLOB FOXE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.foxe")
if(FOXE_FILES)
  install(FILES ${FOXE_FILES} DESTINATION share/${PROJECT_NAME}/extensions)
else()
  message(STATUS "${PROJECT_NAME}: no .foxe found, run `npm run package` first")
endif()

ament_package()
