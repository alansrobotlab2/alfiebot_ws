cmake_minimum_required(VERSION 3.5)
project(alfie_foxglove_extension)

find_package(ament_cmake REQUIRED)

# The extension itself is an npm project built outside colcon (see README.md).
# Whatever `npm run package` last produced is installed to share/ so nodes on
# the robot can hand it out to laptops for drag-and-drop install.
# CONFIGURE_DEPENDS re-globs on every build, so a newly packaged version is
# picked up without wiping build/.
file(GLOB FOXE_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.foxe")
# Installs only ever add files; clear out the previous version first so share/
# holds exactly what is in the source folder.
install(CODE "
  file(GLOB _old_foxe \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/extensions/*.foxe\")
  if(_old_foxe)
    file(REMOVE \${_old_foxe})
  endif()
")
if(FOXE_FILES)
  install(FILES ${FOXE_FILES} DESTINATION share/${PROJECT_NAME}/extensions)
else()
  message(STATUS "${PROJECT_NAME}: no .foxe found, run `npm run package` first")
endif()

ament_package()
