cmake_minimum_required(VERSION 3.5)
project(alfie_foxglove_extension)

find_package(ament_cmake REQUIRED)

# The extension itself is an npm project built outside colcon (see README.md).
# Whatever `npm run package` last produced is installed to share/ so nodes on
# the robot can hand it out to laptops for drag-and-drop install.
#
# The .foxe is found at install time, not configure time, so a newly packaged
# version is picked up on the next build without re-configuring (a configure-
# time CONFIGURE_DEPENDS glob works too, but prints "GLOB mismatch!" to stderr
# on every version bump).
install(CODE "set(_foxe_src \"${CMAKE_CURRENT_SOURCE_DIR}\")")
install(CODE [[
  set(_dest "${CMAKE_INSTALL_PREFIX}/share/alfie_foxglove_extension/extensions")
  # Installs only ever add files; clear out the previous version first so
  # share/ holds exactly what is in the source folder.
  file(GLOB _old_foxe "$ENV{DESTDIR}${_dest}/*.foxe")
  if(_old_foxe)
    file(REMOVE ${_old_foxe})
  endif()
  file(GLOB _foxe "${_foxe_src}/*.foxe")
  if(_foxe)
    file(INSTALL ${_foxe} DESTINATION "${_dest}")
  else()
    message(STATUS "alfie_foxglove_extension: no .foxe found, run `npm run package` first")
  endif()
]])

ament_package()
