cmake_minimum_required(VERSION 3.10)
# A Rust package, built the way aav_imu_yahboom is: CMake only runs cargo and installs the
# result, so colcon, `ros2 run` and launch files treat radar_clusters like any other node.
project(aav_radar_clusters NONE)

find_package(ament_cmake REQUIRED)

find_program(CARGO cargo)
if(NOT CARGO)
  message(FATAL_ERROR "cargo not found: build with `pixi run build`, or install Rust 1.85 "
    "or newer with rustup (Ubuntu's apt cargo is too old for rclrs)")
endif()

# Build in colcon's build folder, not in src/. cargo itself decides what's out of date.
# make's jobserver flags are dropped: cargo can't reach the jobserver from here, and says so.
set(CARGO_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/target)
add_custom_target(cargo_build ALL
  COMMAND ${CMAKE_COMMAND} -E env --unset=MAKEFLAGS --unset=MFLAGS --unset=CARGO_MAKEFLAGS
    CARGO_TARGET_DIR=${CARGO_TARGET_DIR}
    ${CARGO} build --release --locked --quiet --bin radar_clusters
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "cargo build --release --bin radar_clusters"
  VERBATIM)

install(PROGRAMS ${CARGO_TARGET_DIR}/release/radar_clusters
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
