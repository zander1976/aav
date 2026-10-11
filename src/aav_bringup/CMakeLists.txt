cmake_minimum_required(VERSION 3.10)
project(aav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # car.launch.py finds its machine files in this package's own install folder.
  ament_add_pytest_test(test_car_launch test/test_car_launch.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
  ament_add_pytest_test(test_sim_launch test/test_sim_launch.py
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
endif()

ament_package()
