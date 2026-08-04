cmake_minimum_required(VERSION 3.8)
project(starcrawler_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch udev systemd
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
