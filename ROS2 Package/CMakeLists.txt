cmake_minimum_required(VERSION 3.5)
project(mobile_project_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add any other dependencies here using find_package

ament_package()


install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  DIRECTORY
    description
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



# Add any other install directives here
