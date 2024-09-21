cmake_minimum_required(VERSION 3.8)
project(cca_kinova_gen3_7dof)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find and link required packages
find_package(Eigen3 REQUIRED)  # Linear algebra library
# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cca_ros REQUIRED)
find_package(geometry_msgs)
find_package(tf2_ros)
find_package(std_srvs)

# Specify the node executables
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# Specify ROS dependencies for the target
ament_target_dependencies(${PROJECT_NAME}_node rclcpp cca_ros)

# Link Eigen libraries against this project library
target_link_libraries(${PROJECT_NAME}_node Eigen3::Eigen)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
