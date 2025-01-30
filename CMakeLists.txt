cmake_minimum_required(VERSION 3.8)
project(rovc_comm_mqtt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PahoMqttCpp REQUIRED)  # ✅ Added Paho MQTT library

# Add executable for publisher node
add_executable(publisher_node src/publisher_node.cpp)
target_link_libraries(publisher_node PahoMqttCpp::paho-mqttpp3)  # ✅ Link MQTT library
ament_target_dependencies(publisher_node rclcpp std_msgs PahoMqttCpp)  # ✅ Include MQTT in dependencies

# Add executable for subscriber node
add_executable(subscriber_node src/subscriber_node.cpp)
target_link_libraries(subscriber_node PahoMqttCpp::paho-mqttpp3)  # ✅ Link MQTT library
ament_target_dependencies(subscriber_node rclcpp std_msgs PahoMqttCpp)  # ✅ Include MQTT in dependencies

# Install executables
install(TARGETS
  publisher_node
  subscriber_node
  DESTINATION lib/${PROJECT_NAME})

# Install other files (optional)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright check if needed
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

