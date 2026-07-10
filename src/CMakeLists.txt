set(_deps rclcpp sensor_msgs std_msgs std_srvs)

add_executable(ak_motor_cable_control_node ak_motor_cable_control_node.cpp)
target_compile_features(ak_motor_cable_control_node PRIVATE cxx_std_17)
ament_target_dependencies(ak_motor_cable_control_node ${_deps})

install(TARGETS ak_motor_cable_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
