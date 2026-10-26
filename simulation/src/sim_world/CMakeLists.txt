cmake_minimum_required(VERSION 3.8)
project(south_lake_sim)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/remote_stop_bridge.py scripts/fused_odometry_tf.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config worlds models DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE SOURCE.md DESTINATION share/${PROJECT_NAME})

ament_package()
