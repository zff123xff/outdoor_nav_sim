cmake_minimum_required(VERSION 3.16)
project(px4_gps_imu)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# All estimator sources are unmodified upstream files. No download at build time.
file(GLOB ECL_EKF_SOURCES CONFIGURE_DEPENDS vendor/ecl/EKF/*.cpp)
add_library(px4_ecl STATIC ${ECL_EKF_SOURCES}
  vendor/ecl/geo/geo.cpp vendor/ecl/geo_lookup/geo_mag_declination.cpp)
set_target_properties(px4_ecl PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_definitions(px4_ecl PUBLIC ECL_STANDALONE)
target_include_directories(px4_ecl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/ecl>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/matrix>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/px4_gps_imu/vendor/ecl>
  $<INSTALL_INTERFACE:include/px4_gps_imu/vendor/matrix>
  $<INSTALL_INTERFACE:include>)
target_compile_options(px4_ecl PRIVATE -fno-associative-math)

add_executable(px4_ekf_node src/px4_ekf_node.cpp)
target_link_libraries(px4_ekf_node px4_ecl)
ament_target_dependencies(px4_ekf_node rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs)


if(BUILD_TESTING)
  enable_testing()
  add_executable(fusion_test test/fusion_test.cpp)
  target_link_libraries(fusion_test px4_ecl)
  add_test(NAME px4_fusion COMMAND fusion_test)
endif()
install(TARGETS px4_ekf_node DESTINATION lib/${PROJECT_NAME})
install(FILES README.md SOURCE.md vendor/ecl/LICENSE DESTINATION share/${PROJECT_NAME})
# Export PX4 math for the simulated sensor generator without a Gazebo dependency.
install(TARGETS px4_ecl EXPORT px4_ecl_targets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY vendor/ecl vendor/matrix DESTINATION include/px4_gps_imu/vendor
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
ament_export_targets(px4_ecl_targets HAS_LIBRARY_TARGET)
ament_package()
