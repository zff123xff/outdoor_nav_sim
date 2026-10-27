cmake_minimum_required(VERSION 3.5)
project(msg_set_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  nav_msgs
  std_msgs
  mavros_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AdjustTasks.msg"
  "msg/AntiJamTask.msg"
  "msg/AreaSample.msg"
  "msg/Bbox.msg"
  "msg/Bboxes.msg"
  "msg/CruiseInput.msg"
  "msg/CruiseMission.msg"
  "msg/DroneOdometry.msg"
  "msg/DropReturnMission.msg"
  "msg/DropInput.msg"
  "msg/EnrouteInput.msg"
  "msg/EnrouteOutput.msg"
  "msg/ExplorationMission.msg"
  "msg/FeatureInfo.msg"
  "msg/FixedPointTask.msg"
  "msg/FormationInput.msg"
  "msg/MultiGoal.msg"
  "msg/MultiGoalPoint.msg"
  "msg/MultiTask.msg"
  "msg/PolyTraj.msg"
  "msg/SearchAttack.msg"
  "msg/SingleTask.msg"
  "msg/TakeoffGathering.msg"
  "msg/GatherGoal.msg"
  "msg/GatherInput.msg"
  "msg/UsmCtrlState.msg"
  "msg/UsmDataInfo.msg"
  "msg/UsmAlgoCmd.msg"
  "msg/ReturnInput.msg"
  "msg/RelayInput.msg"
  "msg/RiseMoveInput.msg"
  "msg/SearchMission.msg"
  "msg/SeInput.msg"
  "msg/SfInput.msg"
  "msg/SfOutput.msg"
  "msg/SwarmMincoTraj.msg"
  "msg/SwarmMincoTrajInt.msg"
  "msg/JumpTask.msg"
  "msg/ObjectInfo.msg"
  "msg/ObjectDetection.msg"
  "msg/CoverParam.msg"
  "msg/CoverIndex.msg"
  "msg/CoverTask.msg"
  "msg/MultiTaskConfirm.msg"
  "msg/CoverConfirm.msg"
  "msg/ConfirmInfo.msg"
  "msg/EtOutput.msg"
  "msg/EtInput.msg"
  "msg/GlobalParam.msg"
  "msg/TemplatesState.msg"
  "msg/TemplatesTransit.msg"
  "msg/RelayTask.msg"
  "msg/TargetPose.msg"
  "msg/TargetsPose.msg"
  "msg/TransformationPoints.msg"
  DEPENDENCIES ${dependencies}
)

install(FILES config/mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
