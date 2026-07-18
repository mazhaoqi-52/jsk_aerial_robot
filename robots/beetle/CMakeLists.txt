cmake_minimum_required(VERSION 3.0.2)
project(beetle)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  gimbalrotor
  diagnostic_msgs
  std_msgs
  std_srvs
  geometry_msgs
  message_generation
  )

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

add_message_files(FILES
  ModuleModel.msg
  TaggedWrench.msg
  TaggedWrenches.msg
  UnifiedControlReference.msg
  )

catkin_python_setup()

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES beetle_robot_model beetle_controller beetle_navigation
  CATKIN_DEPENDS aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib roscpp gimbalrotor diagnostic_msgs std_msgs std_srvs geometry_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  )

add_library(beetle_sensor_pluginlib  src/sensor/imu.cpp)
target_link_libraries(beetle_sensor_pluginlib ${catkin_LIBRARIES})
add_dependencies(beetle_sensor_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp beetle_generate_messages_cpp)

add_library(beetle_controller src/control/beetle_controller.cpp src/control/beetle_unified_controller.cpp src/control/formation_momentum_observer.cpp)
target_link_libraries(beetle_controller beetle_sensor_pluginlib ${catkin_LIBRARIES} ${OsqpEigen_LIBRARIES})
add_dependencies(beetle_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(beetle_robot_model src/model/beetle_robot_model.cpp)
target_link_libraries(beetle_robot_model ${catkin_LIBRARIES})

add_library(beetle_navigation src/beetle_navigation.cpp)
target_link_libraries(beetle_navigation ${catkin_LIBRARIES})
add_dependencies(beetle_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS beetle_sensor_pluginlib beetle_robot_model beetle_controller beetle_navigation 
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config plugins launch robots urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(DIRECTORY script
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

# catkin_install_python(PROGRAMS script/beetle/kondo_control.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#   )
catkin_install_python(PROGRAMS
  script/demos/valve_rotation_demo/valve_rotation_smach_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  script/demos/valve_rotation_demo/valve_rotation_states_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  script/demos/valve_rotation_demo/valve_rotation_fang_single.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  script/demos/valve_rotation_demo/valve_rotation_formation_clean.py
  script/demos/valve_rotation_demo/static_downward_valve_torque.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  script/demos/aerial_towing_demo/load_towing_formation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  script/demos/aerial_pushing_demo/wall_pushing_formation.py
  script/demos/aerial_pushing_demo/wall_pushing_single_uav.py
  script/demos/aerial_pushing_demo/force_sensor_auto_calib.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---- Unit test for the unified-mode allocation QP ----
# Run under rostest so an auto-launched roscore satisfies the ROS logging /
# NodeHandle calls that beetle_unified_controller.cpp's transitive dependencies
# pull in (otherwise they block retrying to contact a master). The test itself
# exercises only the allocation math via a friend fixture; no hardware needed.
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_unified_allocation
    test/test_unified_allocation.test
    test/test_unified_allocation.cpp
    src/control/beetle_unified_controller.cpp)
  if(TARGET test_unified_allocation)
    add_dependencies(test_unified_allocation
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_unified_allocation
      -Wl,--no-as-needed
      beetle_navigation beetle_robot_model
      ${catkin_LIBRARIES} ${OsqpEigen_LIBRARIES})
  endif()
endif()
