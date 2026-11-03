cmake_minimum_required(VERSION 3.0.2)
project(hih)

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
  LIBRARIES hih_robot_model hih_controller hih_navigation
  CATKIN_DEPENDS aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib roscpp gimbalrotor diagnostic_msgs std_msgs std_srvs geometry_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  )

add_library(hih_sensor_pluginlib  src/sensor/imu.cpp)
target_link_libraries(hih_sensor_pluginlib ${catkin_LIBRARIES})
add_dependencies(hih_sensor_pluginlib aerial_robot_msgs_generate_messages_cpp spinal_generate_messages_cpp hih_generate_messages_cpp)

add_library(hih_controller src/control/hih_controller.cpp src/control/hih_unified_controller.cpp src/control/formation_momentum_observer.cpp)
target_link_libraries(hih_controller hih_sensor_pluginlib ${catkin_LIBRARIES} ${OsqpEigen_LIBRARIES})
add_dependencies(hih_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(hih_robot_model src/model/hih_robot_model.cpp)
target_link_libraries(hih_robot_model ${catkin_LIBRARIES})

add_library(hih_navigation src/hih_navigation.cpp)
target_link_libraries(hih_navigation ${catkin_LIBRARIES})
add_dependencies(hih_navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS hih_sensor_pluginlib hih_robot_model hih_controller hih_navigation 
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

# catkin_install_python(PROGRAMS script/hih/kondo_control.py
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
# NodeHandle calls that hih_unified_controller.cpp's transitive dependencies
# pull in (otherwise they block retrying to contact a master). The test itself
# exercises only the allocation math via a friend fixture; no hardware needed.
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(hih_test_unified_allocation
    test/test_unified_allocation.test
    test/test_unified_allocation.cpp
    src/control/hih_unified_controller.cpp)
  if(TARGET hih_test_unified_allocation)
    add_dependencies(hih_test_unified_allocation
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(hih_test_unified_allocation
      -Wl,--no-as-needed
      hih_navigation hih_robot_model
      ${catkin_LIBRARIES} ${OsqpEigen_LIBRARIES})
  endif()
endif()

# Export the package-owned headers for downstream HiH extensions.
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS
  script/keyboard_command.py
  script/keyboard_command_multi.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
