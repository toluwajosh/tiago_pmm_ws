cmake_minimum_required(VERSION 2.8.3)
project(diff_drive_controller)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    message_generation
    std_msgs
    urdf
    tf
    nav_msgs)

add_message_files(
  DIRECTORY msg
  FILES
    WheelData.msg
    WheelDataStamped.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/diff_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES diff_drive_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(catkin COMPONENTS rostest std_srvs controller_manager)
  include_directories(include ${catkin_INCLUDE_DIRS})

  add_executable(diffbot test/diffbot.cpp)
  target_link_libraries(diffbot ${catkin_LIBRARIES})

  add_executable(skidsteerbot test/skidsteerbot.cpp)
  target_link_libraries(skidsteerbot ${catkin_LIBRARIES})

  add_dependencies(tests diffbot skidsteerbot)

  add_rostest_gtest(diff_drive_test
    test/diff_drive_controller.test
    test/diff_drive_test.cpp)
  target_link_libraries(diff_drive_test ${catkin_LIBRARIES})

  add_rostest_gtest(diff_drive_nan_test
    test/diff_drive_controller_nan.test
    test/diff_drive_nan_test.cpp)
  target_link_libraries(diff_drive_nan_test ${catkin_LIBRARIES})

  add_rostest_gtest(diff_drive_limits_test
    test/diff_drive_controller_limits.test
    test/diff_drive_limits_test.cpp)
  target_link_libraries(diff_drive_limits_test ${catkin_LIBRARIES})

  add_rostest_gtest(diff_drive_timeout_test
    test/diff_drive_timeout.test
    test/diff_drive_timeout_test.cpp)
  target_link_libraries(diff_drive_timeout_test ${catkin_LIBRARIES})

  add_rostest(test/diff_drive_multipliers.test)

  add_rostest_gtest(diff_drive_fail_test
    test/diff_drive_wrong.test
    test/diff_drive_fail_test.cpp)
  target_link_libraries(diff_drive_fail_test ${catkin_LIBRARIES})

  add_rostest_gtest(diff_drive_no_preserve_turning_radius_test
    test/diff_drive_controller_no_preserve_turning_radius.test
    test/diff_drive_no_preserve_turning_radius_test.cpp)
  target_link_libraries(diff_drive_no_preserve_turning_radius_test ${catkin_LIBRARIES})

  add_rostest_gtest(diff_drive_publish_wheel_data_test
    test/diff_drive_controller_publish_wheel_data.test
    test/diff_drive_publish_wheel_data_test.cpp)
  target_link_libraries(diff_drive_publish_wheel_data_test ${catkin_LIBRARIES})

  add_rostest(test/diff_drive_bad_urdf.test)

  add_rostest(test/diff_drive_open_loop.test)

  add_rostest(test/skid_steer_controller.test)

  add_rostest(test/skid_steer_no_wheels.test)
endif()
