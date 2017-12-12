cmake_minimum_required(VERSION 2.8.3)
project(adi_driver)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(adis16470_node src/adis16470_node.cpp src/adis16470.cpp)
target_link_libraries(adis16470_node ${catkin_LIBRARIES})

add_executable(adxl345_node src/adxl345_node.cpp src/adxl345.cpp)
target_link_libraries(adxl345_node ${catkin_LIBRARIES})

install(TARGETS adis16470_node adxl345_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(STATUS "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()

# catkin_add_gtest(${PROJECT_NAME}-test test/test_adi_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
