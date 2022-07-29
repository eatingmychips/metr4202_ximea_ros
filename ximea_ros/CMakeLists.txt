cmake_minimum_required( VERSION 3.16.3 )

project( ximea_ros )
find_package( PkgConfig REQUIRED )
find_package( OpenCV REQUIRED )

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ximea_cv
  CATKIN_DEPENDS roscpp rospy std_msgs
)


add_library( ximea_cv src/ximea_cv.cpp )
add_executable( ximea_demo src/ximea_demo.cpp )
target_link_libraries( ximea_demo ximea_cv ${OpenCV_LIBS} m3api )


install(TARGETS ximea_demo ximea_cv
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

