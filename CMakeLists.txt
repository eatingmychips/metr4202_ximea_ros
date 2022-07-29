cmake_minimum_required( VERSION 3.16.3 )

project( ximea_camera )
find_package( PkgConfig REQUIRED )
find_package( OpenCV REQUIRED )

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)



add_library( ximea_cv src/ximea_cv.cpp )
add_executable( ximea_demo src/ximea_demo.cpp )
target_link_libraries( ximea_demo ximea_cv ${OpenCV_LIBS} m3api )
