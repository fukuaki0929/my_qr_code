cmake_minimum_required(VERSION 2.8.3)
project(qr_code)
set(CMAKE_MODULE_PATH ${ZBARCV_SOURCE_DIR})
find_package (OpenCV)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  message_filters
  tf
)

add_message_files(
   FILES
   imageimage.msg
   barcode.msg
   distance.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

find_package(PkgConfig)
pkg_check_modules(PC_ZBAR QUIET zbar)
set(ZBAR_DEFINITIONS ${PC_ZBAR_CFLAGS_OTHER})
find_library(ZBAR_LIBRARIES NAMES zbar
             HINTS ${PC_ZBAR_LIBDIR} ${PC_ZBAR_LIBRARY_DIRS} )
find_path(ZBAR_INCLUDE_DIR Decoder.h
          HINTS ${PC_ZBAR_INCLUDEDIR} ${PC_ZBAR_INCLUDE_DIRS}
          PATH_SUFFIXES zbar )
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZBAR  DEFAULT_MSG  ZBAR_LIBRARIES ZBAR_INCLUDE_DIR)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zbar_opencv
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(img_subscriber src/img_subscriber.cpp)
target_link_libraries(img_subscriber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)
add_executable(img_Transform src/img_Transform.cpp)
target_link_libraries(img_Transform
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)
add_executable(qr_code src/qr_code.cpp)
target_link_libraries(qr_code
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)
add_executable(match src/match.cpp)
target_link_libraries(match
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)
add_executable(img_qr_sync src/img_qr_sync.cpp)
target_link_libraries(img_qr_sync
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)
add_executable(vis_float32_publisher src/vis_float32_publisher.cpp)
target_link_libraries(vis_float32_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)
add_executable(qr_position src/qr_position.cpp)
target_link_libraries(qr_position
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)
add_executable(marker src/marker.cpp)
target_link_libraries(marker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)


add_executable(IMU src/IMU.cpp)
target_link_libraries(IMU ${catkin_LIBRARIES})

add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(tf_listener ${catkin_LIBRARIES})
