cmake_minimum_required(VERSION 2.8.3)
project(ros_h264_streamer)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  image_transport 
  cv_bridge 
  msg_all  
  sensor_msgs
  std_msgs
  )
include_directories(${catkin_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# OpenCV
find_package(OpenCV REQUIRED)
#输出cv版本
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
#输出cv库的路径
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
include_directories(${OpenCV_INCLUDE_DIRS})

if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS roscpp image_transport sensor_msgs std_msgs msg_all cv_bridge
  )
  find_package(dynamic_graph_bridge_msgs)
    if(${dynamic_graph_bridge_msgs_FOUND})
      include_directories(${dynamic_graph_bridge_msgs_INCLUDE_DIRS})
      add_definitions("-DHAS_DGBRIDGE_MSGS")
    endif()
endif()

set(ros_h264_streamer_src
  src/h264_encoder.cpp
  src/h264_decoder.cpp
)

add_library(ros_h264_streamer SHARED ${ros_h264_streamer_src})
target_link_libraries(ros_h264_streamer ${catkin_LIBRARIES} x264 avcodec avutil swscale)

add_executable(image_encoder_node src/image_encoder_node.cpp)
target_link_libraries(image_encoder_node ros_h264_streamer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(image_decoder_node src/image_decoder_node.cpp)
target_link_libraries(image_decoder_node ros_h264_streamer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
