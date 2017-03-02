cmake_minimum_required(VERSION 2.8.3)
project(swiftnav_piksi_msgs)

set(MSG_DEPS
  std_msgs
)

set(BUILD_DEPS
  ${MSG_DEPS}
  message_generation
)

set(RUNTIME_DEPS
  ${MSG_DEPS}
  message_runtime
)

find_package(catkin REQUIRED COMPONENTS ${BUILD_DEPS})
include_directories(${catkin_INCLUDE_DIRS})

add_message_files(FILES
  SbpBaseline.msg
  SbpGpsTime.msg
  SbpPosLlh.msg
)

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS ${RUNTIME_DEPS})
