add_compile_options(-std=c++11)
cmake_minimum_required(VERSION 2.8.3)
project(ndr_downscale)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs
)

add_message_files(FILES one.msg two.msg three.msg seven.msg six.msg twentyone.msg trans.msg buttons.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
  LIBRARIES ndr_downscale
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /usr/xenomai/include
    /opt/etherlab/include
)

link_directories(
    /usr/xenomai/lib
    /opt/etherlab/lib
)

link_libraries(
    native
    xenomai
    rtdm
    pthread
    pthread_rt
    ethercat
    ethercat_rtdm
# -static
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/aidinvi.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## sub
add_executable(sub_downscale_POSITION_mode subsrc/sub_downscale_POSITION_mode.cpp)
add_dependencies(sub_downscale_POSITION_mode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sub_downscale_POSITION_mode ${catkin_LIBRARIES} )
add_dependencies(sub_downscale_POSITION_mode ndr_downscale_generate_messages_cpp)

add_executable(sub_downscale_POSITION_mode_backup subsrc/sub_downscale_POSITION_mode_backup.cpp)
add_dependencies(sub_downscale_POSITION_mode_backup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sub_downscale_POSITION_mode_backup ${catkin_LIBRARIES} )
add_dependencies(sub_downscale_POSITION_mode_backup ndr_downscale_generate_messages_cpp)


