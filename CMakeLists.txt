cmake_minimum_required(VERSION 2.8.3)
project(dds)

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_PREFIX_PATH "/usr/lib64;/opt/ros/kinetic;${CMAKE_PREFIX_PATH}")

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(automsgs REQUIRED)
find_package(vector_map REQUIRED)
find_package(messages REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        automsgs
        vector_map
        vector_map_msgs
        visualization_msgs
        std_msgs
        jsk_recognition_msgs
        geometry_msgs)

catkin_package(
        CATKIN_DEPENDS
        automsgs
        vector_map
        vector_map_msgs
        visualization_msgs
        std_msgs
        jsk_recognition_msgs
        geometry_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${automsgs_INCLUDE_DIRS}
        ${vector_map_INCLUDE_DIRS}
        ${vector_map_msgs_INCLUDE_DIRS}
        ${messages_INCLUDE_DIRS}
)

catkin_simple()

file(GLOB SOURCES_HPP "include/*.h")
file(GLOB SOURCES_CPP "src/*.cpp")

cs_add_executable(${PROJECT_NAME} ${SOURCES_HPP} ${SOURCES_CPP})
target_link_libraries(${PROJECT_NAME}
        fastcdr fastrtps
        messages
        ${catkin_LIBRARIES}
        )

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )

cs_install()

cs_export()

