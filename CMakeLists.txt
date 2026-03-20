cmake_minimum_required(VERSION 3.5)
project(trajectory_generator LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

#set(abb_libegm_INCLUDE_DIRS "C:/TFG_Irene_Calvo_Corredera/abb_libegm/include/abb_libegm")
#set(abb_libegm_LIBRARIES "C:/TFG_Irene_Calvo_Corredera/abb_libegm/build/Release/abb_libegm.lib")

add_definitions( -DBOOST_ALL_NO_LIB )
set( Boost_USE_STATIC_LIBS ON )

find_package(abb_libegm REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread program_options)
find_package(orocos_kdl REQUIRED)
find_package(YARP 3.9 REQUIRED COMPONENTS os)
#find_package(PythonLibs REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ROBOTICSLAB_VISION REQUIRED)
###########
## Build ##
###########

include_directories(
 ${abb_libegm_INCLUDE_DIRS}
 ${CMAKE_SOURCE_DIR}/include
 ${orocos_kdl_INCLUDE_DIRS}
 ${PYTHON_INCLUDE_DIRS}
 #${YAML_CPP_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/trajectory_generator.cpp src/robot.cpp )
#set_target_properties(${PROJECT_NAME}_joint_trajectory_node PROPERTIES OUTPUT_NAME a1_joint_trajectory_node PREFIX "")
target_link_libraries(${PROJECT_NAME}_node ${abb_libegm_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES} YARP::YARP_os YARP::YARP_init matplotlib_cpp::matplotlib_cpp yaml-cpp::yaml-cpp ROBOTICSLAB::VisionIDL)


