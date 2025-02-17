find_package(unitree_sdk2 REQUIRED)
find_package(Boost COMPONENTS program_options thread system REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)



set(H1_2_MOTION_PATH ./../h1_2_motion)

add_executable(pushing_wheelchair src/pushing_wheelchair.cpp src/h1_2_kdl.cpp ${H1_2_MOTION_PATH}/src/locomotion.cpp ${H1_2_MOTION_PATH}/src/arm_motion.cpp ${H1_2_MOTION_PATH}/src/hand_motion.cpp)
target_link_libraries(pushing_wheelchair unitree_sdk2 Boost::thread Boost::system ${orocos_kdl_LIBRARIES} kdl_parser)
target_include_directories(pushing_wheelchair PRIVATE ./../include ${orocos_kdl_INCLUDE_DIRS} ${kdl_parser_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})



