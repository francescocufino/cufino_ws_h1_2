list(INSERT CMAKE_PREFIX_PATH 0 "/usr")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost COMPONENTS program_options thread system REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)
link_libraries(libboost_program_options.a)

add_executable(hand_service src/hand_service.cpp)

add_library(h1_2_motion STATIC
    src/arm_motion.cpp
    src/hand_motion.cpp
    src/locomotion.cpp
    src/hand_service.cpp
    src/h1_2_kdl.cpp
)

target_link_libraries(h1_2_motion Boost::thread Boost::system ${orocos_kdl_LIBRARIES} kdl_parser)

target_include_directories(h1_2_motion PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${orocos_kdl_INCLUDE_DIRS}
    ${kdl_parser_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)


