find_package(unitree_sdk2 REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(whole_body_motion src/whole_body_motion.cpp src/locomotion.cpp src/arm_motion.cpp src/hand_motion.cpp)
target_link_libraries(whole_body_motion unitree_sdk2)
target_include_directories(whole_body_motion PRIVATE include)


include_directories(
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
)

link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)
link_libraries(libboost_program_options.a)

add_executable(hand_service src/hand_service.cpp)
add_executable(hand_motion src/hand_motion.cpp)

