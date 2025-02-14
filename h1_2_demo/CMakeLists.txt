find_package(unitree_sdk2 REQUIRED)
find_package(Boost COMPONENTS program_options thread system REQUIRED)

set(H1_2_MOTION_PATH ./../h1_2_motion)

add_executable(whole_body_motion src/whole_body_motion.cpp ${H1_2_MOTION_PATH}/src/locomotion.cpp ${H1_2_MOTION_PATH}/src/arm_motion.cpp ${H1_2_MOTION_PATH}/src/hand_motion.cpp)
target_link_libraries(whole_body_motion unitree_sdk2 Boost::thread Boost::system)
target_include_directories(whole_body_motion PRIVATE ./../include)



