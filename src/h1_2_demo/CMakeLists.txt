add_executable(whole_body_motion src/whole_body_motion.cpp)
target_link_libraries(whole_body_motion PRIVATE h1_2_motion)

add_executable(motion_test src/motion_test.cpp)
target_link_libraries(motion_test PRIVATE h1_2_motion)


