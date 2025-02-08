add_executable(loco_arm_motion src/loco_arm_motion.cpp src/locomotion.cpp src/arm_motion.cpp)
target_link_libraries(loco_arm_motion unitree_sdk2)
target_include_directories(loco_arm_motion PRIVATE include)

