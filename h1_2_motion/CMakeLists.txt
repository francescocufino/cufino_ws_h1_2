find_package(unitree_sdk2 REQUIRED)
find_package(Boost COMPONENTS program_options thread system REQUIRED)



include_directories(
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  ./../include
)

link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)
link_libraries(libboost_program_options.a)

add_executable(hand_service src/hand_service.cpp)


