/**
 * @file pushing_wheelchair.cpp
 * @brief This contains the user script for wheelchair pushing.
 */

#include "h1_2_kdl.h"

//high level (for locomotion, arm motion, hand motion)
#include "locomotion.h"
#include "arm_motion.h"
#include "hand_motion.h"

//UDP socket for streaming data
#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

#define UDP_IP "192.168.2.2"
#define UDP_PORT 5005
#define SEND_INTERVAL 100000 // 100ms


/**
 * @class Pushing
 * @brief A class for pushing exploiting locomotion, arm motion, hand motion.
 */ 
class Pushing : public Locomotion, public Arm_motion, public Hand_motion{
  private:
    H1_2_kdl h1_2;

    //stream data UDP
    int sockfd;
    struct sockaddr_in serverAddr;

  public:
    Pushing();
    std::array<float, 12> get_est_forces();
    void test_pushing();
    void test_force_est();

    //stream data UDP
    int create_UDP_socket();
    bool send_wrenches_UDP(std::array<float, 12> f_est);


  };

Pushing::Pushing(){
  
}

std::array<float, 12> Pushing::get_est_forces(){
  return h1_2.compute_ee_forces(Arm_motion::get_angles(), Arm_motion::get_est_torques());
}


void Pushing::test_pushing(){
  std::array<float, 15> arm_pos_pushing_test = {0.060183, 0.0551721, 0.000710487, 1.18461, 0.0328448, 0.0337567, -0.0323732,
                                              0.0184591, -0.000766754, -0.0243986, -0.0613762, -0.064399, 0.0569582, -0.0392926,
                                              -0.00184074};//to define
  std::array<float, 12> hand_opened_pos; hand_opened_pos.fill(1);
  std::array<float, 12> hand_closed_pos; hand_closed_pos.fill(0);
  initialize_arms();
  move_arms_polynomial(arm_pos_pushing_test, 3);
  std::cout << "Press ENTER to grasp ...";
  std::cin.get();
  move_hands(hand_closed_pos);
  std::cout << "Press ENTER to start walking ...";
  std::cin.get();
  walk(0.1, 0, 0);
  std::cout << "Press ENTER to stop walking ...";
  std::cin.get();
  stop_walk();
  std::cout << "Press ENTER to release grasp and stop arms ...";
  std::cin.get();
  move_hands(hand_opened_pos);
  stop_arms();
}





int Pushing::create_UDP_socket(){  
    
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0){
        perror("Socket creation failed");
        return EXIT_FAILURE;
    }

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(UDP_PORT);
    serverAddr.sin_addr.s_addr = inet_addr(UDP_IP);

    std::cout << "Sending UDP JSON data to " << UDP_IP << ":" << UDP_PORT << std::endl;
    return 0;

}

bool Pushing::send_wrenches_UDP(std::array<float, 12> f_est){
    // Format data as JSON
    std::ostringstream dataStream;
    dataStream << "{"
                << "\"wrench_left_arm\": {\"fx\": " << f_est.at(0) << ", \"fy\": " << f_est.at(1) << ", \"fz\": " << f_est.at(2)
                << ", \"tx\": " << f_est.at(3) << ", \"ty\": " << f_est.at(4) << ", \"tz\": " << f_est.at(5) << "},"
                << "\"wrench_right_arm\": {\"fx\": " << f_est.at(6) << ", \"fy\": " << f_est.at(7) << ", \"fz\": " << f_est.at(8)
                << ", \"tx\": " << f_est.at(9) << ", \"ty\": " << f_est.at(10) << ", \"tz\": " << f_est.at(11) << "}"
                << "}";

    std::string message = dataStream.str();

    ssize_t sentBytes = sendto(sockfd, message.c_str(), message.length(), 0,
                                (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if (sentBytes < 0){
        perror("Failed to send UDP packet");
        return false;
    }
    return true;

}

void Pushing::test_force_est(){
  std::array<float, 12UL> f_est = {0,0,0,0,0,0,0,0,0,0,0,0};
  std::array<float, 15> arm_pos_force_test = {0.060183, 0.0551721, 0.000710487, 1.18461, 0.0328448, 0.0337567, -0.0323732,
                                            0.0184591, -0.000766754, -0.0243986, -0.0613762, -0.064399, 0.0569582, -0.0392926,
                                            -0.00184074};

  initialize_arms();
  move_arms_polynomial(arm_pos_force_test, 3);

  create_UDP_socket();  
  while (true){
      f_est = get_est_forces();
      send_wrenches_UDP(f_est);
      usleep(SEND_INTERVAL);
  }

  close(sockfd);

}




int main(int argc, char const *argv[]) {
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  Pushing h1_pushing;

  //TEST FORCE ESTIMATION
  //h1_pushing.test_force_est();
  h1_pushing.print_foot_force();

  //TEST PUSHING WHEELCHAIR
  //h1_pushing.test_pushing();
  

  //READ POSITION
  // std::array<float, 15> q; q=h1_pushing.Arm_motion::get_angles();
  // for (int j = 0; j < q.size(); ++j) {
  //   std::cout << q.at(j) << ", ";
  // }

  return 0;
}















