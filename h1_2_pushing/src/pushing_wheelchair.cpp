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
class Pushing : /*public Locomotion,*/ public Arm_motion, public Hand_motion{
  private:
    H1_2_kdl h1_2;

  public:
    Pushing();
    std::array<float, 12> get_est_forces();


  };

Pushing::Pushing(){
  
}

std::array<float, 12> Pushing::get_est_forces(){
  return h1_2.compute_ee_forces(Arm_motion::get_angles(), Arm_motion::get_est_torques());
}





int create_UDP_socket(int & sockfd, struct sockaddr_in & serverAddr){  
    
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
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

bool send_wrenches_UDP(std::array<float, 12> f_est, int sockfd, struct sockaddr_in serverAddr){
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
    if (sentBytes < 0)
    {
        perror("Failed to send UDP packet");
        return false;
    }
    return true;

}


int main(int argc, char const *argv[]) {
    std::array<float, 12UL> f_est = {0,0,0,0,0,0,0,0,0,0,0,0};
    int sockfd;
    struct sockaddr_in serverAddr;
    create_UDP_socket(sockfd, serverAddr);


    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    Pushing h1_pushing;
    h1_pushing.initialize_arms();

    while (true){
       f_est = h1_pushing.get_est_forces();
       send_wrenches_UDP(f_est, sockfd, serverAddr);
       usleep(SEND_INTERVAL);
    }

    close(sockfd);
    return 0;
}















