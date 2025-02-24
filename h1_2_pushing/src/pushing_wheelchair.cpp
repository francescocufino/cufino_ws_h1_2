/**
 * @file pushing_wheelchair.cpp
 * @brief This contains the user script for wheelchair pushing.
 */

#include "h1_2_kdl.h"

//high level (for locomotion, arm motion, hand motion)
#include "locomotion.h"
#include "arm_motion.h"
#include "hand_motion.h"

#include <boost/thread.hpp>

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

const int JOINT_DIM = 15;
const int WRENCH_DIM = 6;
const int LEFT_ARM_JOINTS = 7;
const int RIGHT_ARM_JOINTS = 7;
const int WAIST_INDEX = 14;


/**
 * @class Pushing
 * @brief A class for pushing exploiting locomotion, arm motion, hand motion.
 */ 
class Pushing : public Locomotion, public Arm_motion, public Hand_motion{
  private:
    H1_2_kdl h1_2;
    bool grasped = false;

    //stream data UDP
    int sockfd;
    struct sockaddr_in serverAddr;
    
    //Data storage
    std::vector<std::vector<float>> leftArmTorques;
    std::vector<std::vector<float>> rightArmTorques;
    std::vector<std::vector<float>> leftArmAngles;
    std::vector<std::vector<float>> rightArmAngles;
    std::vector<std::vector<float>> armWrenchLeft;
    std::vector<std::vector<float>> armWrenchRight;
    std::vector<float> waistAngles;


  public:
    Pushing();
    std::array<float, 12> get_est_forces();
    void test_pushing();
    void test_force_est();
    void test_COM_motion();
    void stop_motion();

    //stream data UDP
    int create_UDP_socket();
    bool send_wrenches_UDP(std::array<float, 12> f_est);
    
    //Save data to csv
    void store_data();
    void writeDataToCSV(const std::string& filename, const std::vector<std::vector<float>>& data);

  };

Pushing::Pushing(){
  
}

std::array<float, 12> Pushing::get_est_forces(){
  return h1_2.compute_ee_forces(Arm_motion::get_angles(), Arm_motion::get_est_torques());
}


void Pushing::test_pushing(){
  // std::array<float, 15> arm_pos_pushing_test = {-0.0316936, 0.240571, -0.109105, 0.394378, -0.0633168, 0.361801, -0.133496,
  //                                   -0.0316936, -0.240571, 0.109105, 0.394378, 0.0633168, 0.361801, 0.133496,
  //                                   0.f};
  std::array<float, 15> arm_pos_pushing_test = {-0.0237002, 0.21376, -0.121089, 0.41662, -0.0699079, 0.359728, -0.142376,
                                              -0.0145812, -0.210837, 0.12468, 0.440892, 0.0750732, 0.37466, 0.150034, 
                                              -0.000562433,};
  
  std::array<float, 15> arm_pos_2_pushing_test = {0.5, 0.240571, -0.109105, 0.394378, -0.0633168, 0.361801, -0.133496,
                                    0.5, -0.240571, 0.109105, 0.394378, 0.0633168, 0.361801, 0.133496,
                                    0.f};
  std::array<float, 12> hand_opened_pos; hand_opened_pos.fill(1);
  std::array<float, 12> hand_closed_pos; hand_closed_pos.fill(0);
  move_hands(hand_opened_pos);
  initialize_arms();
  move_arms_polynomial(arm_pos_pushing_test, 3);
  std::cout << "Press ENTER to grasp ...";
  std::cin.get();
  move_hands(hand_closed_pos);
  grasped = true;
  std::cout << "Press ENTER to start walking ...";
  std::cin.get();
  walk(0.2, 0, 0);
  std::cout << "Press ENTER to stop walking ...";
  std::cin.get();
  stop_walk();
  std::cout << "Press ENTER to release grasp and stop arms ...";
  std::cin.get();
  move_hands(hand_opened_pos);
  grasped = false;
  move_arms_polynomial(arm_pos_2_pushing_test, 3);
  //stop_arms();
}


void Pushing::stop_motion(){
  stop_walk();
  stop_arms();
  std::array<float, 12> hand_opened_pos; hand_opened_pos.fill(1);
  move_hands(hand_opened_pos);
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

void Pushing::test_COM_motion(){
  std::array<float, 15> arm_pos_COM_test = {-1.2, 0, 0, 1.43903, 0, 0, 0,
                                            -1.2, 0, 0, 1.43903, 0, 0, 0, 
                                            0};
  initialize_arms();
  move_arms_polynomial(arm_pos_COM_test, 3);
  std::cout << "Press ENTER to start rotate ...";
  std::cin.get();
  walk(0,0,0.3);
  std::cout << "Press ENTER to stop rotate ...";
  std::cin.get();
  stop_walk();
  

}

void Pushing::store_data(){
	while(!grasped) {usleep(100000);}
	std::cout << "Grasping done. Starting data storage...\n";
  std::array<float, 15> jointData;
  std::array<float, 15> torqueData;
  std::array<float, 12> wrenchData;

	while(grasped){
		//Get data
    jointData = Arm_motion::get_angles();
    torqueData = Arm_motion::get_est_torques();
    wrenchData = h1_2.compute_ee_forces(jointData, torqueData);

    //Save data in data structure
    leftArmAngles.push_back(std::vector<float>(jointData.begin(), jointData.begin() + LEFT_ARM_JOINTS));
    rightArmAngles.push_back(std::vector<float>(jointData.begin() + LEFT_ARM_JOINTS, jointData.begin() + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS));
    waistAngles.push_back(jointData[WAIST_INDEX]);
    leftArmTorques.push_back(std::vector<float>(torqueData.begin(), torqueData.begin() + LEFT_ARM_JOINTS));
    rightArmTorques.push_back(std::vector<float>(torqueData.begin() + LEFT_ARM_JOINTS, torqueData.begin() + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS));
    armWrenchLeft.push_back(std::vector<float>(wrenchData.begin(), wrenchData.begin() + WRENCH_DIM));
    armWrenchRight.push_back(std::vector<float>(wrenchData.begin() + WRENCH_DIM, wrenchData.begin() + WRENCH_DIM + WRENCH_DIM));
		
    usleep(100000);
	}
  std::cout << "Grasping released. Starting data writing...\n";
  
  // Write everything to files when grasping has been releaseed
  writeDataToCSV("left_arm_angles.csv", leftArmAngles);
  writeDataToCSV("right_arm_angles.csv", rightArmAngles);
  writeDataToCSV("left_arm_torques.csv", leftArmTorques);
  writeDataToCSV("right_arm_torques.csv", rightArmTorques);
  writeDataToCSV("left_arm_wrench.csv", armWrenchLeft);
  writeDataToCSV("right_arm_wrench.csv", armWrenchRight);

  std::ofstream waistFile("waist_angles.csv");
  if (waistFile.is_open()) {
      for (const auto& angle : waistAngles) {
          waistFile << angle << std::endl;
      }
      waistFile.close();
  }

  std::cout << "Data writing complete!\n";
  
}

void Pushing::writeDataToCSV(const std::string& filename, const std::vector<std::vector<float>>& data) {
  std::ofstream file(filename);
  if (file.is_open()) {
      for (const auto& row : data) {
          for (size_t i = 0; i < row.size(); i++) {
              file << row[i];
              if (i < row.size() - 1) file << ",";
          }
          file << std::endl;
      }
      file.close();
  } else {
      std::cerr << "Unable to open file: " << filename << std::endl;
  }
}


Pushing* global_pushing_instance;

void handleSigint(int sig) {
    std::cout << "\nInterrupt signal (" << sig << ") received. Stopping robot motion...\n";
    global_pushing_instance->stop_motion();
    exit(0); // Terminate the program
}



int main(int argc, char const *argv[]) {
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  Pushing h1_pushing;
  
  //Stop everything when ctrl+c is pressed
  global_pushing_instance = & h1_pushing;
  if (signal(SIGINT, handleSigint) == SIG_ERR) {
    std::cerr << "Error setting up signal handler." << std::endl;
    return 1;
  }

  //TEST FORCE ESTIMATION
  //h1_pushing.test_force_est();

  //TEST PUSHING WHEELCHAIR storing data
  std::shared_ptr<boost::thread> thread1 = std::make_shared<boost::thread>(boost::bind(&Pushing::test_pushing,&h1_pushing));  
  std::shared_ptr<boost::thread> thread2 = std::make_shared<boost::thread>(boost::bind(&Pushing::store_data,&h1_pushing));  
  thread1->join();
  thread2->join();
  

  //TEST COM MOTION
  //h1_pushing.test_COM_motion();
  

  //READ POSITION
  // std::array<float, 15> q; q=h1_pushing.Arm_motion::get_angles();
  // for (int j = 0; j < q.size(); ++j) {
  //   std::cout << q.at(j) << ", ";
  // }

  return 0;
}















