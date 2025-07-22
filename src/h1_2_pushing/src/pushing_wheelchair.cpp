/**
 * @file pushing_wheelchair.cpp
 * @brief This contains the user script for wheelchair pushing.
 */

#include "h1_2_kdl.h"
#include <algorithm>  // For std::copy
#include <iomanip>  // for std::fixed and std::setprecision

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
#define SEND_INTERVAL 10000 // 10ms




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
    std::vector<std::array<float, UPPER_LIMB_JOINTS_DIM+1>> torques;
    std::vector<std::array<float, UPPER_LIMB_JOINTS_DIM+1>> positions;
    std::vector<std::array<float, WRENCH_DIM+1>> wrench;


  public:
    Pushing();
    std::array<float, WRENCH_DIM> get_est_forces();
    void test_pushing();
    void test_force_est();
    void test_COM_motion();
    void stop_motion();

    //stream data UDP
    int create_UDP_socket();
    bool send_wrenches_UDP(std::array<float, WRENCH_DIM> f_est);
    
    //Save data to csv
    void store_data();
    template <size_t N>
    void writeCSV(const std::string &filename, const std::vector<std::array<float, N>> &data, const std::string &header);
  };

Pushing::Pushing(){
  
}

std::array<float, WRENCH_DIM> Pushing::get_est_forces(){
  return h1_2.compute_ee_forces(Arm_motion::get_angles(), Arm_motion::get_est_torques(), 0.1);
}


void Pushing::test_pushing(){
  // std::array<float, UPPER_LIMB_JOINTS_DIM> arm_pos_pushing_test = {-0.0316936, 0.240571, -0.109105, 0.394378, -0.0633168, 0.361801, -0.133496,
  //                                   -0.0316936, -0.240571, 0.109105, 0.394378, 0.0633168, 0.361801, 0.133496,
  //                                   0.f};
  std::array<float, UPPER_LIMB_JOINTS_DIM> arm_pos_pushing_test = {-0.0237002, 0.21376, -0.121089, 0.41662, -0.0699079, 0.359728, -0.142376,
                                              -0.0145812, -0.210837, 0.12468, 0.440892, 0.0750732, 0.37466, 0.150034}; 
                                              //-0.000562433,};
  
  std::array<float, UPPER_LIMB_JOINTS_DIM> arm_pos_2_pushing_test = {0.5, 0.240571, -0.109105, 0.394378, -0.0633168, 0.361801, -0.133496,
                                    0.5, -0.240571, 0.109105, 0.394378, 0.0633168, 0.361801, 0.133496};
                                    //0.f};
  std::array<float, HANDS_JOINTS_DIM> hand_opened_pos; hand_opened_pos.fill(1);
  std::array<float, HANDS_JOINTS_DIM> hand_closed_pos; hand_closed_pos.fill(0);
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
  std::array<float, HANDS_JOINTS_DIM> hand_opened_pos; hand_opened_pos.fill(1);
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

bool Pushing::send_wrenches_UDP(std::array<float, WRENCH_DIM> f_est){
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
  std::array<float, WRENCH_DIM> f_est = {0,0,0,0,0,0,0,0,0,0,0,0};
  std::array<float, UPPER_LIMB_JOINTS_DIM> arm_pos_force_test = {0.060183, 0.0551721, 0.000710487, 1.18461, 0.0328448, 0.0337567, -0.0323732,
                                            0.0184591, -0.000766754, -0.0243986, -0.0613762, -0.064399, 0.0569582, -0.0392926};
                                            //-0.00184074};

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
  std::array<float, UPPER_LIMB_JOINTS_DIM> arm_pos_COM_test = {-1.2, 0, 0, 1.43903, 0, 0, 0,
                                            -1.2, 0, 0, 1.43903, 0, 0, 0}; 
                                            //0};
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
  float time = 0;
	while(!grasped) {usleep(100000);}
	std::cout << "\nGrasping done. Starting data storage...\n";

  std::array<float, UPPER_LIMB_JOINTS_DIM> arr_torques; std::array<float, UPPER_LIMB_JOINTS_DIM+1> arr_torques_t;
  std::array<float, UPPER_LIMB_JOINTS_DIM> arr_positions; std::array<float, UPPER_LIMB_JOINTS_DIM+1> arr_positions_t;
  std::array<float, WRENCH_DIM> arr_wrench; std::array<float, WRENCH_DIM+1> arr_wrench_t; 

  

	while(grasped){ 
		//Get data and add the time 
    arr_positions = Arm_motion::get_angles();
    arr_torques = Arm_motion::get_est_torques();
    arr_wrench = h1_2.compute_ee_forces(arr_positions, arr_torques, 0.1);
    std::copy(arr_positions.begin(), arr_positions.end(), arr_positions_t.begin()); arr_positions_t.at(UPPER_LIMB_JOINTS_DIM) = time;   
    std::copy(arr_torques.begin(), arr_torques.end(), arr_torques_t.begin()); arr_torques_t.at(UPPER_LIMB_JOINTS_DIM) = time;  
    std::copy(arr_wrench.begin(), arr_wrench.end(), arr_wrench_t.begin()); arr_wrench_t.at(WRENCH_DIM) = time;  
   
    //push back the data with time
    positions.push_back(arr_positions_t);
    torques.push_back(arr_torques_t);
    wrench.push_back(arr_wrench_t);
  
    //dummy data
    // torques.push_back({0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f,time});
    // positions.push_back({1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f, 1.7f, 1.8f, 1.9f, 2.0f, 2.1f, 2.2f, 2.3f, 2.4f, 2.5f,time});
    // wrench.push_back({0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f, 1.1f, 1.2f, 1.3f, 1.4f, 1.5f, 1.6f,time});

    usleep(SEND_INTERVAL);
    time = float(time + float(SEND_INTERVAL) * 0.000001);
	}

  std::cout << "\nGrasping released. Starting data writing...\n";
  
  // Headers
  std::string torqueHeader = "left_arm_torques.tau_1,left_arm_torques.tau_2,left_arm_torques.tau_3,"
                           "left_arm_torques.tau_4,left_arm_torques.tau_5,left_arm_torques.tau_6,"
                           "left_arm_torques.tau_7,"
                           "right_arm_torques.tau_1,right_arm_torques.tau_2,right_arm_torques.tau_3,"
                           "right_arm_torques.tau_4,right_arm_torques.tau_5,right_arm_torques.tau_6,"
                           "right_arm_torques.tau_7,"
                           "waist_torque.tau_w,"
                           "time.t";

  std::string positionHeader = "left_arm_positions.q_1,left_arm_positions.q_2,left_arm_positions.q_3,"
                             "left_arm_positions.q_4,left_arm_positions.q_5,left_arm_positions.q_6,"
                             "left_arm_positions.q_7,"
                             "right_arm_positions.q_1,right_arm_positions.q_2,right_arm_positions.q_3,"
                             "right_arm_positions.q_4,right_arm_positions.q_5,right_arm_positions.q_6,"
                             "right_arm_positions.q_7,"
                             "waist_position.q_w,"
                             "time.t";

  std::string wrenchHeader =  "left_hand_wrench.f_x,left_hand_wrench.f_y,left_hand_wrench.f_z,"
                           "left_hand_wrench.tau_x,left_hand_wrench.tau_y,left_hand_wrench.tau_z,"
                           "right_hand_wrench.f_x,right_hand_wrench.f_y,right_hand_wrench.f_z,"
                           "right_hand_wrench.tau_x,right_hand_wrench.tau_y,right_hand_wrench.tau_z,"
                           "time.t";
  // Write to files
  writeCSV("../h1_2_pushing/output/torques.csv", torques, torqueHeader);
  writeCSV("../h1_2_pushing/output/positions.csv", positions, positionHeader);
  writeCSV("../h1_2_pushing/output/wrench.csv", wrench, wrenchHeader);

}

template <size_t N>
void Pushing::writeCSV(const std::string &filename, const std::vector<std::array<float, N>> &data, const std::string &header) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << filename << std::endl;
        return;
    }

    file << header << std::endl;
    //file << std::fixed << std::setprecision(1);
    for (const auto &row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1)
                file << ",";
        }
        file << std::endl;
    }
    

    file.close();
    std::cout << "Data written to " << filename << std::endl;
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
  //std::shared_ptr<boost::thread> thread2 = std::make_shared<boost::thread>(boost::bind(&Pushing::store_data,&h1_pushing));  
  thread1->join();
  //thread2->join();
  

  //TEST COM MOTION
  //h1_pushing.test_COM_motion();
  

  //READ POSITION
  // std::array<float, UPPER_LIMB_JOINTS_DIM> q; q=h1_pushing.Arm_motion::get_angles();
  // for (int j = 0; j < q.size(); ++j) {
  //   std::cout << q.at(j) << ", ";
  // }

  return 0;
}















