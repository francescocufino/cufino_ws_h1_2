/**
 * @file whole_body_motion.cpp
 * @brief This contains the user script for whole body motion.
 */

#include <array>
#include <thread>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <memory>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

//high level (for locomotion, arm motion, hand motion)
#include "locomotion.h"
#include "arm_motion.h"
#include "hand_motion.h"

/**
 * @class Whole_body_motion
 * @brief A class for locomotion, arm motion, hand motion usage.
 */ 
class Whole_body_motion : public Locomotion, public Arm_motion, public Hand_motion{
  private:

  public:
    Whole_body_motion();
    void wave_arm();
    void wave_hand();
    void wave_arm_hand();
  };

Whole_body_motion::Whole_body_motion(){

}

void Whole_body_motion::wave_arm(){
  std::array<float, 15> wave_1_pos = {0.f, M_PI_2,  0.f, M_PI_2, 0, 0, 0,
                                    0.f, -M_PI_2, 0.f, M_PI_2, 0, 0, 0,
                                    0.f};
  std::array<float, 15> wave_2_pos = {0.f, M_PI_2,  0.f, M_PI_2, 0, 0, 0,
                                    0.f, -M_PI_2, 0.f, M_PI_2, 0, 0, 0,
                                    0.f};
  initialize_arms();
  while(1){
    move_arms_polynomial(wave_1_pos, 5);
    move_arms_polynomial(wave_2_pos, 5);
  }                                  

}

void Whole_body_motion::wave_hand(){
  std::array<float, 12> wave_closed_pos; wave_closed_pos.fill(0);
  std::array<float, 12> wave_opened_pos; wave_opened_pos.fill(1);
  initialize_arms();
  while(1){
    move_hands(wave_closed_pos);
    move_hands(wave_opened_pos);
  }                                  

}

void Whole_body_motion::wave_arm_hand(){
  boost::thread* thread1 = new boost::thread(boost::bind(&Whole_body_motion::wave_arm, this));
  //boost::thread thread2(boost::bind(&Whole_body_motion::wave_arm, this));
  // Wait for both threads to finish
  thread1->join();
  //thread2.join();
}



int main(int argc, char const *argv[]) {
  std:: cout<<"Netw interface: " <<  argv[1] << "\n";
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  //unitree::robot::ChannelFactory::Instance()->Init(0);

  std::array<float, 15> target_pos_arms_lateral_lift = {0.f, M_PI_2,  0.f, M_PI_2, 0, 0, 0,
                                    0.f, -M_PI_2, 0.f, M_PI_2, 0, 0, 0,
                                    0.f};

  std::array<float, 15> target_pos_arms_forward = {-M_PI/(12.f), 0.3,  0.f, M_PI/(12.f), 0, 0, 0,
                                    -M_PI/(12.f), -0.3, 0.f, M_PI/(12.f), 0, 0, 0,
                                    0.f};

  std::array<float, 12> target_pos_hands_closed; target_pos_hands_closed.fill(0);
  std::array<float, 12> target_pos_hands_opened; target_pos_hands_opened.fill(1);

  Whole_body_motion h1_wbm;

  h1_wbm.initialize_arms();
  h1_wbm.walk(0.1, 0, 0);
  h1_wbm.move_arms_polynomial(target_pos_arms_lateral_lift, 3);
  h1_wbm.stop_walk();
  h1_wbm.stop_arms();
  h1_wbm.move_hands(target_pos_hands_closed);
  h1_wbm.move_hands(target_pos_hands_opened);

  //loco.walk_temporized(0.2, 0, 0, 2);
  

    
}











