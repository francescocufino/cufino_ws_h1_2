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
  };

Whole_body_motion::Whole_body_motion(){

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
  std::array<float, 12> target_pos_hands_opened; target_pos_hands_closed.fill(1);

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











