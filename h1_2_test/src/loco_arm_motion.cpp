#include <array>
#include <thread>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <memory>

//high level (for locomotion)
#include "locomotion.h"
#include "arm_motion.h"

class Loco_arm_motion : public Locomotion, public Arm_motion{
  private:

  public:
    Loco_arm_motion();
  };

Loco_arm_motion::Loco_arm_motion(){

}


int main(int argc, char const *argv[]) {
  std:: cout<<"Netw interface: " <<  argv[1] << "\n";
  //unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::ChannelFactory::Instance()->Init(0);

  std::array<float, 15> target_pos_lateral_lift = {0.f, M_PI_2,  0.f, M_PI_2, 0, 0, 0,
                                    0.f, -M_PI_2, 0.f, M_PI_2, 0, 0, 0,
                                    0.f};

  std::array<float, 15> target_pos_arms_forward = {-M_PI/(12.f), 0.3,  0.f, M_PI/(12.f), 0, 0, 0,
                                    -M_PI/(12.f), -0.3, 0.f, M_PI/(12.f), 0, 0, 0,
                                    0.f};

  Loco_arm_motion loco;

  loco.initialize_arms();
  loco.walk(0.1, 0, 0);
  loco.move_arms_polynomial(target_pos_lateral_lift, 3);
  loco.stop_walk();
  loco.stop_arms();
 
  //loco.walk_temporized(0.2, 0, 0, 2);
  

    
}











