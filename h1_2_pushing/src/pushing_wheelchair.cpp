/**
 * @file pushing_wheelchair.cpp
 * @brief This contains the user script for wheelchair pushing.
 */

#include "h1_2_kdl.h"

//high level (for locomotion, arm motion, hand motion)
#include "locomotion.h"
#include "arm_motion.h"
#include "hand_motion.h"

/**
 * @class Pushing
 * @brief A class for pushing exploiting locomotion, arm motion, hand motion.
 */ 
class Pushing : public Locomotion, public Arm_motion, public Hand_motion{
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



int main(int argc, char const *argv[]) {

  //unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::ChannelFactory::Instance()->Init(0);
  Pushing h1_2_pushing;
  std::array<float, 12> f_ee = h1_2_pushing.get_est_forces();

}











