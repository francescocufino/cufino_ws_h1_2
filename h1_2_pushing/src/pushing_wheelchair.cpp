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
  };

Pushing::Pushing(){
  
}


//std::shared_ptr<boost::thread> thread1 = std::make_shared<boost::thread>(boost::bind(&Whole_body_motion::move_arms_polynomial, this, arm_pos_fist_bump_2, 1));  
//  std::shared_ptr<boost::thread> thread2 = std::make_shared<boost::thread>(boost::bind(&Whole_body_motion::move_hands, this, hand_fist_bump_opened_pos));

//  thread1->join();
//  thread2->join();


int main(int argc, char const *argv[]) {

  //unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::ChannelFactory::Instance()->Init(0);

  Pushing h1_pushing;
  
/*

  //PRINT IN LOOP EST TORQUES
  std::array<float, 15> tau_est;
  while(1){
    tau_est = h1_pushing.get_est_torques();
    for (int j = 0; j < tau_est.size(); ++j) {
     std::cout << "tau" << j << ": " << tau_est.at(j) << ' ';
   }
   std::cout << std::endl << std::endl;

  }

*/

}











