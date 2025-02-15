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
#include <fstream>

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
    bool start_waving = false;

  public:
    Whole_body_motion();
    void wave_arm();
    void wave_hand();
    void wave_arm_hand();
    void shake_hand();
    void fist_bump();

  };

Whole_body_motion::Whole_body_motion(){

}

void Whole_body_motion::wave_arm(){
  std::array<float, 15> wave_1_pos = {-0.0923371 , 0.0433179 , 0.246224 , 1.45887 , 0.0195663 , 0.109725 , 0.00963151 ,
                                    -0.4 , -0.0855193 , -0.0869789 , -0.714452 ,-1.47578 , 0.406538 , -0.312845 , 
                                    -0.000945929};
  std::array<float, 15> wave_2_pos = {-0.0923371 , 0.0433179 , 0.246224 , 1.45887 , 0.0195663 , 0.109725 , 0.00963151 ,
                                    -0.4 , -0.0855193 , -0.0869789 , -0.714452 ,-1.47578 , -0.406538 , -0.312845 , 
                                    -0.000945929};

  initialize_arms();
  while(1){
    //Start iterative motion from pos 1 to pos 2 and viceversa, and set start waving 
    move_arms_polynomial(wave_1_pos, 2);
    if(!start_waving) {start_waving=true;}
    move_arms_polynomial(wave_2_pos, 2);
  }                                  

}

void Whole_body_motion::wave_hand(){
  std::array<float, 12> wave_closed_pos = {0,0,0,0,0,0,
                                          1,1,1,1,1,1};
  std::array<float, 12> wave_opened_pos; wave_opened_pos.fill(1);
  while(1){
    //Start iterative motion from close to open pos and viceversa
    move_hands(wave_closed_pos);
    move_hands(wave_opened_pos);
  }                                  

}

void Whole_body_motion::wave_arm_hand(){
  //Start wave arm
  std::shared_ptr<boost::thread> thread1 = std::make_shared<boost::thread>(boost::bind(&Whole_body_motion::wave_arm, this));  
  //When arm started waving, then start wave hand simultaneously
  while(!start_waving){usleep(10000);}
  std::shared_ptr<boost::thread> thread2 = std::make_shared<boost::thread>(boost::bind(&Whole_body_motion::wave_hand, this));
  thread1->join();
  thread2->join();
}

void Whole_body_motion::shake_hand(){
  //Joint order: right [pinky, ring, middle, index, thumb_bend, thumb_rotation],
  //   * left [pinky, ring, middle, index, thumb_bend, thumb_rotation]
  std::array<float, 12> hand_shake_opened_pos; hand_shake_opened_pos.fill(1);
  std::array<float, 12> hand_shake_closed_pos = {0.5,0.6,0.7,0.8,0.7,0.7,
                                          1,1,1,1,1,1};
  std::array<float, 15> arm_target_pos = {0.00200272, 0.065944, -0.0749669, 1.43511, 0.273104, 0.0142946, 0.00735462,
                                                -0.2, -0.00331497, -0.0423436, 0.459666, 0.0179806, -0.483458, -0.0203335, 
                                                0.0416987};
  initialize_arms();                                              
  move_hands(hand_shake_opened_pos);
  move_arms_polynomial(arm_target_pos, 3);
  std::cout << "Press ENTER to shake hand ...";
  std::cin.get();
  move_hands(hand_shake_closed_pos);
  usleep(3000000);
  move_hands(hand_shake_opened_pos);
  stop_arms();
}

void Whole_body_motion::fist_bump(){
   //Joint order: right [pinky, ring, middle, index, thumb_bend, thumb_rotation],
  //   * left [pinky, ring, middle, index, thumb_bend, thumb_rotation]
  std::array<float, 12> hand_fist_bump_opened_pos; hand_fist_bump_opened_pos.fill(1);
  std::array<float, 12> hand_fist_bump_closed_pos = {0,0,0,0,0,0,
                                          1,1,1,1,1,1};
  std::array<float, 15> arm_pos_fist_bump_1 = {-0.0811305, 0.120383 , -0.297125 , 1.41618 , -0.103212 , -0.104181 , -0.0382934 ,
                                                 -0.443636, 0.0161242, 0.0254326, 0.388924, -1.5967, -0.136587, -0.0239768,
                                                  -0.164673};
  std::array<float, 15> arm_pos_fist_bump_2 = {-0.0811305, 0.120383 , -0.297125 , 1.41618 , -0.103212 , -0.104181 , -0.0382934 ,
                                                 0.152418 , 0.000162125 , 0.251394 , -0.227234 , -1.57687 , -0.0975542 , -0.0243244 , 
                                                 -0.17017};
  initialize_arms();                                              
  move_hands(hand_fist_bump_closed_pos);
  move_arms_polynomial(arm_pos_fist_bump_1, 1);
  std::cout << "Press ENTER to fist bump ...";
  std::cin.get();

  std::shared_ptr<boost::thread> thread1 = std::make_shared<boost::thread>(boost::bind(&Whole_body_motion::move_arms_polynomial, this, arm_pos_fist_bump_2, 1));  
  std::shared_ptr<boost::thread> thread2 = std::make_shared<boost::thread>(boost::bind(&Whole_body_motion::move_hands, this, hand_fist_bump_opened_pos));

  thread1->join();
  thread2->join();

  stop_arms();

}




int main(int argc, char const *argv[]) {

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);


  Whole_body_motion h1_wbm;

  //WHOLE BODY MOTION TEST
  // std::array<float, 15> target_pos_arms_lateral_lift = {0.f, M_PI_2,  0.f, M_PI_2, 0, 0, 0,
  //                                  0.f, -M_PI_2, 0.f, M_PI_2, 0, 0, 0,
  //                                  0.f};

  // std::array<float, 15> target_pos_arms_forward = {-M_PI/(12.f), 0.3,  0.f, M_PI/(12.f), 0, 0, 0,
  //                                  -M_PI/(12.f), -0.3, 0.f, M_PI/(12.f), 0, 0, 0,
  //                                  0.f};

  // std::array<float, 12> target_pos_hands_closed; target_pos_hands_closed.fill(0);
  // std::array<float, 12> target_pos_hands_opened; target_pos_hands_opened.fill(1);

  
  // h1_wbm.initialize_arms();
  // h1_wbm.walk(0.1, 0, 0);
  // h1_wbm.move_arms_polynomial(target_pos_arms_lateral_lift, 3);
  // h1_wbm.stop_walk();
  // h1_wbm.stop_arms();



  //PRINT INIT POS
  //std::array<float, 15> q; q=h1_wbm.Arm_motion::get_angles();
  //for (int j = 0; j < q.size(); ++j) {
  //    //std::cout << "q" << j << ": " << q.at(j) << ' ';
  //  }


  //DEMO ROUTINES
  std::ifstream file("../h1_2_demo/config/routine.txt");
  std::string command;
  if(!file){std::cerr<<"Error opening file\n"; return 1;}
  file >>command;

  if(command == "shake"){
    h1_wbm.shake_hand();
  }
  else if(command == "wave"){
    h1_wbm.wave_arm_hand();
  }
  else if(command == "fist_bump"){
    h1_wbm.fist_bump();
  }
  else{
    h1_wbm.wave_arm_hand();
  }


}











