#ifndef _LOCOMOTION_
#define _LOCOMOTION_

#include <iostream>


//high level (for locomotion)
#include <unitree/robot/h1/loco/h1_loco_api.hpp>
#include <unitree/robot/h1/loco/h1_loco_client.hpp>

    
class Locomotion{
  private:
    std::shared_ptr<unitree::robot::h1::LocoClient> client;
   
  public:
    Locomotion();
    void walk_temporized(float, float, float, float);
    void walk(float, float, float);
    void stop_walk();
  };

  #endif