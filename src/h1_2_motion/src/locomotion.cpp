/**
 * @file locomotion.cpp
 * @brief This contains the implementation of functions for basic high level locomotion.
 * 
 * @author Francesco Cufino
 * @date 09/02/2024
 * @version 1.0
 */


#include "locomotion.h"

Locomotion::Locomotion(){
  state_msg = std::make_shared<unitree_hg::msg::dds_::LowState_>();
  client = std::make_shared<unitree::robot::h1::LocoClient>();
  client->Init();
  client->SetTimeout(10.f);
  client->Start();

  low_state_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicState));
  low_state_subscriber->InitChannel(
      [&](const void *msg_ptr) {
          auto s = static_cast<const unitree_hg::msg::dds_::LowState_*>(msg_ptr);
          *state_msg = *s;  // Dereferencing shared_ptr to copy data
          if(!first_cb){first_cb = true;}
      }, 
      1
  );
}

void Locomotion::walk_temporized(float vx, float vy, float vyaw, float duration){
  client->SetVelocity(vx, vy, vyaw, duration);
}

void Locomotion::walk(float vx, float vy, float vyaw){
  client->Move(vx, vy, vyaw,1);
}

void Locomotion::stop_walk(){
  std::cout << "Stopping walk\n";
  stop = true;
  client->Move(0,0,0);
  client->StopMove();
}











