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
  client = std::make_shared<unitree::robot::h1::LocoClient>();
  client->Init();
  client->SetTimeout(10.f);
  client->Start();
}

void Locomotion::walk_temporized(float vx, float vy, float vyaw, float duration){
  client->SetVelocity(vx, vy, vyaw, duration);
}

void Locomotion::walk(float vx, float vy, float vyaw){
  client->Move(vx, vy, vyaw,1);
}

void Locomotion::stop_walk(){
  client->StopMove();
}











