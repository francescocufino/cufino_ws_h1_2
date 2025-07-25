/**
 * @file locomotion.h
 * @brief Provides functions and classes for basic high level locomotion.
 * 
 * This header file contains the declaration of functions and classes 
 * that perform basic high level locomotion, like walking with a desired velocity
 * for a desired duration
 * 
 * @author Francesco Cufino
 * @date 09/02/2024
 * @version 1.0
 */

#ifndef _LOCOMOTION_
#define _LOCOMOTION_

#include <iostream>


//high level (for locomotion)
#include <unitree/robot/h1/loco/h1_loco_api.hpp>
#include <unitree/robot/h1/loco/h1_loco_client.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>


/**
 * @class Locomotion
 * @brief A class for basic high level locomotion.
 */    
class Locomotion{
  private:
    std::shared_ptr<unitree::robot::h1::LocoClient> client;

    const std::string kTopicState = "rt/lowstate";
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> low_state_subscriber;
    std::shared_ptr<unitree_hg::msg::dds_::LowState_> state_msg;
    bool first_cb=false;
    bool stop = false;

   
  public:
    Locomotion();
    /**
     * @brief Perform a walk with desired velocity and duration.
     * 
     * @param vx Forward velocity in m/s
     * @param vy Lateral velocity in m/s
     * @param vyaw Yaw velocity in m/s
     * @param duration Duration in s 
     */
    void walk_temporized(float vx, float vy, float vyaw, float duration);
    /**
     * @brief Perform a continuous walk with desired velocity.
     * 
     * @param vx Forward velocity in m/s
     * @param vy Lateral velocity in m/s
     * @param vyaw Yaw velocity in m/s
     */
    void walk(float vx, float vy, float vyaw);
    /**
     * @brief Stop the walk.
     */
    void stop_walk();

    /**
     * @brief Get accelerometer
     */
    std::array <float, 3> get_accelerometer(){return state_msg->imu_state().accelerometer();} 

    bool get_stop_status(){return stop;}

  };

  #endif