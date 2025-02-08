#ifndef _ARM_MOTION_
#define _ARM_MOTION_

#include <array>
#include <thread>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <memory>

//low level (for arms)
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

class Arm_motion{
  private:
      enum JointIndex {
      // Left leg
      kLeftHipYaw = 0,
      kLeftHipPitch = 1,
      kLeftHipRoll = 2,
      kLeftKnee = 3,
      kLeftAnkle = 4,
      kLeftAnkleRoll = 5,
      // Right leg
      kRightHipYaw = 6,
      kRightHipPitch = 7,
      kRightHipRoll = 8,
      kRightKnee = 9,
      kRightAnkle = 10,
      kRightAnkleRoll = 11,

      kWaistYaw = 12,

      // Left arm
      kLeftShoulderPitch = 13,
      kLeftShoulderRoll = 14,
      kLeftShoulderYaw = 15,
      kLeftElbow = 16,
      kLeftWistRoll = 17,
      kLeftWistPitch = 18,
      kLeftWistYaw = 19,
      // Right arm
      kRightShoulderPitch = 20,
      kRightShoulderRoll = 21,
      kRightShoulderYaw = 22,
      kRightElbow = 23,
      kRightWistRoll = 24,
      kRightWistPitch = 25,
      kRightWistYaw = 26,

      kNotUsedJoint = 27,
      kNotUsedJoint1 = 28,
      kNotUsedJoint2 = 29,
      kNotUsedJoint3 = 30,
      kNotUsedJoint4 = 31,
      kNotUsedJoint5 = 32,
      kNotUsedJoint6 = 33,
      kNotUsedJoint7 = 34
    };
    //flags
    bool first_cb = false;
    bool arm_initialized = false;
    //parameters
    const std::string kTopicArmSDK = "rt/arm_sdk";
    const std::string kTopicState = "rt/lowstate";

    
    const std::array<float, 15> kp_array = { 120, 120, 80, 50, 50, 50, 50, 
                                    120, 120, 80, 50, 50, 50, 50, 
                                    200 };
    const std::array<float, 15> kd_array = { 2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 
                                    2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 
                                    2.0 };
    /*                                
    const std::array<float, 15> kp_array = { 240, 240, 160, 100, 50, 50, 50, 
                                    240, 240, 160, 100, 50, 50, 50, 
                                    200 };
    const std::array<float, 15> kd_array = { 2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 
                                    2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 
                                    2.0 };
    */
    const std::array<float, 15> init_pos{0.f, 0.3,  0.f, 0, 0, 0, 0,
                                    0.f, -0.3, 0.f, 0, 0, 0, 0,
                                    0.f};
                                    //Joints command
    std::array<float, 15> q_cmd{};

    const std::array<JointIndex, 15> arm_joints = {
        JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
        JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
        JointIndex::kLeftWistRoll,       JointIndex::kLeftWistPitch,     JointIndex::kLeftWistYaw,       
        JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
        JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow, 
        JointIndex::kRightWistRoll,      JointIndex::kRightWistPitch,    JointIndex::kRightWistYaw,       
        JointIndex::kWaistYaw};
    float weight = 0.f;
    float weight_rate = 0.2f;
    float dq = 0.f;
    float tau_ff = 0.f;

    float control_dt = 0.02f;
    float max_joint_velocity = 0.5f;

    float delta_weight = weight_rate * control_dt;
    float max_joint_delta = max_joint_velocity * control_dt;
    std::chrono::duration<int64_t, std::milli> sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

  
    //Publisher, subscriber and msg for low level (arms)
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> arm_sdk_publisher;
    std::shared_ptr<unitree_hg::msg::dds_::LowCmd_> msg;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> low_state_subscriber;
    std::shared_ptr<unitree_hg::msg::dds_::LowState_> state_msg;

  public:
    Arm_motion();
    void initialize_arms();
    void move_arms_integral(std::array<float, 15>);
    void move_arms_polynomial(std::array<float, 15>, float);
    void stop_arms();
  };

  #endif