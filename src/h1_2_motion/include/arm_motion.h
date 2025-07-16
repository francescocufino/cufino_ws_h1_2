/**
 * @file arm_motion.h
 * @brief Provides functions and classes for basic arm motion.
 * 
 * This header file contains the declaration of functions and classes 
 * that perform basic arm motion, like set target joint position with a specified duration. 
 * 
 * @author Francesco Cufino
 * @date 09/02/2024
 * @version 1.0
 */
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

//Kinematics and dynamics
#include "h1_2_kdl.h"



/**
 * @class Arm_motion
 * @brief A class for basic arm motion.
 */ 
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

    /*
    const std::array<float, UPPER_LIMB_JOINTS_DIM> kp_array = { 100, 100, 60, 35, 35, 35, 35, 
                                    100, 100, 60, 35, 35, 35, 35, 
                                    200 };
    const std::array<float, UPPER_LIMB_JOINTS_DIM> kd_array = { 2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 
                                    2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 
                                    2.0 };
                                    
    */                       
    const std::array<float, UPPER_LIMB_JOINTS_DIM> kp_array = { 240, 240, 160, 100, 50, 50, 50, 
                                    240, 240, 160, 100, 50, 50, 50, 
                                    200 };
    const std::array<float, UPPER_LIMB_JOINTS_DIM> kd_array = { 2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 
                                    2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 
                                    2.0 };
    
    const std::array<float, UPPER_LIMB_JOINTS_DIM> init_pos{0.f, 0.3,  0.f, 0, 0, 0, 0,
                                    0.f, -0.3, 0.f, 0, 0, 0, 0,
                                    0.f};
                                    //Joints command
    std::array<float, UPPER_LIMB_JOINTS_DIM> q_cmd{};
    std::array<float, UPPER_LIMB_JOINTS_DIM> q_cmd_ikin{};


    bool initialized_q_cmd = false;
    bool initialized_q_cmd_ikin = false;


    const std::array<JointIndex, UPPER_LIMB_JOINTS_DIM> arm_joints = {
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

    //Kinematics and dynamics
    H1_2_kdl h1_2_kdl;

  
    //Publisher, subscriber and msg for low level (arms)
    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> arm_sdk_publisher;
    std::shared_ptr<unitree_hg::msg::dds_::LowCmd_> msg;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> low_state_subscriber;
    std::shared_ptr<unitree_hg::msg::dds_::LowState_> state_msg;

    //Safety check
    bool safety_check(std::array<float, UPPER_LIMB_JOINTS_DIM> q_target, double delta);

    //Data storage for test move_ee_linear
    std::vector<std::array<float, UPPER_LIMB_JOINTS_DIM>> joint_positions_cmd;
    std::vector<std::array<float, UPPER_LIMB_JOINTS_DIM>> joint_positions_actual;
    std::vector<std::array<float, CARTESIAN_DIM>> left_ee_cmd;
    std::vector<std::array<float, CARTESIAN_DIM>> right_ee_cmd;
    std::vector<std::array<float, CARTESIAN_DIM>> left_ee_actual;
    std::vector<std::array<float, CARTESIAN_DIM>> right_ee_actual;
    std::vector<std::array<float, 6>> left_twist_ee_cmd;
    std::vector<std::array<float, 6>> right_twist_ee_cmd;

    //Save data to csv for test move_ee_linear
    template <size_t N>
    void writeCSV(const std::string &filename, const std::vector<std::array<float, N>> &data);
    void store_data();

    //Emergency stop
    bool stop = false;

  public:
    Arm_motion();

    /**
     * @brief Initialize the arms bringing them to a specific initial configuration
     */
    void initialize_arms();

    /**
     * @brief Move the arms to the specified joint angles with an integral planner
     * 
     * @param q_f Target configuration. Joint order: left [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],      
     * right [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],    
     * WaistYaw
     * @param t_f duration
     */
    void move_arms_integral(std::array<float, UPPER_LIMB_JOINTS_DIM> q_f, float t_f);

    /**
     * @brief Move the arms to the specified joint angles with a 5-th order polynomial planner
     * 
     * @param q_f Target configuration. Joint order: left [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],      
     * right [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],    
     * WaistYaw
     * @param t_f duration
     */
    void move_arms_polynomial(std::array<float, UPPER_LIMB_JOINTS_DIM> q_f, float t_f);

    /**
     * @brief Move the end-effectors to the specified targets along
     * a straight line, with a polynomial velocity profile
     * 
     * @param target_left_ee_pose Left end-effector target pose. Coordinates order: 
     * [PositionX, PositionY, PositionZ, QuaternionX, QuaternionY, QuaternionZ, QuaternionW]
     * @param target_right_ee_pose Right end-effector target pose. Coordinates order: 
     * [PositionX, PositionY, PositionZ, QuaternionX, QuaternionY, QuaternionZ, QuaternionW]
     * @param t_f duration
     */
    void move_ee_linear(std::array<float, CARTESIAN_DIM> target_left_ee_pose, 
      std::array<float, CARTESIAN_DIM> target_right_ee_pose, float t_f);

    /**
     * @brief Sets instantaneously the upper limb joints to the specified joint angles
     * 
     * @param q_f Target configuration. Joint order: left [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],      
     * right [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],    
     * WaistYaw
     */
    void set_upper_limb_joints(std::array<float, UPPER_LIMB_JOINTS_DIM> q_target);

    /**
     * @brief Sets instantaneously the end-effectors poses.
     * This function has to be called in a loop in which the targets change following a
     * desired trajectory. Closed loop inverse kinematics is performed, producing
     * as output a velocity command, which is then integrated
     * with an integration time dt and sent as command to the manipulator
     * 
     * @param target_left_ee_pose Left end-effector target pose. Coordinates order: 
     * [PositionX, PositionY, PositionZ, QuaternionX, QuaternionY, QuaternionZ, QuaternionW]
     * @param target_right_ee_pose Right end-effector target pose. Coordinates order: 
     * [PositionX, PositionY, PositionZ, QuaternionX, QuaternionY, QuaternionZ, QuaternionW]
     * @param target_left_ee_twist Left end-effector target twist. Coordinates order: 
     * [VelocityX, VelocityY, VelocityZ, OmegaX, OmegaY, OmegaZ]
     * @param target_right_ee_twist Right end-effector target twist. Coordinates order: 
     * [VelocityX, VelocityY, VelocityZ, OmegaX, OmegaY, OmegaZ]
     * @param dt Integration time
     */

    bool set_end_effector_targets(std::array<float, CARTESIAN_DIM> target_left_ee_pose, 
      std::array<float, CARTESIAN_DIM> target_right_ee_pose,
      std::array<float, 6> target_left_ee_twist,
      std::array<float, 6> target_right_ee_twist,
      float dt);

    /**
     * @brief Stop the arms bringing them to the specific initial configuration
     */
    void stop_arms();
        /**
     * @brief Get the arms angles
     * 
     * Joint order: left [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],      
     * right [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],    
     * WaistYaw
     */
    std::array<float, UPPER_LIMB_JOINTS_DIM> get_angles();
    
    /**
     * @brief Get the end-effector poses
     * @param left_ee_pose Left end-effector pose. Coordinates order: 
     * [PositionX, PositionY, PositionZ, QuaternionX, QuaternionY, QuaternionZ, QuaternionW]
     * @param right_ee_pose Right end-effector pose. Coordinates order: 
     * [PositionX, PositionY, PositionZ, QuaternionX, QuaternionY, QuaternionZ, QuaternionW]
     */
    void get_end_effectors_poses(std::array<float, CARTESIAN_DIM>& left_ee_pose, 
                            std::array<float, CARTESIAN_DIM>& right_ee_pose);
    /**
     * @brief Get estimated arms torques
     * 
     * Joint order: left [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],      
     * right [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],    
     * WaistYaw
     */
    std::array<float, UPPER_LIMB_JOINTS_DIM> get_est_torques();


  };

  #endif
