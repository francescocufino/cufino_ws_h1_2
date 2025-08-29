/**
 * @file hand_motion.h
 * @brief Provides functions and classes for basic inspire hands motion.
 * 
 * This header file contains the declaration of functions and classes 
 * that perform basic inspire hands motion, like close and open, and to  
 * get the actual state. 
 * 
 * @author Francesco Cufino
 * @date 09/02/2024
 * @version 1.0
 */

#ifndef _HAND_MOTION_
#define _HAND_MOTION_

// Inspire Hand Topic IDL Types
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
// DDS Channel
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/thread/thread.hpp>

#include "inspire_hand_ctrl.hpp"
#include "inspire_hand_state.hpp"
#include "inspire_hand_touch.hpp"

#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <array>

#define HANDS_JOINTS_DIM 12
#define R_HAND_JOINTS_DIM 6
#define L_HAND_JOINTS_DIM 6


/**
 * @class Hand_motion
 * @brief A class for basic inspire hands motion.
 */
class Hand_motion{
public:
    Hand_motion();
    /**
     * @brief Move the fingers to the specified angles 
     * @param angles Target angles for hands normalized in the range [0, 1], with 
     *       0: close  1: open.
     * Joint order: right [pinky, ring, middle, index, thumb_bend, thumb_rotation],
     * left [pinky, ring, middle, index, thumb_bend, thumb_rotation]
     * 
     */
    void move_hands(std::array<float, HANDS_JOINTS_DIM>& angles);
    
    /**
     * @brief Get the right hand angles normalized in the range [0, 1], with 
     *       0: close  1: open.
     * 
     * Joint order: right [pinky, ring, middle, index, thumb_bend, thumb_rotation]
     */
    std::array<float, 6> getRightQ();

    /**
     * @brief Get the left hand angles normalized in the range [0, 1], with 
     *       0: close  1: open.
     * 
     * Joint order: left [pinky, ring, middle, index, thumb_bend, thumb_rotation]
     */
    std::array<float, 6> getLeftQ();

    /**
     * @brief Get all the hand angles normalized in the range [0, 1], with 
     *       0: close  1: open.
     * 
     * Joint order: right [pinky, ring, middle, index, thumb_bend, thumb_rotation],
     * left [pinky, ring, middle, index, thumb_bend, thumb_rotation]
     */
    std::array<float, HANDS_JOINTS_DIM> get_angles();

private:
    void InitDDS_();

    // DDS parameters
    std::mutex mtx;
    unitree::robot::ChannelPublisherPtr<inspire::inspire_hand_ctrl> handcmd_r;
    unitree::robot::ChannelSubscriberPtr<inspire::inspire_hand_state> handstate_r;
    unitree::robot::ChannelSubscriberPtr<inspire::inspire_hand_touch> handtouch_r;
    inspire::inspire_hand_ctrl cmd_r;
    inspire::inspire_hand_state state_r;
    inspire::inspire_hand_touch touch_r;

    unitree::robot::ChannelPublisherPtr<inspire::inspire_hand_ctrl> handcmd_l;
    unitree::robot::ChannelSubscriberPtr<inspire::inspire_hand_state> handstate_l;
    unitree::robot::ChannelSubscriberPtr<inspire::inspire_hand_touch> handtouch_l;
    inspire::inspire_hand_ctrl cmd_l;
    inspire::inspire_hand_state state_l;
    inspire::inspire_hand_touch touch_l;


};


#endif