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

#include <eigen3/Eigen/Dense>
#include <unordered_map>
#include <array>

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
    void move_hands(std::array<float, 12>& angles);
    
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
    std::array<float, 12> get_angles();

private:
    void InitDDS_();

    // DDS parameters
    std::mutex mtx;
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::MotorCmds_> handcmd;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::MotorStates_> handstate;
    unitree_go::msg::dds_::MotorCmds_ cmd;
    unitree_go::msg::dds_::MotorStates_ state;

};


#endif