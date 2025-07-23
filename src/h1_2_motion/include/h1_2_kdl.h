#ifndef _H1_2_KDL_
#define _H1_2_KDL_

/**
 * @file h1_2_kdl.h
 * @brief This contains the kdl functionalities implementation for h1_2.
 */


#include <fstream>
#include <filesystem> 
#include <array>
#include <iostream>
#include <unistd.h>
#include <string>
#include <set>

//KDL

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <eigen3/Eigen/Dense>

#define WRENCH_DIM 12
#define UPPER_LIMB_JOINTS_DIM 14
#define CARTESIAN_DIM 7
#define JOINTS_DIM 27




/**
 * @class H1_2_kdl
 * @brief A class for H1_2 kinematics and dynamics, using KDL.
 */ 
class H1_2_kdl{
  private:
    //KDL
    KDL::Tree _h1_2_tree;
    KDL::Tree _h1_2_upper_limb_tree;

    std::string base_link = "torso_link";
    std::string tip_link_l  = "L_hand_base_link";
    std::string tip_link_r  = "R_hand_base_link";
    std::vector<std::string> _upper_limb_joint_names;

    KDL::Chain _k_chain_l;
    KDL::Chain _k_chain_r;
    KDL::Jacobian _jacobian_l; //Left arm Jacobian matrix
    KDL::Jacobian _jacobian_r; //Right arm Jacobian matrix
    KDL::Jacobian J_cog; //COG Jacobian matrix
    std::shared_ptr<KDL::JntArray>_q_l; //Left arm joint position (includes the waist in _q_l[0])
    std::shared_ptr<KDL::JntArray>_q_r; //Right arm joint position (includes the waist in _q_r[0])
    std::shared_ptr<KDL::JntArray> _q_lb; //Upper limb joints lower bounds
    std::shared_ptr<KDL::JntArray> _q_ub; //Upper limb joints upper bounds
    std::shared_ptr<KDL::ChainJntToJacSolver> _jacobian_r_solver; //Right Jacobian solver
    std::shared_ptr<KDL::ChainJntToJacSolver> _jacobian_l_solver; //Left Jacobian solver

    std::shared_ptr<KDL::TreeFkSolverPos_recursive> fkSolver;
    std::shared_ptr<KDL::TreeJntToJacSolver> jacSolver;

    Eigen::MatrixXd _jacobian_l_eigen;
    Eigen::MatrixXd _jacobian_r_eigen;
    Eigen::VectorXd _tau_est_l;
    Eigen::VectorXd _tau_est_r;
    Eigen::VectorXd _f_est_l;
    Eigen::VectorXd _f_est_r;
    Eigen::VectorXd _f_est_l_old;
    Eigen::VectorXd _f_est_r_old;
    bool first_force_comput = false;

    Eigen::MatrixXd M_adm;
    Eigen::MatrixXd D_adm;
    Eigen::MatrixXd K_adm;

    Eigen::MatrixXd r_pinv(Eigen::MatrixXd); //Right pseudoinverse
    Eigen::MatrixXd r_pinv_svd(Eigen::MatrixXd A, double tol = 1e-6); //Right pseudoinverse svd

    void update_state(std::array<float, UPPER_LIMB_JOINTS_DIM> q, std::array<float, UPPER_LIMB_JOINTS_DIM> tau_est);
    void update_state(std::array<float, UPPER_LIMB_JOINTS_DIM> q);

    bool extractMinimalSubTree();

    
    void extract_joint_names_from_tree();
    void extract_joint_names_recursive(const KDL::TreeElement& segment);
    

  public:
    H1_2_kdl();
    bool init_robot_model();
    std::array<float, WRENCH_DIM> compute_ee_forces(std::array<float, UPPER_LIMB_JOINTS_DIM> q, std::array<float, UPPER_LIMB_JOINTS_DIM> tau_est, float alpha);
    Eigen::MatrixXd get_upper_limb_jacobian(std::array<float, UPPER_LIMB_JOINTS_DIM> q);
    Eigen::MatrixXd computeWholeBodyCoGJacobianHumanoid(std::array<float, JOINTS_DIM> q);

    /**
     * @brief Two-dimensional admittance filter for force tracking with infinite compliance. 
     * This function receives as input the force tracking error, 
     * (desired force - actual force) and the computed reference velocity at the previous step. As output, the function computes the reference acceleration 
     * according to the admittance equation to regulate
     * the force, with infinite compliance, without position reference. The output is given by the equation inv(M_d) * (f_d - f - D * x_dot)
     * 
     * @param left_position Computed left end-effector reference position at previous step. Coordinates order: 
     * [PositionX, PositionY]
     * @param left_velocity Computed left end-effector reference velocity at previous step. Coordinates order: 
     * [VelocityX, VelocityY]
     * @param left_delta_force Left end-effector force error, desired force - actual force. Coordinates order: 
     * [ForceX, ForceY]
     * @param left_eq_position Equilibrium left end-effector position. Coordinates order: 
     * [PositionX, PositionY]
     * @param left_inertia Desired left end-effector inertia Matrix. This has to be positive definite. Order 
     * [inertiaXX, inertiaXY, inertiaYX, inertiaYY]
     * @param left_damping Desired left end-effector Damping Matrix. This has to be positive definite. Order 
     * [DampingXX, DampingXY, DampingYX, DampingYY]
     * @param left_stiffness Desired left end-effector Stiffness Matrix. This has to be positive definite. Order 
     * [StiffnessXX, StiffnessXY, StiffnessYX, StiffnessYY]
     * @param right_position Computed right end-effector reference position at previous step. Coordinates order: 
     * [PositionX, PositionY]
     * @param right_velocity Computed right end-effector reference velocity at previous step. Coordinates order: 
     * [VelocityX, VelocityY]
     * @param right_delta_force Right end-effector force error, desired force - actual force. Coordinates order: 
     * [ForceX, ForceY]
     * @param right_eq_position Equilibrium right end-effector position. Coordinates order: 
     * [PositionX, PositionY]
     * @param right_inertia Desired right end-effector inertia Matrix. This has to be positive definite. Order 
     * [inertiaXX, inertiaXY, inertiaYX, inertiaYY]
     * @param right_damping Desired right end-effector Damping Matrix. This has to be positive definite. Order 
     * [DampingXX, DampingXY, DampingYX, DampingYY]
     * @param right_stiffness Desired right end-effector Stiffness Matrix. This has to be positive definite. Order 
     * [StiffnessXX, StiffnessXY, StiffnessYX, StiffnessYY]
     */
      void admittance_filter_2d(std::array<float, 2> left_position, std::array<float, 2> left_velocity, std::array<float, 2> & left_acceleration,
                                  std::array<float, 2> left_delta_force, 
                                  std::array<float, 2> left_eq_position, 
                                  std::array<float, 4> left_inertia, std::array<float, 4> left_damping, std::array<float, 4> left_stiffness,
                                  std::array<float, 2> right_position,
                                  std::array<float, 2> right_velocity, std::array<float, 2> & right_acceleration,std::array<float, 2> right_delta_force, 
                                  std::array<float, 2> right_eq_position, 
                                  std::array<float, 4> right_inertia, std::array<float, 4> right_damping, std::array<float, 4> right_stiffness);

    /**
     * @brief Performs the upper limb inverse kinematics
     * 
     * @param target_left_ee_pose Left end-effector target pose. Coordinates order: 
     * [PositionX, PositionY, PositionZ, QuaternionX, QuaternionY, QuaternionZ, QuaternionW]
     * @param target_right_ee_pose Right end-effector target pose. Coordinates order: 
     * [PositionX, PositionY, PositionZ, QuaternionX, QuaternionY, QuaternionZ, QuaternionW]
     * @param target_left_ee_twist Left end-effector target twist. Coordinates order: 
     * [VelocityX, VelocityY, VelocityZ, OmegaX, OmegaY, OmegaZ]
     * @param target_right_ee_twist Right end-effector target twist. Coordinates order: 
     * [VelocityX, VelocityY, VelocityZ, OmegaX, OmegaY, OmegaZ]
     * @param q_feedback Actual joint configuration. Joint order: left [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],      
     * right [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],    
     * WaistYaw  
     * @param q_dot_output Output joint velocity commands. Joint order: left [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],      
     * right [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],    
     * WaistYaw  
     * @param k_p Position error proportional gain. As order of magnitude, a good
     * choice can be 1/dt, with dt sample time
     * @param k_p Orientation error proportional gain. As order of magnitude, a good
     * choice can be 1/dt, with dt sample time  
     * @param lambda Damping factor
     */

    bool compute_upper_limb_ikin(std::array<float, CARTESIAN_DIM> target_left_ee_pose, 
          std::array<float, CARTESIAN_DIM> target_right_ee_pose, 
          std::array<float, 6> target_left_ee_twist, 
          std::array<float, 6> target_right_ee_twist,
          std::array<float, UPPER_LIMB_JOINTS_DIM> q_feedback, 
          std::array<float, UPPER_LIMB_JOINTS_DIM> & q_dot_output,
          double k_p,
          double k_o,
          double lambda);

    /**
     * @brief Performs the upper limb forward kinematics
     * 
     * @param q_in Input joint configuration. Joint order: left [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],      
     * right [ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, 
     * WristRoll, WristPitch, WristYaw],    
     * WaistYaw  
     * @param left_ee_pose Output left end-effector configuration. Coordinates order: 
     * [PositionX, PositionY, PositionZ, QuaternionX, QuaternionY, QuaternionZ, QuaternionW]
     * @param right_ee_pose Output right end-effector configuration. Coordinates order: 
     * [PositionX, PositionY, PositionZ, QuaternionX, QuaternionY, QuaternionZ, QuaternionW]
     */
    bool compute_upper_limb_fk(std::array<float, UPPER_LIMB_JOINTS_DIM> q_in,
                     std::array<float, CARTESIAN_DIM> & left_ee_pose,
                     std::array<float, CARTESIAN_DIM> & right_ee_pose);

  };


#endif