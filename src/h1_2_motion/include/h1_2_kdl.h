#ifndef _H1_2_KDL_
#define _H1_2_KDL_

/**
 * @file h1_2_kdl.h
 * @brief This contains the kdl functionalities implementation for h1_2.
 */


#include <fstream>
#include "arm_motion.h"
#include <filesystem> 

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

#define SE3_dim 7 //3 position, 4 quaternion 



/**
 * @class H1_2_kdl
 * @brief A class for H1_2_kdl exploiting locomotion, arm motion, hand motion.
 */ 
class H1_2_kdl{
  private:
    //KDL
    KDL::Tree _h1_2_tree;
    KDL::Tree _h1_2_upper_limb_tree;

    std::string base_link = "pelvis";
    std::string tip_link_l  = "L_hand_base_link";
    std::string tip_link_r  = "R_hand_base_link";

    KDL::Chain _k_chain_l;
    KDL::Chain _k_chain_r;
    KDL::Jacobian _jacobian_l; //Left arm Jacobian matrix
    KDL::Jacobian _jacobian_r; //Right arm Jacobian matrix
    KDL::Jacobian J_cog; //COG Jacobian matrix
    std::shared_ptr<KDL::JntArray>_q_l; //Left arm joint position (includes the waist in _q_l[0])
    std::shared_ptr<KDL::JntArray>_q_r; //Right arm joint position (includes the waist in _q_r[0])
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





  public:
    H1_2_kdl();
    bool init_robot_model();
    std::array<float, CARTESIAN_DIM> compute_ee_forces(std::array<float, UPPER_LIMB_JOINTS_DIM> q, std::array<float, UPPER_LIMB_JOINTS_DIM> tau_est, float alpha);
    Eigen::MatrixXd get_upper_limb_jacobian(std::array<float, UPPER_LIMB_JOINTS_DIM> q);
    Eigen::MatrixXd computeWholeBodyCoGJacobianHumanoid(std::array<float, JOINTS_DIM> q);
  
    //std::array<float, UPPER_LIMB_JOINTS_DIM> compute_ikin(std::array<float, UPPER_LIMB_JOINTS_DIM> q_in, std::array<float, CARTESIAN_DIM> x_e);
    std::array<float, CARTESIAN_DIM> admittance_control(std::array<float, CARTESIAN_DIM> x_e, std::array<float, CARTESIAN_DIM> f_ext);
    void set_admittance_gains(Eigen::MatrixXd M_d,  Eigen::MatrixXd D_d,  Eigen::MatrixXd K_d);
    bool compute_upper_limb_ikin(std::array<float, SE3_dim> left_ee_pose, 
                        std::array<float, SE3_dim> right_ee_pose, 
                        std::array<float, UPPER_LIMB_JOINTS_DIM> q_init, std::array<float, UPPER_LIMB_JOINTS_DIM> q_min, 
                        std::array<float, UPPER_LIMB_JOINTS_DIM> q_max, std::array<float, UPPER_LIMB_JOINTS_DIM> & q_output);

    bool compute_upper_limb_fk(std::array<float, UPPER_LIMB_JOINTS_DIM> q_in,
                     std::array<float, SE3_dim> & left_ee_pose,
                     std::array<float, SE3_dim> & right_ee_pose);

  };


#endif