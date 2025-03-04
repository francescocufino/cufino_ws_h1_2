#ifndef _H1_2_KDL_
#define _H1_2_KDL_

/**
 * @file h1_2_kdl.h
 * @brief This contains the kdl functionalities implementation for h1_2.
 */


#include <fstream>
#include "arm_motion.h"

//KDL

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <eigen3/Eigen/Dense>



/**
 * @class H1_2_kdl
 * @brief A class for H1_2_kdl exploiting locomotion, arm motion, hand motion.
 */ 
class H1_2_kdl{
  private:
    //KDL
    KDL::Tree _h1_2_tree;
    KDL::Chain _k_chain_l;
    KDL::Chain _k_chain_r;
    KDL::Jacobian _jacobian_l; //Left arm Jacobian matrix
    KDL::Jacobian _jacobian_r; //Right arm Jacobian matrix
    std::shared_ptr<KDL::JntArray>_q_l; //Left arm joint position
    std::shared_ptr<KDL::JntArray>_q_r; //Right arm joint position
    std::shared_ptr<KDL::ChainJntToJacSolver> _jacobian_r_solver; //Right Jacobian solver
    std::shared_ptr<KDL::ChainJntToJacSolver> _jacobian_l_solver; //Left Jacobian solver
    Eigen::MatrixXd _jacobian_l_eigen;
    Eigen::MatrixXd _jacobian_r_eigen;
    Eigen::VectorXd _tau_est_l;
    Eigen::VectorXd _tau_est_r;
    Eigen::VectorXd _f_est_l;
    Eigen::VectorXd _f_est_r;
    Eigen::VectorXd _f_est_l_old;
    Eigen::VectorXd _f_est_r_old;
    bool first_force_comput = false;
    Eigen::MatrixXd r_pinv(Eigen::MatrixXd); //Right pseudoinverse
    Eigen::MatrixXd r_pinv_svd(Eigen::MatrixXd A, double tol = 1e-6); //Right pseudoinverse svd

    void update_state(std::array<float, UPPER_LIMB_JOINTS_DIM> q, std::array<float, UPPER_LIMB_JOINTS_DIM> tau_est);



  public:
    H1_2_kdl();
    bool init_robot_model();
    std::array<float, CARTESIAN_DIM> compute_ee_forces(std::array<float, UPPER_LIMB_JOINTS_DIM> q, std::array<float, UPPER_LIMB_JOINTS_DIM> tau_est, float alpha);
  };


#endif