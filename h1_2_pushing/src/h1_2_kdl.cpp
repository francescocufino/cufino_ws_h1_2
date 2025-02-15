#include "h1_2_kdl.h"

H1_2_kdl::H1_2_kdl(){
  if(!init_robot_model()){std::cerr << "Failed to build robot model"; exit(1);}
  std::cout << "Tree joints and segments: " << _h1_2_tree.getNrOfJoints() << " - " << _h1_2_tree.getNrOfSegments() << std::endl;
  std::cout << "Left hand chain joints and segments: " << _k_chain_l.getNrOfJoints() << " - " << _k_chain_l.getNrOfSegments() << std::endl;
  std::cout << "Right hand chain joints and segments: " << _k_chain_r.getNrOfJoints() << " - " << _k_chain_r.getNrOfSegments() << std::endl;
      
}

bool H1_2_kdl::init_robot_model(){
  //Parse tree from urdf
  if (!kdl_parser::treeFromFile("../h1_2_description/h1_2.urdf", _h1_2_tree)){
              return false;
      }

	std::string base_link = "torso_link";
	//std::string tip_link_l  = "left_wrist_yaw_link";
  std::string tip_link_l  = "L_hand_base_link";
  //std::string tip_link_r  = "right_wrist_yaw_link";
  std::string tip_link_r  = "R_hand_base_link";
  
  //Create kinematic chains
	if (!_h1_2_tree.getChain(base_link, tip_link_l, _k_chain_l) ||
    !_h1_2_tree.getChain(base_link, tip_link_r, _k_chain_r)) return false;

	//Resize eigen MatrixXd
  _jacobian_l.resize(_k_chain_l.getNrOfJoints());
  _jacobian_r.resize(_k_chain_r.getNrOfJoints());

  //Allocate jacobian solver and jntarray
  _jacobian_l_solver=std::make_shared<KDL::ChainJntToJacSolver>(_k_chain_l);
  _jacobian_r_solver=std::make_shared<KDL::ChainJntToJacSolver>(_k_chain_r);
  _q_l = std::make_shared<KDL::JntArray>(_k_chain_l.getNrOfJoints());
  _q_r = std::make_shared<KDL::JntArray>(_k_chain_r.getNrOfJoints());

  return true;

}

Eigen::VectorXd H1_2_kdl::compute_left_ee_force(Eigen::VectorXd q, Eigen::VectorXd tau){
    //update joint position
    if(q.size()!=_q_l->rows() || tau.size()!=_q_l->rows()){std::cerr<<"Wrong dimension\n";}
    _q_l->data = q;

    // Compute Jacobian at the given joint configuration
    if (_jacobian_l_solver->JntToJac(*_q_l, _jacobian_l) < 0) {
        std::cerr << "Failed to compute Jacobian!" << std::endl;
        return Eigen::VectorXd::Zero(6);
    }

    // Convert Jacobian to Eigen format
    _jacobian_l_eigen = _jacobian_l.data;

    // Compute End-Effector Force using the Transpose Method
    return r_pinv(_jacobian_l_eigen.transpose()) * tau;

}

Eigen::VectorXd H1_2_kdl::compute_right_ee_force(Eigen::VectorXd q, Eigen::VectorXd tau){
    //update joint position
    if(q.size()!=_q_r->rows() || tau.size()!=_q_r->rows()){std::cerr<<"Wrong dimension\n";}
    _q_r->data = q;

    // Compute Jacobian at the given joint configuration
    if (_jacobian_r_solver->JntToJac(*_q_r, _jacobian_r) < 0) {
        std::cerr << "Failed to compute Jacobian!" << std::endl;
        return Eigen::VectorXd::Zero(6);
    }

    // Convert Jacobian to Eigen format
    _jacobian_r_eigen = _jacobian_r.data;

    // Compute End-Effector Force using the Transpose Method
    return r_pinv(_jacobian_r_eigen.transpose()) * tau;

}

Eigen::MatrixXd H1_2_kdl::r_pinv(Eigen::MatrixXd A){
  return A.transpose() * (A * A.transpose()).inverse(); 
}












