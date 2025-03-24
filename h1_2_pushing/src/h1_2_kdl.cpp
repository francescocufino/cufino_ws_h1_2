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
  
	std::string base_link = "pelvis";
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
  _tau_est_r.resize(_k_chain_r.getNrOfJoints());
  _tau_est_l.resize(_k_chain_l.getNrOfJoints());
  _f_est_l.resize(6);
  _f_est_r.resize(6);
  _f_est_l_old.resize(6);
  _f_est_r_old.resize(6);

  //Allocate jacobian solver and jntarray
  _jacobian_l_solver=std::make_shared<KDL::ChainJntToJacSolver>(_k_chain_l);
  _jacobian_r_solver=std::make_shared<KDL::ChainJntToJacSolver>(_k_chain_r);
  _q_l = std::make_shared<KDL::JntArray>(_k_chain_l.getNrOfJoints());
  _q_r = std::make_shared<KDL::JntArray>(_k_chain_r.getNrOfJoints());

  return true;

}

void H1_2_kdl::update_state(std::array<float, UPPER_LIMB_JOINTS_DIM> q, std::array<float, UPPER_LIMB_JOINTS_DIM> tau_est){
  //Update joints and torques
  for(int i=0; i<UPPER_LIMB_JOINTS_DIM; i++){
    if(i<=6){
      _q_l->data[i+1] = q.at(i);
      _tau_est_l(i+1) = tau_est.at(i);
    }
    else if(i<=14){
      _q_r->data[i+1-7] = q.at(i);
      _tau_est_r(i+1-7) = tau_est.at(i);
    }
    else{//UNDERSTAND IF THIS IS THE TORSO AS EXPECTED !!!!!!
      _q_l->data[0] = q.at(i);
      _tau_est_l(0) = tau_est.at(i);
      _q_r->data[0] = q.at(i);
      _tau_est_r(0) = tau_est.at(i);
    }
  }

  //Update Jacobians
    if (_jacobian_l_solver->JntToJac(*_q_l, _jacobian_l) < 0) {
        std::cerr << "Failed to compute left arm Jacobian!" << std::endl;
        exit(1);
    }
    _jacobian_l_eigen = _jacobian_l.data;

    if (_jacobian_r_solver->JntToJac(*_q_r, _jacobian_r) < 0) {
        std::cerr << "Failed to compute right arm Jacobian!" << std::endl;
        exit(1);
    }
    _jacobian_r_eigen = _jacobian_r.data;
}

void H1_2_kdl::update_state(std::array<float, UPPER_LIMB_JOINTS_DIM> q){
  //Update joints and torques
  for(int i=0; i<UPPER_LIMB_JOINTS_DIM; i++){
    if(i<=6){
      _q_l->data[i+1] = q.at(i);
    }
    else if(i<=14){
      _q_r->data[i+1-7] = q.at(i);
    }
    else{//UNDERSTAND IF THIS IS THE TORSO AS EXPECTED !!!!!!
      _q_l->data[0] = q.at(i);
      _q_r->data[0] = q.at(i);
    }
  }

  //Update Jacobians
    if (_jacobian_l_solver->JntToJac(*_q_l, _jacobian_l) < 0) {
        std::cerr << "Failed to compute left arm Jacobian!" << std::endl;
        exit(1);
    }
    _jacobian_l_eigen = _jacobian_l.data;

    if (_jacobian_r_solver->JntToJac(*_q_r, _jacobian_r) < 0) {
        std::cerr << "Failed to compute right arm Jacobian!" << std::endl;
        exit(1);
    }
    _jacobian_r_eigen = _jacobian_r.data;
}

Eigen::MatrixXd H1_2_kdl::get_upper_limb_jacobian(std::array<float, UPPER_LIMB_JOINTS_DIM> q){
  Eigen::MatrixXd J_u;
  update_state(q);
  //ADJUST HERE, THE FIRST ROW OF BOTH RELATES TORSO VEL
  J_u << _jacobian_r_eigen, _jacobian_l_eigen;
  return J_u;
}


std::array<float, CARTESIAN_DIM> H1_2_kdl::compute_ee_forces(std::array<float, UPPER_LIMB_JOINTS_DIM> q, std::array<float, UPPER_LIMB_JOINTS_DIM> tau_est, float alpha){
  //update state
  update_state(q, tau_est);
  //transform torques in forces at ee
  _f_est_l = r_pinv_svd(_jacobian_l_eigen.transpose()) * _tau_est_l;
  _f_est_r = r_pinv_svd(_jacobian_r_eigen.transpose()) * _tau_est_r;
  //filter forces (after first iteration)
  // if(first_force_comput) {
  // _f_est_l = alpha*_f_est_l + (1-alpha)*_f_est_l_old;
  // _f_est_r = alpha*_f_est_r + (1-alpha)*_f_est_r_old;
  // }
  // else{first_force_comput = true;}
  // _f_est_l_old = _f_est_l;
  // _f_est_r_old = _f_est_r;
  std::array<float, CARTESIAN_DIM> f_ee;
  std::copy(_f_est_l.data(), _f_est_l.data() + 6, f_ee.begin());      
  std::copy(_f_est_r.data(), _f_est_r.data() + 6, f_ee.begin() + 6);  
  return f_ee;
}


Eigen::MatrixXd H1_2_kdl::r_pinv(Eigen::MatrixXd A){
  return A.transpose() * (A * A.transpose()).inverse(); 
}

Eigen::MatrixXd H1_2_kdl::r_pinv_svd(Eigen::MatrixXd A, double tol){
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd singularValues = svd.singularValues();

    // Create a diagonal matrix for the inverse singular values
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(A.cols(), A.rows());
   
    for (int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tol) {
            S_inv(i, i) = 1.0 / singularValues(i);
        }
    }

    // Compute the right pseudoinverse
    return svd.matrixV() * S_inv * svd.matrixU().transpose();
}









