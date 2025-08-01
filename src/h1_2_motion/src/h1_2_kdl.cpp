#include "h1_2_kdl.h"

H1_2_kdl::H1_2_kdl(){
  if(!init_robot_model()){std::cerr << "Failed to build robot model"; exit(1);}
  std::cout << "Tree joints and segments: " << _h1_2_tree.getNrOfJoints() << " - " << _h1_2_tree.getNrOfSegments() << std::endl;
  std::cout << "Upper limb tree joints and segments: " << _h1_2_upper_limb_tree.getNrOfJoints() << " - " << _h1_2_upper_limb_tree.getNrOfSegments() << std::endl;
  std::cout << "Left hand chain joints and segments: " << _k_chain_l.getNrOfJoints() << " - " << _k_chain_l.getNrOfSegments() << std::endl;
  std::cout << "Right hand chain joints and segments: " << _k_chain_r.getNrOfJoints() << " - " << _k_chain_r.getNrOfSegments() << std::endl;
      
}

bool H1_2_kdl::init_robot_model(){

  char exe_path_c[4096];
  ssize_t len = readlink("/proc/self/exe", exe_path_c, sizeof(exe_path_c) - 1);
  if (len == -1) {
      std::cerr << "Failed to get executable path" << std::endl;
      return 1;
  }
  exe_path_c[len] = '\0';
  std::filesystem::path exe_path(exe_path_c);

  // Step 2: Compute URDF path relative to executable
  std::filesystem::path urdf_path = exe_path.parent_path().parent_path().parent_path() / "src" / "h1_2_description" / "h1_2.urdf";
  std::string urdf_path_str = urdf_path.string();

  //Parse tree from urdf
  if (!kdl_parser::treeFromFile(urdf_path_str, _h1_2_tree)){
              return false;
      }

  if(!extractMinimalSubTree())
    return false;

  extract_joint_names_from_tree();
//   std::cout << "Joint names in order:\n";
//   for(int i=0; i<_upper_limb_joint_names.size(); i++){
//     std::cout << _upper_limb_joint_names[i] << "\n";
//   }

//   //Print upper limb tree
//   for (const auto& segment_pair : _h1_2_upper_limb_tree.getSegments()) {
//     const std::string& segment_name = segment_pair.first;
//     const KDL::Segment& segment = segment_pair.second.segment;
//     const KDL::Joint& joint = segment.getJoint();

//     std::cout << "Segment: " << segment_name
//               << ", Joint: " << joint.getName()
//               << ", Joint Type: " << joint.getTypeName() << std::endl;
// }

  
  //Create kinematic chains
	if (!_h1_2_tree.getChain(base_link, tip_link_l, _k_chain_l) ||
    !_h1_2_tree.getChain(base_link, tip_link_r, _k_chain_r)) return false;


	//Resize eigen MatrixXd
  _jacobian_l.resize(_k_chain_l.getNrOfJoints());
  _jacobian_r.resize(_k_chain_r.getNrOfJoints());
  J_cog.resize(_h1_2_tree.getNrOfJoints());
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
  _q_lb = std::make_shared<KDL::JntArray>(UPPER_LIMB_JOINTS_DIM);
  _q_ub = std::make_shared<KDL::JntArray>(UPPER_LIMB_JOINTS_DIM);


  // Joints limits (hard-coded)
  _q_lb->data[0] = -3.14f;    // kLeftShoulderPitch
  _q_lb->data[1] = -0.38f;    // kLeftShoulderRoll
  _q_lb->data[2] = -2.66f;    // kLeftShoulderYaw
  _q_lb->data[3] = -0.95f;    // kLeftElbow
  _q_lb->data[4] = -3.01f;    // kLeftWistRoll
  _q_lb->data[5] = -0.4625f;  // kLeftWistPitch
  _q_lb->data[6] = -1.27f;    // kLeftWistYaw
  _q_lb->data[7] = -3.14f;    // kRightShoulderPitch
  _q_lb->data[8] = -3.4f;     // kRightShoulderRoll
  _q_lb->data[9] = -3.01f;    // kRightShoulderYaw
  _q_lb->data[10] = -0.95f;    // kRightElbow
  _q_lb->data[11] = -2.75f;    // kRightWistRoll
  _q_lb->data[12] = -0.4625f;  // kRightWistPitch
  _q_lb->data[13] = -1.27f;    // kRightWistYaw
  //_q_lb->data[0] =-2.35f;     // kWaistYaw

  _q_ub->data[0] =  1.57f;     // kLeftShoulderPitch
  _q_ub->data[1] =  3.4f;      // kLeftShoulderRoll
  _q_ub->data[2] =  3.01f;     // kLeftShoulderYaw
  _q_ub->data[3] =  3.18f;     // kLeftElbow
  _q_ub->data[4] =  2.75f;     // kLeftWistRoll
  _q_ub->data[5] =  0.4625f;   // kLeftWistPitch
  _q_ub->data[6] =  1.27f;     // kLeftWistYaw
  _q_ub->data[7] =  1.57f;     // kRightShoulderPitch
  _q_ub->data[8] =  0.38f;     // kRightShoulderRoll
  _q_ub->data[9] =  2.66f;     // kRightShoulderYaw
  _q_ub->data[10] =  3.18f;     // kRightElbow
  _q_ub->data[11] =  3.01f;     // kRightWistRoll
  _q_ub->data[12] =  0.4625f;   // kRightWistPitch
  _q_ub->data[13] =  1.27f;     // kRightWistYaw
  //_q_ub->data[0] =  2.35f;      // kWaistYaw


  fkSolver=std::make_shared<KDL::TreeFkSolverPos_recursive>(_h1_2_tree);
  jacSolver=std::make_shared<KDL::TreeJntToJacSolver>(_h1_2_tree);

  return true;

}

void H1_2_kdl::update_state(std::array<float, UPPER_LIMB_JOINTS_DIM> q, std::array<float, UPPER_LIMB_JOINTS_DIM> tau_est){//FIX JOINTS ORDER
  //Update joints and torques
  for(int i=0; i<UPPER_LIMB_JOINTS_DIM; i++){
    if(i<=6){
      _q_l->data[i] = q.at(i);
      _tau_est_l(i) = tau_est.at(i);
    }
    else {
      _q_r->data[i-7] = q.at(i);
      _tau_est_r(i-7) = tau_est.at(i);
    }
    // else{//UNDERSTAND IF THIS IS THE TORSO AS EXPECTED !!!!!!
    //   _q_l->data[0] = q.at(i);
    //   _tau_est_l(0) = tau_est.at(i);
    //   _q_r->data[0] = q.at(i);
    //   _tau_est_r(0) = tau_est.at(i);
    // }
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

void H1_2_kdl::update_state(std::array<float, UPPER_LIMB_JOINTS_DIM> q){//FIX JOINTS ORDER
  //Update joints and torques
  for(int i=0; i<UPPER_LIMB_JOINTS_DIM; i++){
    if(i<=6){
      _q_l->data[i] = q.at(i);
    }
    else {
      _q_r->data[i-7] = q.at(i);
    }
    // else{//UNDERSTAND IF THIS IS THE TORSO AS EXPECTED !!!!!!
    //   //First element of left and right arm in this struture is the waist yaw (see if it corresponds to torso)
    //   _q_l->data[0] = q.at(i);
    //   _q_r->data[0] = q.at(i);
    // }
  }
  //So _q_l = [_q_w, _q_l_0, ...] _q_r = [_q_w, _q_r_0, ...]
  //The Jacobians will be jacobian_l = [J_lw, Jl], jacobian_r = [J_rw, Jr]
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
  Eigen::MatrixXd J_u(4,UPPER_LIMB_JOINTS_DIM), J_l(2,7), J_r(2,7), zero;
  update_state(q);
  //Both the Jacobian r and l contain 1st column related to torso and next 7 columns 
  //related to the respective arm. jacobian_l = [J_lw, Jl], jacobian_r = [J_rw, Jr]
  //We consider only the first two rows for x and y velocities
  //J_lw = _jacobian_l_eigen.block<2, 1>(0, 0);
  J_l = _jacobian_l_eigen.block<2, 7>(0, 1);
  //J_rw = _jacobian_r_eigen.block<2, 1>(0, 0);
  J_r = _jacobian_r_eigen.block<2, 7>(0, 1);
  zero = Eigen::MatrixXd::Zero(2, 7);
  //So, the upper limb Jacobian is 
  //Ju = [J_l, 0, J_lw;
        // 0, J_r, J_rw]
  // considering the order of joints variable as q = [q_l; q_r; q_w]

  J_u << J_l, zero,// J_lw,
        zero, J_r;// J_rw;

  return J_u;
}

Eigen::MatrixXd H1_2_kdl::computeWholeBodyCoGJacobianHumanoid(std::array<float, JOINTS_DIM> q_wb){
    KDL::JntArray q;
    q.resize(_h1_2_tree.getNrOfJoints());
    for(int i=0; i<_h1_2_tree.getNrOfJoints(); i++){
      q.data[i] = q_wb.at(i);
      //SORT CORRECTLY !!!!!
    }

    J_cog.data.setZero();

    // Iterate through each segment in the tree
    double totalMass = 0.0;
    for (const auto &[segmentName, segment] : _h1_2_tree.getSegments()) {
        KDL::RigidBodyInertia inertia = segment.segment.getInertia();
        double mass = inertia.getMass();
        if (mass <= 0.0) continue; // Ignore massless links

        // Get the CoG position relative to the link
        KDL::Vector CoM = inertia.getCOG();

        // Compute forward kinematics for this link
        KDL::Frame linkFrame;
        fkSolver->JntToCart(q, linkFrame, segmentName);

        // Compute Jacobian for this segment
        KDL::Jacobian J_link(_h1_2_tree.getNrOfJoints());
        jacSolver->JntToJac(q, J_link, segmentName);

        // Adjust Jacobian to CoG location
        KDL::Jacobian J_cog_i = J_link;
        for (unsigned int j = 0; j < _h1_2_tree.getNrOfJoints(); j++) {
            KDL::Vector omega(J_link(0, j), J_link(1, j), J_link(2, j));
            KDL::Vector linear_correction = omega * CoM;
            J_cog_i(0, j) += linear_correction.x();
            J_cog_i(1, j) += linear_correction.y();
            J_cog_i(2, j) += linear_correction.z();
        }

        // Mass-weighted sum
        J_cog.data += mass * J_cog_i.data;
        totalMass += mass;
    }

    // Normalize by total mass
    if (totalMass > 0.0) {
        J_cog.data /= totalMass;
    }
    Eigen::MatrixXd J_cog_eigen;
    J_cog_eigen = J_cog.data;
    return J_cog_eigen;
}

std::array<float, WRENCH_DIM> H1_2_kdl::compute_ee_forces(std::array<float, UPPER_LIMB_JOINTS_DIM> q, std::array<float, UPPER_LIMB_JOINTS_DIM> tau_est, float alpha){

  //Convert to KDL
  for(int i=0; i<UPPER_LIMB_JOINTS_DIM; i++){
    if(i<=6){
      _q_l->data[i] = q.at(i);
      _tau_est_l(i) = tau_est.at(i);
    }
    else {
      _q_r->data[i-7] = q.at(i);
      _tau_est_r(i-7) = tau_est.at(i);
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



  //transform torques in forces at ee
  _f_est_l = - r_pinv_svd(_jacobian_l_eigen.transpose()) * _tau_est_l;
  _f_est_r = - r_pinv_svd(_jacobian_r_eigen.transpose()) * _tau_est_r;
  //filter forces (after first iteration)
  // if(first_force_comput) {
  // _f_est_l = alpha*_f_est_l + (1-alpha)*_f_est_l_old;
  // _f_est_r = alpha*_f_est_r + (1-alpha)*_f_est_r_old;
  // }
  // else{first_force_comput = true;}
  // _f_est_l_old = _f_est_l;
  // _f_est_r_old = _f_est_r;
  std::array<float, WRENCH_DIM> f_ee;
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

void H1_2_kdl::admittance_filter_2d(std::array<float, 2> left_position, std::array<float, 2> left_velocity, std::array<float, 2> & left_acceleration,
                                    std::array<float, 2> left_delta_force, 
                                    std::array<float, 2> left_eq_position, 
                                    std::array<float, 4> left_inertia, std::array<float, 4> left_damping, std::array<float, 4> left_stiffness,
                                    std::array<float, 2> right_position,
                                    std::array<float, 2> right_velocity, std::array<float, 2> & right_acceleration,std::array<float, 2> right_delta_force, 
                                    std::array<float, 2> right_eq_position, 
                                    std::array<float, 4> right_inertia, std::array<float, 4> right_damping, std::array<float, 4> right_stiffness){
    //Convert to Eigen
    Eigen::Vector2d x_l(left_position.at(0), left_position.at(1));
    Eigen::Vector2d x_dot_l(left_velocity.at(0), left_velocity.at(1));
    Eigen::Vector2d delta_f_l(left_delta_force.at(0), left_delta_force.at(1));
    Eigen::Vector2d x_eq_l(left_eq_position.at(0), left_eq_position.at(1));
    Eigen::Matrix2d M_l, D_l, K_l;
    M_l << left_inertia.at(0), left_inertia.at(1), left_inertia.at(2), left_inertia.at(3);
    D_l << left_damping.at(0), left_damping.at(1), left_damping.at(2), left_damping.at(3);
    K_l << left_stiffness.at(0), left_stiffness.at(1), left_stiffness.at(2), left_stiffness.at(3);

    Eigen::Vector2d x_r(right_position.at(0), right_position.at(1));
    Eigen::Vector2d x_dot_r(right_velocity.at(0), right_velocity.at(1));
    Eigen::Vector2d delta_f_r(right_delta_force.at(0), right_delta_force.at(1));
    Eigen::Vector2d x_eq_r(right_eq_position.at(0), right_eq_position.at(1));
    Eigen::Matrix2d M_r, D_r, K_r;
    M_r << right_inertia.at(0), right_inertia.at(1), right_inertia.at(2), right_inertia.at(3);
    D_r << right_damping.at(0), right_damping.at(1), right_damping.at(2), right_damping.at(3);
    K_r << right_stiffness.at(0), right_stiffness.at(1), right_stiffness.at(2), right_stiffness.at(3);

    //Compute acceleration
    Eigen::Vector2d x_ddot_l = M_l.inverse()*(delta_f_l - D_l * x_dot_l - K_l * (x_l - x_eq_l));
    left_acceleration = {(float)x_ddot_l(0), (float)x_ddot_l(1)};

    Eigen::Vector2d x_ddot_r = M_r.inverse()*(delta_f_r - D_r * x_dot_r - K_r * (x_r - x_eq_r));
    right_acceleration = {(float)x_ddot_r(0), (float)x_ddot_r(1)};

  }


bool H1_2_kdl::compute_upper_limb_ikin(std::array<float, CARTESIAN_DIM> target_left_ee_pose, 
                                              std::array<float, CARTESIAN_DIM> target_right_ee_pose, 
                                              std::array<float, 6> target_left_ee_twist, 
                                              std::array<float, 6> target_right_ee_twist,
                                              std::array<float, UPPER_LIMB_JOINTS_DIM> q_feedback, 
                                              std::array<float, UPPER_LIMB_JOINTS_DIM> & q_dot_output,
                                              double k_p,
                                              double k_o,
                                              double lambda){

  //Convert to Eigen
  //Desired Poses
  Eigen::Vector3d p_l_des(target_left_ee_pose.at(0),target_left_ee_pose.at(1), target_left_ee_pose.at(2));
  Eigen::Quaterniond q_l_des(target_left_ee_pose.at(6), target_left_ee_pose.at(3), target_left_ee_pose.at(4), target_left_ee_pose.at(5));
  Eigen::Vector3d p_r_des(target_right_ee_pose.at(0),target_right_ee_pose.at(1), target_right_ee_pose.at(2));
  Eigen::Quaterniond q_r_des(target_right_ee_pose.at(6), target_right_ee_pose.at(3), target_right_ee_pose.at(4), target_right_ee_pose.at(5));
  
  //Actual Poses
  std::array<float, CARTESIAN_DIM> left_ee_pose{};
  std::array<float, CARTESIAN_DIM> right_ee_pose{};
  compute_upper_limb_fk(q_feedback, left_ee_pose, right_ee_pose);

  Eigen::Vector3d p_l(left_ee_pose.at(0),left_ee_pose.at(1), left_ee_pose.at(2));
  Eigen::Quaterniond q_l(left_ee_pose.at(6), left_ee_pose.at(3), left_ee_pose.at(4), left_ee_pose.at(5));
  Eigen::Vector3d p_r(right_ee_pose.at(0),right_ee_pose.at(1), right_ee_pose.at(2));
  Eigen::Quaterniond q_r(right_ee_pose.at(6), right_ee_pose.at(3), right_ee_pose.at(4), right_ee_pose.at(5));
  
  //Desired Twists
  Eigen::Vector3d p_l_dot_des(target_left_ee_twist.at(0),target_left_ee_twist.at(1), target_left_ee_twist.at(2));
  Eigen::Vector3d omega_l_des(target_left_ee_twist.at(3), target_left_ee_twist.at(4), target_left_ee_twist.at(5));
  Eigen::Vector3d p_r_dot_des(target_right_ee_twist.at(0),target_right_ee_twist.at(1), target_right_ee_twist.at(2));
  Eigen::Vector3d omega_r_des(target_right_ee_twist.at(3), target_right_ee_twist.at(4), target_right_ee_twist.at(5));
  
  // std::cout << "p_l_des: " << p_l_des << "\n";
  // std::cout << "q_l_des: " << q_l_des.vec() << ' ' << q_l_des.w() << "\n";
  // std::cout << "p_r_des: " << p_r_des << "\n";
  // std::cout << "q_r_des: " << q_r_des.vec() << ' ' << q_l_des.w() << "\n";

  // std::cout << "p_l: " << p_l << "\n";
  // std::cout << "q_l: " << q_l.vec() << ' ' << q_l.w() << "\n";
  // std::cout << "p_r: " << p_r << "\n";
  // std::cout << "q_r: " << q_r.vec() << ' ' << q_l.w() << "\n";

  // std::cout << "p_l_dot_des: " << p_l_dot_des << "\n";
  // std::cout << "omega_l_des: " << omega_l_des << "\n";
  // std::cout << "p_r_dot_des: " << p_r_dot_des << "\n";
  // std::cout << "omega_r_des: " << omega_r_des << "\n";


  //Position and orientation error
  Eigen::Vector3d e_p_l = p_l_des - p_l;
  Eigen::Vector3d e_p_r = p_r_des - p_r;

  q_l_des.normalize();
  q_l.normalize();
  Eigen::Vector3d e_o_l =(q_l_des * q_l.conjugate()).vec();
  q_r_des.normalize();
  q_r.normalize();
  Eigen::Vector3d e_o_r =(q_r_des * q_r.conjugate()).vec();
  // std::cout << "e_p_l: " << e_p_l << "\n";
  // std::cout << "e_p_r: " << e_p_r << "\n";
  // std::cout << "e_o_l: " << e_o_l << "\n";
  // std::cout << "e_o_r: " << e_o_r << "\n";

  //Compute the twist in (3.92) v_in = [p_dot_d + K_p e_p; omega_d + K_o e_o]
  Eigen::VectorXd t_l_in(6);
  t_l_in.head<3>() = p_l_dot_des + k_p * e_p_l;
  t_l_in.tail<3>() = omega_l_des + k_o * e_o_l;

  Eigen::VectorXd t_r_in(6);
  t_r_in.head<3>() = p_r_dot_des + k_p * e_p_r;
  t_r_in.tail<3>() = omega_r_des + k_o * e_o_r;

  // std::cout << "t_l_in: " << t_l_in << "\n";
  // std::cout << "t_r_in: " << t_r_in << "\n";
  
  //Convert to KDL
  //Twists
  std::vector<std::string> endpoints = {
    tip_link_l,
    tip_link_r
  };

  KDL::Twist t_l(KDL::Vector(t_l_in[0], t_l_in[1], t_l_in[2]),
                  KDL::Vector(t_l_in[3], t_l_in[4], t_l_in[5]));
  KDL::Twist t_r(KDL::Vector(t_r_in[0], t_r_in[1], t_r_in[2]),
                  KDL::Vector(t_r_in[3], t_r_in[4], t_r_in[5]));    
  KDL::Twists t_in;
  
  t_in[endpoints[0]] = t_l;
  t_in[endpoints[1]] = t_r;

  //Joints Positions
  KDL::JntArray q_i(UPPER_LIMB_JOINTS_DIM);

  // q_in has the torso joint as last joint value, but here in kdl tree we have it as first joint value, so we remap

  // std::array<int, UPPER_LIMB_JOINTS_DIM> remap = {
  //   14, // torso_joint
  //   0,  // left_shoulder_pitch_joint
  //   1,  // left_shoulder_roll_joint
  //   2,  // left_shoulder_yaw_joint
  //   3,  // left_elbow_joint
  //   4,  // left_wrist_roll_joint
  //   5,  // left_wrist_pitch_joint
  //   6,  // left_wrist_yaw_joint
  //   7,  // right_shoulder_pitch_joint
  //   8,  // right_shoulder_roll_joint
  //   9,  // right_shoulder_yaw_joint
  //   10, // right_elbow_joint
  //   11, // right_wrist_roll_joint
  //   12, // right_wrist_pitch_joint
  //   13  // right_wrist_yaw_joint
  // };

  for (size_t i = 0; i < UPPER_LIMB_JOINTS_DIM; ++i) {
    q_i(i) = q_feedback[i];
  }

  // std::cout << "q_i: ";
  // for (size_t i = 0; i < UPPER_LIMB_JOINTS_DIM; ++i) {
  //     std::cout << q_i(i) << ' '; 
  // }
  //std::cout << "\n";


  //Perform inverse kinematics
  KDL::JntArray q_dot_out(UPPER_LIMB_JOINTS_DIM);
  KDL::TreeIkSolverVel_wdls ik_vel_solver(_h1_2_upper_limb_tree, endpoints);
  ik_vel_solver.setLambda(lambda);

  if(ik_vel_solver.CartToJnt(q_i, t_in, q_dot_out) < 0){
    std::cerr << "Failed ik\n";
    return false;
  }
  else{
    // Fill output array
    for(int i=0; i<UPPER_LIMB_JOINTS_DIM; i++){
      q_dot_output[i]=q_dot_out(i);
    }
    bool has_nan = std::any_of(q_dot_output.begin(), q_dot_output.end(), [](float x) { return std::isnan(x); });
    if(has_nan){
      std::cerr << "Failed ik: NaN detected\n";
      return false;
    }
    // std::cout << "q_dot_out: ";
    // for (size_t i = 0; i < UPPER_LIMB_JOINTS_DIM; ++i) {
    //     std::cout << q_dot_out(i) << ' '; 
    // }
    // std::cout << "\n";
    return true;
  }


  //Call this function in a loop and integrate qdot outside this function. Then, at next iteration, provide
  //the q feedback


}


















bool H1_2_kdl::compute_upper_limb_fk(std::array<float, UPPER_LIMB_JOINTS_DIM> q_in,
                           std::array<float, CARTESIAN_DIM> & left_ee_pose,
                           std::array<float, CARTESIAN_DIM> & right_ee_pose){

                            

  //Convert to KDL
  KDL::JntArray q_i(UPPER_LIMB_JOINTS_DIM);

  // q_in has the torso joint as last joint value, but here in kdl tree we have it as first joint value, so we remap

  // std::array<int, UPPER_LIMB_JOINTS_DIM> remap = {
  //   14, // torso_joint
  //   0,  // left_shoulder_pitch_joint
  //   1,  // left_shoulder_roll_joint
  //   2,  // left_shoulder_yaw_joint
  //   3,  // left_elbow_joint
  //   4,  // left_wrist_roll_joint
  //   5,  // left_wrist_pitch_joint
  //   6,  // left_wrist_yaw_joint
  //   7,  // right_shoulder_pitch_joint
  //   8,  // right_shoulder_roll_joint
  //   9,  // right_shoulder_yaw_joint
  //   10, // right_elbow_joint
  //   11, // right_wrist_roll_joint
  //   12, // right_wrist_pitch_joint
  //   13  // right_wrist_yaw_joint
  // };

  for (size_t i = 0; i < UPPER_LIMB_JOINTS_DIM; ++i) {
    q_i(i) = q_in[i];
  }
  
  KDL::Frame r_pose;
  KDL::Frame l_pose;

  //Setup fk solver
  KDL::TreeFkSolverPos_recursive fk_solver(_h1_2_upper_limb_tree);  

  //Solve fk
  if(fk_solver.JntToCart(q_i, l_pose, tip_link_l) < 0 || fk_solver.JntToCart(q_i, r_pose, tip_link_r) < 0){
    std::cerr << "TreeFkSolverPos_recursive failed" << std::endl;
    return false;
  }

  //Fill output
  double l_quat[4], r_quat[4];
  l_pose.M.GetQuaternion(l_quat[0], l_quat[1], l_quat[2], l_quat[3]);
  r_pose.M.GetQuaternion(r_quat[0], r_quat[1], r_quat[2], r_quat[3]);

  for (int i=0; i<CARTESIAN_DIM; i++){
    if(i<3){
      left_ee_pose.at(i) = l_pose.p.data[i];
      right_ee_pose.at(i) = r_pose.p.data[i];
    }
    else{
      left_ee_pose.at(i) = l_quat[i-3];
      right_ee_pose.at(i) = r_quat[i-3];
    }
  }
  return true;

}


bool H1_2_kdl::extractMinimalSubTree(){

  KDL::Chain chain1, chain2;

  // Extract chains
  if (!_h1_2_tree.getChain(base_link, tip_link_l, chain1)) {
  std::cerr << "Failed to extract chain to " << tip_link_l << "\n";
  return false;
  }

  if (!_h1_2_tree.getChain(base_link, tip_link_r, chain2)) {
  std::cerr << "Failed to extract chain to " << tip_link_r << "\n";
  return false;
  }

  // Initialize the output tree with the base link
  _h1_2_upper_limb_tree = KDL::Tree(base_link);
  std::set<std::string> added_links;
  added_links.insert(base_link);

  // Helper to add a chain to the tree
  auto add_chain = [&](const KDL::Chain& chain, const std::string& root_link) -> bool {
    std::string parent_link = root_link;

    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
      const KDL::Segment& seg = chain.getSegment(i);
      std::string child_link = seg.getName();  // Segment name is usually the child link name

      if (added_links.find(child_link) == added_links.end()) {
        if (!_h1_2_upper_limb_tree.addSegment(seg, parent_link)) {
          std::cerr << "Failed to add segment " << child_link << " to parent " << parent_link << "\n";
          return false;
        }
        added_links.insert(child_link);
      }

      parent_link = child_link;
    }
    return true;
  };

  if (!add_chain(chain1, base_link)) return false;
  if (!add_chain(chain2, base_link)) return false;

  return true;
}

void H1_2_kdl::extract_joint_names_from_tree() {
  _upper_limb_joint_names.clear();
  
  // Get iterator to the base segment
  auto root_it = _h1_2_upper_limb_tree.getSegment(base_link);
  if (root_it == _h1_2_upper_limb_tree.getSegments().end()) {
      std::cerr << "Base link not found in tree: " << base_link << std::endl;
      return;
  }
  
  // Dereference the iterator to get the pair, then access .second for TreeElement
  extract_joint_names_recursive(root_it->second);
}

void H1_2_kdl::extract_joint_names_recursive(const KDL::TreeElement& segment) {
  const KDL::Joint& joint = segment.segment.getJoint();
  if (joint.getType() != KDL::Joint::None) {
      _upper_limb_joint_names.push_back(joint.getName());
  }
  
  for (const auto& child_pair : segment.children) {
      extract_joint_names_recursive((*child_pair).second);
  }
}

std::array<float, UPPER_LIMB_JOINTS_DIM> H1_2_kdl::get_gravity_torques(std::array<float, UPPER_LIMB_JOINTS_DIM> q){
    KDL::Vector gravity(0.0, 0.0, -9.81);

    KDL::ChainDynParam dyn_param_l(_k_chain_l, gravity);
    KDL::ChainDynParam dyn_param_r(_k_chain_r, gravity);


    unsigned int nl = _k_chain_l.getNrOfJoints();
    KDL::JntArray joint_positions_l(nl);
    KDL::JntArray gravity_torques_l(nl);

    unsigned int nr = _k_chain_r.getNrOfJoints();
    KDL::JntArray joint_positions_r(nr);
    KDL::JntArray gravity_torques_r(nr);

    for (unsigned int i = 0; i < nl; ++i) {
      joint_positions_l(i) = q.at(i); 
    }
    for (unsigned int i = 0; i < nr; ++i) {
      joint_positions_r(i) = q.at(i+7); 
    }

    if (dyn_param_l.JntToGravity(joint_positions_l, gravity_torques_l) < 0) {
        std::cerr << "Failed to compute gravity torques!" << std::endl;
    }

    if (dyn_param_r.JntToGravity(joint_positions_r, gravity_torques_r) < 0) {
      std::cerr << "Failed to compute gravity torques!" << std::endl;
    }

    std::array<float, UPPER_LIMB_JOINTS_DIM> gravity_torques;
    for(int i=0; i<UPPER_LIMB_JOINTS_DIM; i++){
      if(i<=6)
        gravity_torques.at(i) = gravity_torques_l(i);
      else
        gravity_torques.at(i) = gravity_torques_r(i-7);
    }
    return gravity_torques;
}


