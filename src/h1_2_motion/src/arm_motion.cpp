/**
 * @file arm_motion.cpp
 * @brief This contains the implementation of functions for basic arm motion.
 * 
 * @author Francesco Cufino
 * @date 09/02/2024
 * @version 1.0
 */

#include "arm_motion.h"

Arm_motion::Arm_motion(){

  //Start low cmd publisher and subscriber

  msg = std::make_shared<unitree_hg::msg::dds_::LowCmd_>();
  state_msg = std::make_shared<unitree_hg::msg::dds_::LowState_>();
  arm_sdk_publisher.reset(new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(kTopicArmSDK));
  arm_sdk_publisher->InitChannel();
  low_state_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(kTopicState));
  low_state_subscriber->InitChannel(
      [&](const void *msg_ptr) {
          auto s = static_cast<const unitree_hg::msg::dds_::LowState_*>(msg_ptr);
          *state_msg = *s;  // Dereferencing shared_ptr to copy data
          if(!first_cb){first_cb = true;}
      }, 
      1
  );

  //Set gains in command msg
  for (int j = 0; j < arm_joints.size(); ++j) {
  msg->motor_cmd().at(arm_joints.at(j)).kp(kp_array.at(j));
  msg->motor_cmd().at(arm_joints.at(j)).kd(kd_array.at(j));
  }
}




void Arm_motion::initialize_arms(){
  while(!first_cb) {std::this_thread::sleep_for(sleep_time);}
  
  // get current joint position
  std::array<float, UPPER_LIMB_JOINTS_DIM> current_jpos{};

  for (int i = 0; i < arm_joints.size(); ++i) {
	  current_jpos.at(i) = state_msg->motor_state().at(arm_joints.at(i)).q();
  }

  // set init pos
  std::cout << "\033[36m[INFO] Initalizing arms ...\033[0m";
  float init_time = 2.0f;
  int init_time_steps = static_cast<int>(init_time / control_dt);

  for (int i = 0; i < init_time_steps; ++i) {
    if(stop){break;}
    // set weight
    weight = 1.0;
    msg->motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
    float phase = 1.0 * i / init_time_steps;

    // set control joints
    for (int j = 0; j < init_pos.size(); ++j) {
      q_cmd.at(j) = init_pos.at(j) * phase + current_jpos.at(j) * (1 - phase);
    }
    if(!initialized_q_cmd){initialized_q_cmd = true;}
    initialized_q_cmd_ikin = false; //q_cmd_ikin has to be initialized again if other function modify q and it is not updated

    set_upper_limb_joints(q_cmd);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }
  arm_initialized = true;
}

  
void Arm_motion::move_arms_integral(std::array<float, UPPER_LIMB_JOINTS_DIM> q_f, float t_f){
  if(!arm_initialized){std::cout << "\033[31m[ERROR] Arms not initialized. Cannot perform arm motion.\033[0m\n"; return;}

  // start control
  int num_time_steps = static_cast<int>(t_f / control_dt);

  std::cout << "\033[36m[INFO] Performing arms motion...\033[0m\n";

  for (int i = 0; i < num_time_steps; ++i) {
    // update jpos des
    for (int j = 0; j < q_f.size(); ++j) {
      q_cmd.at(j) +=
          std::clamp(q_f.at(j) - q_cmd.at(j),
                     -max_joint_delta, max_joint_delta);
    }
    if(!initialized_q_cmd){initialized_q_cmd = true;}
    initialized_q_cmd_ikin = false; //q_cmd_ikin has to be initialized again if other function modify q and it is not updated
    set_upper_limb_joints(q_cmd);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "\033[32m[INFO] Arms motion concluded.\033[0m\n";

}

void Arm_motion::move_arms_polynomial(std::array<float, UPPER_LIMB_JOINTS_DIM> q_f, float t_f){
  if(!arm_initialized){std::cout << "\033[31m[ERROR] Arms not initialized. Cannot perform arm motion.\033[0m\n"; return;}

  //Initial time
  float t = 0;

  //Initial configuration q_i
  std::array<float, UPPER_LIMB_JOINTS_DIM> q_i{};

  for (int i = 0; i < arm_joints.size(); ++i) {
	  q_i.at(i) = q_cmd.at(i);
  }

  //Planning parameters
  std::array<float, UPPER_LIMB_JOINTS_DIM> a0{}, a1{}, a2{}, a3{}, a4{}, a5{};
  for (int j = 0; j < q_f.size(); ++j) {
    a0.at(j) = q_i.at(j);
    a1.at(j) = 0;
    a2.at(j) = 0;
    a3.at(j) = (10*q_f.at(j) - 10*q_i.at(j))/pow(t_f, 3);
    a4.at(j) = (-15*q_f.at(j) + 15*q_i.at(j))/pow(t_f, 4);
    a5.at(j) = (6*q_f.at(j) - 6*q_i.at(j))/pow(t_f, 5);
  }

  std::cout << "\033[36m[INFO] Performing arms motion...\033[0m\n";

  //Planning over the time interval [0, t_f]
  while(t<=t_f && !stop){

    //Commanding joints
    for (int j = 0; j < q_cmd.size(); ++j) {
      q_cmd.at(j) =a5.at(j)*pow(t,5) + a4.at(j)*pow(t,4) + a3.at(j)*pow(t,3) + a2.at(j)*pow(t,2)+ a1.at(j)*t + a0.at(j);
    }

    if(!initialized_q_cmd){initialized_q_cmd = true;}
    initialized_q_cmd_ikin = false; //q_cmd_ikin has to be initialized again if other function modify q and it is not updated

    set_upper_limb_joints(q_cmd);

    // sleep
    std::this_thread::sleep_for(sleep_time);

    t = t + control_dt;
  }
  std::cout << "\033[32m[INFO] Arms motion concluded.\033[0m\n";


}






void Arm_motion::move_ee_linear(std::array<float, CARTESIAN_DIM> target_left_ee_pose, 
                                std::array<float, CARTESIAN_DIM> target_right_ee_pose,
                                bool cmd_orientation, 
                                float t_f){

  if(!arm_initialized){std::cout << "\033[31m[ERROR] Arms not initialized. Cannot perform arm motion \033[0m\n"; return;}


  //Initial time
  float t = 0;

  //TO START FROM LAST COMMANDED CONFIGURATION:
  // for (int i = 0; i < arm_joints.size(); ++i) {
	//   q_cmd_ikin.at(i) = q_cmd.at(i);
  // }

  //Initialize q_cmd_ikin
  if(!initialized_q_cmd_ikin){
    q_cmd_ikin = get_angles(); //TO START FROM ACTUAL CONFIGURATION:
    initialized_q_cmd_ikin = true;
  }

  //Initialize end-effector poses
  std::array<float, CARTESIAN_DIM> init_left_ee_pose;
  std::array<float, CARTESIAN_DIM> init_right_ee_pose;

  h1_2_kdl.compute_upper_limb_fk(q_cmd_ikin, init_left_ee_pose, init_right_ee_pose);

  //Time law planning parameters
  double s = 0;
  double s_dot;
  double s_i = 0; 
  double s_f = 1; 
  double a0, a1, a2, a3, a4, a5;
  
  a0 = s_i;
  a1 = 0;
  a2 = 0;
  a3 = (10*s_f - 10*s_i)/pow(t_f, 3);
  a4 = (-15*s_f + 15*s_i)/pow(t_f, 4);
  a5 = (6*s_f - 6*s_i)/pow(t_f, 5);

  //Geometrical planning parameters
  //Initial and final positions
  Eigen::Vector3d p_l_init(init_left_ee_pose.at(0), init_left_ee_pose.at(1), init_left_ee_pose.at(2) );
  Eigen::Vector3d p_r_init(init_right_ee_pose.at(0), init_right_ee_pose.at(1), init_right_ee_pose.at(2) );
  Eigen::Vector3d p_l_final(target_left_ee_pose.at(0), target_left_ee_pose.at(1), target_left_ee_pose.at(2) );
  Eigen::Vector3d p_r_final(target_right_ee_pose.at(0), target_right_ee_pose.at(1), target_right_ee_pose.at(2) );

  //Initial and final quaternions
  Eigen::Quaterniond q_l_init(init_left_ee_pose.at(6),init_left_ee_pose.at(3),init_left_ee_pose.at(4),init_left_ee_pose.at(5));
  Eigen::Quaterniond q_r_init(init_right_ee_pose.at(6),init_right_ee_pose.at(3),init_right_ee_pose.at(4),init_right_ee_pose.at(5));
  Eigen::Quaterniond q_l_final(target_left_ee_pose.at(6),target_left_ee_pose.at(3),target_left_ee_pose.at(4),target_left_ee_pose.at(5));
  Eigen::Quaterniond q_r_final(target_right_ee_pose.at(6),target_right_ee_pose.at(3),target_right_ee_pose.at(4),target_right_ee_pose.at(5));
  
  //Initial and final angle-axis
  Eigen::AngleAxisd angle_axis_l(q_l_init.toRotationMatrix().transpose()*q_l_final.toRotationMatrix());
  Eigen::Vector3d axis_l = angle_axis_l.axis();
  double angle_l_init = 0;
  double angle_l_final = angle_axis_l.angle();
  Eigen::AngleAxisd angle_axis_r(q_r_init.toRotationMatrix().transpose()*q_r_final.toRotationMatrix());
  Eigen::Vector3d axis_r = angle_axis_r.axis();
  double angle_r_init = 0;
  double angle_r_final = angle_axis_r.angle();


  std::cout << "\033[36m[INFO] Performing arms motion...\033[0m\n";

  //Planning over the time interval [0, t_f]
  while(t<=t_f && !stop){

    //Actual pose
    std::array<float, CARTESIAN_DIM> l_pose;
    std::array<float, CARTESIAN_DIM> r_pose;
    h1_2_kdl.compute_upper_limb_fk(q_cmd_ikin, l_pose, r_pose); //Fake feedback. To use real, use get_angles() instead of q_cmd_ikin
    Eigen::Quaterniond q_actual_l(l_pose.at(6),l_pose.at(3),l_pose.at(4),l_pose.at(5));
    Eigen::Quaterniond q_actual_r(r_pose.at(6),r_pose.at(3),r_pose.at(4),r_pose.at(5));

    //Time law planning
    s = a5*pow(t,5) + a4*pow(t,4) + a3*pow(t,3) + a2*pow(t,2)+ a1*t + a0;
    s_dot = 5*a5*pow(t,4) + 4*a4*pow(t,3) + 3*a3*pow(t,2) + 2*a2*t + a1;

    //Trajectory planning

    //Position
    Eigen::Vector3d p_l_cmd = p_l_init + s*(p_l_final - p_l_init);
    Eigen::Vector3d p_r_cmd = p_r_init + s*(p_r_final - p_r_init);

    //Linear velocity
    Eigen::Vector3d p_l_dot_cmd = s_dot*(p_l_final - p_l_init);
    Eigen::Vector3d p_r_dot_cmd = s_dot*(p_r_final - p_r_init);

    //Orientation
    double angle_l_cmd = angle_l_init + s*(angle_l_final - angle_l_init);
    Eigen::AngleAxisd angle_axis_l_cmd(angle_l_cmd, axis_l);
    Eigen::Quaterniond q_l_cmd(q_actual_l.toRotationMatrix()* angle_axis_l_cmd.toRotationMatrix()); //transformed in base frame

    double angle_r_cmd = angle_l_init + s*(angle_r_final - angle_r_init);
    Eigen::AngleAxisd angle_axis_r_cmd(angle_r_cmd, axis_r);
    Eigen::Quaterniond q_r_cmd(q_actual_r.toRotationMatrix()* angle_axis_r_cmd.toRotationMatrix()); //transformed in base frame

    //Angular velocity
    double angle_l_dot_cmd = s_dot*(angle_l_final - angle_l_init);
    Eigen::Vector3d omega_l_cmd = q_actual_l.toRotationMatrix()*angle_l_dot_cmd*axis_l; //transformed in base frame 
    double angle_r_dot_cmd = s_dot*(angle_r_final - angle_r_init);
    Eigen::Vector3d omega_r_cmd = q_actual_r.toRotationMatrix()*angle_r_dot_cmd*axis_r; //transformed in base frame

    //If orientation is not commanded, give the initial one
    if(!cmd_orientation){
      q_l_cmd = q_l_init;
      q_r_cmd = q_r_init;
      omega_l_cmd = {0,0,0};
      omega_r_cmd = {0,0,0};
    }
    

    //Convert to std::array
    std::array<float, CARTESIAN_DIM> left_ee_pose_cmd{(float)p_l_cmd.x(), (float)p_l_cmd.y(), (float)p_l_cmd.z(), (float)q_l_cmd.x(), (float)q_l_cmd.y(), (float)q_l_cmd.z(), (float)q_l_cmd.w()}; 
    std::array<float, CARTESIAN_DIM> right_ee_pose_cmd{(float)p_r_cmd.x(), (float)p_r_cmd.y(), (float)p_r_cmd.z(), (float)q_r_cmd.x(), (float)q_r_cmd.y(), (float)q_r_cmd.z(), (float)q_r_cmd.w()}; 
    std::array<float, 6> left_ee_twist_cmd{(float)p_l_dot_cmd.x(), (float)p_l_dot_cmd.y(), (float)p_l_dot_cmd.z(),(float)omega_l_cmd.x(), (float)omega_l_cmd.y(), (float)omega_l_cmd.z()};
    std::array<float, 6> right_ee_twist_cmd{(float)p_r_dot_cmd.x(), (float)p_r_dot_cmd.y(), (float)p_r_dot_cmd.z(),(float)omega_r_cmd.x(), (float)omega_r_cmd.y(), (float)omega_r_cmd.z()};

    
    set_end_effector_targets(left_ee_pose_cmd, right_ee_pose_cmd, left_ee_twist_cmd, right_ee_twist_cmd, control_dt);



    // sleep
    std::this_thread::sleep_for(sleep_time);

    t = t + control_dt;
  }
  std::cout << "\033[32m[INFO] Arms motion concluded.\033[0m\n";
  //initialized_q_cmd_ikin = false;
  store_data();

}


bool Arm_motion::set_end_effector_targets(std::array<float, CARTESIAN_DIM> target_left_ee_pose, 
                                          std::array<float, CARTESIAN_DIM> target_right_ee_pose,
                                          std::array<float, 6> target_left_ee_twist,
                                          std::array<float, 6> target_right_ee_twist,
                                          float dt){

  //Initialize q_cmd_ikin

  if(!initialized_q_cmd_ikin){
    q_cmd_ikin = get_angles();;
    initialized_q_cmd_ikin= true;
  }

  //Perform inverse kinematics to obtain joints commands
  std::array<float, UPPER_LIMB_JOINTS_DIM> q_dot_out{};
  bool ikin_res = h1_2_kdl.compute_upper_limb_ikin(target_left_ee_pose, target_right_ee_pose, 
                                                    target_left_ee_twist, target_right_ee_twist, 
                                                    q_cmd_ikin, q_dot_out,
                                                    100, 15, 1e-3); //Fake feedback. To use real, use get_angles() instead of q_cmd_ikin


  if(ikin_res){
    //Command
    for(int i=0; i<q_dot_out.size(); i++)
      q_cmd_ikin.at(i) = q_cmd_ikin.at(i) + dt*q_dot_out.at(i);

    q_cmd = q_cmd_ikin;

    // Set also dq????

    //FAKE FEEDBACK, NOT SETTING THE POSITION, THEN UNCOMMENT
    set_upper_limb_joints(q_cmd_ikin);

    //Get actual quantities for test
    std::array<float, UPPER_LIMB_JOINTS_DIM> q = get_angles();
    std::array<float, CARTESIAN_DIM> left_ee, right_ee;
    get_end_effectors_poses(left_ee, right_ee);

    //Save commanded data for test
    joint_positions_cmd.push_back(q_cmd_ikin);
    joint_positions_actual.push_back(q);
    left_ee_cmd.push_back(target_left_ee_pose);
    right_ee_cmd.push_back(target_right_ee_pose);
    left_ee_actual.push_back(left_ee);
    right_ee_actual.push_back(right_ee);
    left_twist_ee_cmd.push_back(target_left_ee_twist);
    right_twist_ee_cmd.push_back(target_right_ee_twist);
    force_ee.push_back(h1_2_kdl.compute_ee_forces(get_angles(), get_est_torques(), 1));


    return true;
  }
  else{
    set_upper_limb_joints(get_angles());
    std::cerr << "\033[31m[ERROR] Failed ikin.\033[0m\n";
    return false;
  }

}



void Arm_motion::set_upper_limb_joints(std::array<float, UPPER_LIMB_JOINTS_DIM> q_target){

  bool safety = safety_check(q_target, 0.2);
  bool limits = limits_check(q_target);
  
  if(safety && limits){
    //Set the command equal to the target
    for (int j = 0; j < q_target.size(); ++j) {
      msg->motor_cmd().at(arm_joints.at(j)).q(q_target.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg->motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
      msg->motor_cmd().at(arm_joints.at(j)).mode() = 1; 
    }
    // send dds msg
    arm_sdk_publisher->Write(*msg);
  }
  else{
    if(!safety)
      std::cerr << "\033[31m[ERROR] Safety check not passed.\033[0m\n";

    if(!limits)
      std::cerr << "\033[31m[ERROR] Limit check not passed.\033[0m\n";

    std::cerr << "\033[31m[ERROR] Fixing joints at actual configuration and exiting.\033[0m\n";    
    std::array<float, UPPER_LIMB_JOINTS_DIM> q = get_angles();
    //Set the command equal to the target
    for (int j = 0; j < q.size(); ++j) {
      msg->motor_cmd().at(arm_joints.at(j)).q(q.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg->motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }
    // send dds msg
    arm_sdk_publisher->Write(*msg);
    store_data();
    exit(1);
  } 
 
}



void Arm_motion::stop_arms(){
  stop = true;
  std::cout << "\033[36m[INFO] Stop flag activated, exiting from control loops.\033[0m\n";
  /*
  std::cout << "Stopping arm ctrl: returning to initial position ...\n";
  move_arms_polynomial(init_pos, 2);
  std::cout << "Stopping arm ctrl ...\n";
  float stop_time = 2.0f;
  int stop_time_steps = static_cast<int>(stop_time / control_dt);

  for (int i = 0; i < stop_time_steps; ++i) {
    // increase weight
    weight -= delta_weight;
    weight = std::clamp(weight, 0.f, 1.f);

    // set weight
    msg->motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);

    // send dds msg
    arm_sdk_publisher->Write(*msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }
  */
  //store_data();
}

void Arm_motion::admittance_control(std::array<float, 2> left_des_force, std::array<float, 4> left_inertia, std::array<float, 4> left_damping,
                                    std::array<float, 2> right_des_force, std::array<float, 4> right_inertia, std::array<float, 4> right_damping,
                                    double dt){

  std::array<float, CARTESIAN_DIM> left_ee_pos;
  std::array<float, CARTESIAN_DIM> right_ee_pos;
  std::array<float, 6> left_ee_twist;
  std::array<float, 6> right_ee_twist;

  //Initialize admittance
  if(!init_adm){
    get_end_effectors_poses(left_ee_pos, right_ee_pos);
    x_l_adm = {left_ee_pos.at(0), left_ee_pos.at(1)};
    x_r_adm = {right_ee_pos.at(0), right_ee_pos.at(1)};
    init_adm=true;
  }                                      
  //Get left and right force
  std::array<float, 12UL> force = h1_2_kdl.compute_ee_forces(get_angles(), get_est_torques(), 1);
  std::array<float, 2> left_force={force.at(0), force.at(1)};
  std::array<float, 2> right_force={force.at(6), force.at(7)};

  std::array<float, 2> left_delta_force;
  std::array<float, 2> right_delta_force;

  //Compute force error
  for (int i=0; i<left_delta_force.size(); i++){
    left_delta_force.at(i) = left_des_force.at(i) - left_force.at(i);
    right_delta_force.at(i) = right_des_force.at(i) - right_force.at(i);
  }
  
  //Get resulting accelerations from admittance filter
  std::array<float, 2> x_l_ddot_adm = {};
  std::array<float, 2> x_r_ddot_adm = {};
  h1_2_kdl.admittance_filter_2d(x_l_dot_adm,x_l_ddot_adm,left_delta_force, left_inertia, left_damping,
                                x_r_dot_adm,x_r_ddot_adm,right_delta_force, right_inertia, right_damping);
  
  //Integrate
  for(int i=0; i<x_l_dot_adm.size(); i++){
    x_l_dot_adm.at(i) = x_l_dot_adm.at(i) + x_l_ddot_adm.at(i) * dt;
    x_r_dot_adm.at(i) = x_r_dot_adm.at(i) + x_r_ddot_adm.at(i) * dt;

    x_l_adm.at(i) = x_l_adm.at(i) + x_l_dot_adm.at(i) * dt;
    x_r_adm.at(i) = x_r_adm.at(i) + x_r_dot_adm.at(i) * dt;

  }
  //Update reference position and twist
  left_ee_pos.at(0) = x_l_adm.at(0); left_ee_pos.at(1) = x_l_adm.at(1); 
  right_ee_pos.at(0) = x_r_adm.at(0); right_ee_pos.at(1) = x_r_adm.at(1); 

  left_ee_twist.at(0) = x_l_dot_adm.at(0); left_ee_twist.at(1) = x_l_dot_adm.at(1); 
  right_ee_twist.at(0) = x_r_dot_adm.at(0); right_ee_twist.at(1) = x_r_dot_adm.at(1); 

  set_end_effector_targets(left_ee_pos, right_ee_pos, left_ee_twist, right_ee_twist, dt);

  
}


std::array<float, UPPER_LIMB_JOINTS_DIM> Arm_motion::get_angles(){
  while(!first_cb) {std::this_thread::sleep_for(sleep_time);}
  std::array<float, UPPER_LIMB_JOINTS_DIM> q{};
  for (int i = 0; i < q.size(); ++i) {
	  q.at(i) = state_msg->motor_state().at(arm_joints.at(i)).q();
  }
  return q;
}

void Arm_motion::get_end_effectors_poses(std::array<float, CARTESIAN_DIM>& left_ee_pose, 
                              std::array<float, CARTESIAN_DIM>& right_ee_pose){

  std::array<float, UPPER_LIMB_JOINTS_DIM> q = get_angles();
  h1_2_kdl.compute_upper_limb_fk(q, left_ee_pose, right_ee_pose);

  }

std::array<float, UPPER_LIMB_JOINTS_DIM> Arm_motion::get_est_torques(){
  while(!first_cb) {std::this_thread::sleep_for(sleep_time);}
  std::array<float, UPPER_LIMB_JOINTS_DIM> tau_est{};
  for (int i = 0; i < tau_est.size(); ++i) {
	  tau_est.at(i) = state_msg->motor_state().at(arm_joints.at(i)).tau_est();
  }
  return tau_est;
}

bool Arm_motion::safety_check(std::array<float, UPPER_LIMB_JOINTS_DIM> q_target, double delta){
  std::array<float, UPPER_LIMB_JOINTS_DIM> q = get_angles();
  for(int i=0; i<q.size(); i++){
    if(abs(q.at(i) - q_target.at(i)) > delta){
      return false;
    }
  }
  return true;
}

bool Arm_motion::limits_check(std::array<float, UPPER_LIMB_JOINTS_DIM> q_target){
  for(int i=0; i<q_target.size(); i++){
    if(q_target.at(i) <= q_lb.at(i) || q_target.at(i) >= q_ub.at(i)){
      return false;
    }
  }
  return true;
}


template <size_t N>
void Arm_motion::writeCSV(const std::string &filename, const std::vector<std::array<float, N>> &data) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "\033[31m[ERROR]Could not open file: " << filename << "\033[0m" << std::endl;
        return;
    }

    //file << std::fixed << std::setprecision(1);
    for (const auto &row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1)
                file << ",";
        }
        file << std::endl;
    }
    

    file.close();
    std::cout << "\033[32m[INFO] Data written to " << filename << "\033[0m" << std::endl;
}

void Arm_motion::store_data(){
  writeCSV("../src/h1_2_demo/output/joint_positions_actual.csv", joint_positions_actual);
  writeCSV("../src/h1_2_demo/output/joint_positions_cmd.csv", joint_positions_cmd);
  writeCSV("../src/h1_2_demo/output/left_ee_cmd.csv", left_ee_cmd);
  writeCSV("../src/h1_2_demo/output/right_ee_cmd.csv", right_ee_cmd);
  writeCSV("../src/h1_2_demo/output/left_ee_actual.csv", left_ee_actual);
  writeCSV("../src/h1_2_demo/output/right_ee_actual.csv", right_ee_actual);
  writeCSV("../src/h1_2_demo/output/left_twist_ee_cmd.csv", left_twist_ee_cmd);
  writeCSV("../src/h1_2_demo/output/right_twist_ee_cmd.csv", right_twist_ee_cmd);
  writeCSV("../src/h1_2_demo/output/force_ee.csv", force_ee);


}
















