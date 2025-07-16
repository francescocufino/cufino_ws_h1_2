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
  std::cout << "Initailizing arms ...";
  float init_time = 2.0f;
  int init_time_steps = static_cast<int>(init_time / control_dt);

  for (int i = 0; i < init_time_steps; ++i) {
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
  if(!arm_initialized){std::cout << "Arms not initialized. Cannot perform arm motion\n"; return;}

  // start control
  int num_time_steps = static_cast<int>(t_f / control_dt);

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
}

void Arm_motion::move_arms_polynomial(std::array<float, UPPER_LIMB_JOINTS_DIM> q_f, float t_f){
  if(!arm_initialized){std::cout << "Arms not initialized. Cannot perform arm motion\n"; return;}

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

  //Planning over the time interval [0, t_f]
  while(t<=t_f){

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

}






void Arm_motion::move_ee_linear(std::array<float, CARTESIAN_DIM> target_left_ee_pose, 
                                std::array<float, CARTESIAN_DIM> target_right_ee_pose, 
                                float t_f){

  if(!arm_initialized){std::cout << "Arms not initialized. Cannot perform arm motion\n"; return;}

  

  //Initial time
  float t = 0;

  //Initial configuration
  std::array<float, UPPER_LIMB_JOINTS_DIM> q_i;

  //TO START FROM LAST COMMANDED CONFIGURATION:
  // for (int i = 0; i < arm_joints.size(); ++i) {
	//   q_i.at(i) = q_cmd.at(i);
  // }

  //TO START FROM ACTUAL CONFIGURATION:
  q_i = get_angles();

  //Initialize q_cmd_ikin for fake feedback
  if(!initialized_q_cmd_ikin){
    q_cmd_ikin = q_i;
    initialized_q_cmd_ikin = true;
  }

  std::array<float, CARTESIAN_DIM> init_left_ee_pose;
  std::array<float, CARTESIAN_DIM> init_right_ee_pose;

  h1_2_kdl.compute_upper_limb_fk(q_i, init_left_ee_pose, init_right_ee_pose);

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


  //Planning over the time interval [0, t_f]
  while(t<=t_f){


    //Actual pose
    q_i = get_angles();

    //FAKE FEEDBACK, THEN REMOVE!
    q_i = q_cmd_ikin;

    std::array<float, CARTESIAN_DIM> l_pose;
    std::array<float, CARTESIAN_DIM> r_pose;
    h1_2_kdl.compute_upper_limb_fk(q_i, l_pose, r_pose);
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

}


bool Arm_motion::set_end_effector_targets(std::array<float, CARTESIAN_DIM> target_left_ee_pose, 
                                          std::array<float, CARTESIAN_DIM> target_right_ee_pose,
                                          std::array<float, 6> target_left_ee_twist,
                                          std::array<float, 6> target_right_ee_twist,
                                          float dt){

  //Get actual angles
  std::array<float, UPPER_LIMB_JOINTS_DIM> q_in = get_angles();

  //FAKE FEEDBACK, THEN REMOVE
  q_in = q_cmd_ikin;

  std::array<float, CARTESIAN_DIM> left_ee, right_ee;
  get_end_effectors_poses(left_ee, right_ee);

  //Initialize q_cmd_ikin

  if(!initialized_q_cmd_ikin){
    q_cmd_ikin = q_in;
    initialized_q_cmd_ikin= true;
  }

  //Perform inverse kinematics to obtain joints commands
  std::array<float, UPPER_LIMB_JOINTS_DIM> q_dot_out{};
  bool ikin_res = h1_2_kdl.compute_upper_limb_ikin(target_left_ee_pose, target_right_ee_pose, 
                                                    target_left_ee_twist, target_right_ee_twist, 
                                                    q_in, q_dot_out,
                                                    100, 100, 1e-3);



  if(ikin_res){
    //Command
    for(int i=0; i<q_dot_out.size(); i++)
      q_cmd_ikin.at(i) = q_cmd_ikin.at(i) + dt*q_dot_out.at(i);

    q_cmd = q_cmd_ikin;

    // Set also dq????

    //FAKE FEEDBACK, NOT SETTING THE POSITION, THEN UNCOMMENT
    //set_upper_limb_joints(q_cmd_ikin);

    //Save commanded data for test
    joint_positions_cmd.push_back(q_cmd_ikin);
    joint_positions_actual.push_back(q_in);
    left_ee_cmd.push_back(target_left_ee_pose);
    right_ee_cmd.push_back(target_right_ee_pose);
    left_ee_actual.push_back(left_ee);
    right_ee_actual.push_back(right_ee);
    left_twist_ee_cmd.push_back(target_left_ee_twist);
    right_twist_ee_cmd.push_back(target_right_ee_twist);

    return true;
  }
  else{
    set_upper_limb_joints(q_in);
    std::cerr << "Failed ikin\n";
    return false;
  }

}



void Arm_motion::set_upper_limb_joints(std::array<float, UPPER_LIMB_JOINTS_DIM> q_target){
  
  if(safety_check(q_target, 0.05)){
    //Set the command equal to the target
    for (int j = 0; j < q_target.size(); ++j) {
      msg->motor_cmd().at(arm_joints.at(j)).q(q_target.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg->motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }
    // send dds msg
    arm_sdk_publisher->Write(*msg);
  }
  else{
    std::cerr << "Safety check not passed. Fixing joints at actual configuration and exiting\n";
    std::array<float, UPPER_LIMB_JOINTS_DIM> q = get_angles();
    //Set the command equal to the target
    for (int j = 0; j < q.size(); ++j) {
      msg->motor_cmd().at(arm_joints.at(j)).q(q.at(j));
      msg->motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg->motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }
    // send dds msg
    arm_sdk_publisher->Write(*msg);
    exit(1);
  } 
 
}



void Arm_motion::stop_arms(){
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
  store_data();
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

template <size_t N>
void Arm_motion::writeCSV(const std::string &filename, const std::vector<std::array<float, N>> &data) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open file: " << filename << std::endl;
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
    std::cout << "Data written to " << filename << std::endl;
}

void Arm_motion::store_data(){
  writeCSV("../h1_2_demo/output/joint_positions_actual.csv", joint_positions_actual);
  writeCSV("../h1_2_demo/output/joint_positions_cmd.csv", joint_positions_cmd);
  writeCSV("../h1_2_demo/output/left_ee_cmd.csv", left_ee_cmd);
  writeCSV("../h1_2_demo/output/right_ee_cmd.csv", right_ee_cmd);
  writeCSV("../h1_2_demo/output/left_ee_actual.csv", left_ee_actual);
  writeCSV("../h1_2_demo/output/right_ee_actual.csv", right_ee_actual);
  writeCSV("../h1_2_demo/output/left_twist_ee_cmd.csv", left_twist_ee_cmd);
  writeCSV("../h1_2_demo/output/right_twist_ee_cmd.csv", right_twist_ee_cmd);

}
















