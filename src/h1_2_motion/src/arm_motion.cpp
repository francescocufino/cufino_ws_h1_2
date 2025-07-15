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
  //std::cout << "Press ENTER to init arms ...";
  //std::cin.get();


  // get current joint position
  std::array<float, UPPER_LIMB_JOINTS_DIM> current_jpos{};
  //std::cout<<"Current joint position: ";
  //std::cout << std::endl;
  for (int i = 0; i < arm_joints.size(); ++i) {
	  current_jpos.at(i) = state_msg->motor_state().at(arm_joints.at(i)).q();
    //std::cout << "q" << i << ": " << current_jpos.at(i) << ' ';
  }
  //std::cout << std::endl;

  // set init pos
  std::cout << "Initailizing arms ...";
  float init_time = 2.0f;
  int init_time_steps = static_cast<int>(init_time / control_dt);

  for (int i = 0; i < init_time_steps; ++i) {
    // set weight
    weight = 1.0;
    msg->motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
    float phase = 1.0 * i / init_time_steps;
    //std::cout << "Phase: " << phase << std::endl;

    // set control joints
    for (int j = 0; j < init_pos.size(); ++j) {
      q_cmd.at(j) = init_pos.at(j) * phase + current_jpos.at(j) * (1 - phase);
    }
    if(!initialized_q_cmd){initialized_q_cmd = true;}

    set_upper_limb_joints(q_cmd);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }
  arm_initialized = true;
  //std::cout<<"Reached joint position q: ";
  //std::cout << std::endl;
  for (int i = 0; i < arm_joints.size(); ++i) {
    //std::cout << "q" << i << ": " << state_msg->motor_state().at(arm_joints.at(i)).q() << ' ';
  }
  //std::cout << std::endl << std::endl;
}

  
void Arm_motion::move_arms_integral(std::array<float, UPPER_LIMB_JOINTS_DIM> q_f, float t_f){
  if(!arm_initialized){std::cout << "Arms not initialized. Cannot perform arm motion\n"; return;}

  //std::cout << "Press ENTER to start arm ctrl ..." << std::endl;
  //std::cin.get();

  // start control
  //std::cout << "Start arm ctrl!" << std::endl;
  int num_time_steps = static_cast<int>(t_f / control_dt);

  for (int i = 0; i < num_time_steps; ++i) {
    // update jpos des
    for (int j = 0; j < q_f.size(); ++j) {
      q_cmd.at(j) +=
          std::clamp(q_f.at(j) - q_cmd.at(j),
                     -max_joint_delta, max_joint_delta);
    }
    if(!initialized_q_cmd){initialized_q_cmd = true;}
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
  //std::cout<<"Current joint position q: ";
  //std::cout << std::endl;
  for (int i = 0; i < arm_joints.size(); ++i) {
    //std::cout << "q" << i << ": " << state_msg->motor_state().at(arm_joints.at(i)).q() << ' ';
  }
  //std::cout << std::endl << std::endl;

  //std::cout<<"Planning from initial joint position q_i (last commanded one): ";
  //std::cout << std::endl;
  for (int i = 0; i < arm_joints.size(); ++i) {
	  q_i.at(i) = q_cmd.at(i);
    //std::cout << "q_i" << i << ": " << q_i.at(i) << ' ';
  }
  //std::cout << std::endl << std::endl;

  //std::cout<<"Desired target joint position q_f: ";
  //std::cout << std::endl;
  for (int i = 0; i < arm_joints.size(); ++i) {
    //std::cout << "q_f" << i << ": " << q_f.at(i) << ' ';
  }
  //std::cout << std::endl << std::endl;

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

  //std::cout << "Press ENTER to start arm ctrl poly..." << std::endl;
  //std::cin.get();

  // start control
  //std::cout << "Start arm ctrl poly!" << std::endl;

  //Planning over the time interval [0, t_f]
  while(t<=t_f){

    //Commanding joints
    for (int j = 0; j < q_cmd.size(); ++j) {
      q_cmd.at(j) =a5.at(j)*pow(t,5) + a4.at(j)*pow(t,4) + a3.at(j)*pow(t,3) + a2.at(j)*pow(t,2)+ a1.at(j)*t + a0.at(j);
    }

    if(!initialized_q_cmd){initialized_q_cmd = true;}
    set_upper_limb_joints(q_cmd);

    // sleep
    std::this_thread::sleep_for(sleep_time);

    t = t + control_dt;
  }

  //std::cout<<"Reached joint position q: ";
  //std::cout << std::endl;
  for (int i = 0; i < arm_joints.size(); ++i) {
    //std::cout << "q" << i << ": " << state_msg->motor_state().at(arm_joints.at(i)).q() << ' ';
  }
  //std::cout << std::endl << std::endl;
  

}

void Arm_motion::set_upper_limb_joints(std::array<float, UPPER_LIMB_JOINTS_DIM> q_target){
  
  //Set the command equal to the target
  for (int j = 0; j < q_target.size(); ++j) {
    msg->motor_cmd().at(arm_joints.at(j)).q(q_target.at(j));
    msg->motor_cmd().at(arm_joints.at(j)).dq(dq);
    msg->motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
  }
  // send dds msg
  arm_sdk_publisher->Write(*msg);
}

bool Arm_motion::set_end_effector_targets(std::array<float, CARTESIAN_DIM> target_left_ee_pose, 
                                          std::array<float, CARTESIAN_DIM> target_right_ee_pose,
                                          std::array<float, 6> target_left_ee_twist,
                                          std::array<float, 6> target_right_ee_twist,
                                          float dt){

  //Get actual angles
  std::array<float, UPPER_LIMB_JOINTS_DIM> q_in = get_angles();

  //Initialize q_cmd
  if(!initialized_q_cmd){
    q_cmd = q_in;
    initialized_q_cmd = true;
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
      q_cmd.at(i) = q_cmd.at(i) + dt*q_dot_out.at(i);
      
    // Set also dq????
    set_upper_limb_joints(q_cmd);
  }
  else{
    set_upper_limb_joints(q_in);
    std::cerr << "Failed ikin\n";
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
}

std::array<float, UPPER_LIMB_JOINTS_DIM> Arm_motion::get_angles(){
  while(!first_cb) {std::this_thread::sleep_for(sleep_time);}
  std::array<float, UPPER_LIMB_JOINTS_DIM> q{};
  for (int i = 0; i < q.size(); ++i) {
	  q.at(i) = state_msg->motor_state().at(arm_joints.at(i)).q();
  }
  return q;
}

std::array<float, UPPER_LIMB_JOINTS_DIM> Arm_motion::get_est_torques(){
  while(!first_cb) {std::this_thread::sleep_for(sleep_time);}
  std::array<float, UPPER_LIMB_JOINTS_DIM> tau_est{};
  for (int i = 0; i < tau_est.size(); ++i) {
	  tau_est.at(i) = state_msg->motor_state().at(arm_joints.at(i)).tau_est();
  }
  return tau_est;
}















