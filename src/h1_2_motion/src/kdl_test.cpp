#include "h1_2_kdl.h"
#include <cmath>
int main (){
    H1_2_kdl h1_2;
    std::array<float, 7> l_pose;
    std::array<float, 7> r_pose;

    //TEST 1: FIXED POSE IK
    /*


    //Example joint configuration
    std::array<float, UPPER_LIMB_JOINTS_DIM> arm_target_pos = {0.00200272, 0.065944, -0.0749669, 1.43511, 0.273104, 0.0142946, 0.00735462,
        -0.2, -0.00331497, -0.0423436, 0.459666, 0.0179806, -0.483458, -0.0203335, 
        0.0416987};

    //Compute fk
    h1_2.compute_upper_limb_fk(arm_target_pos, l_pose, r_pose);

    std::cout << "Left hand:\n";
    std::cout << "position:\n";
    std::cout << "x: " << l_pose.at(0) << ' ' << "y: " << l_pose.at(1) << ' ' << "z: " << l_pose.at(2) << '\n'; 
    std::cout << "orientation:\n";
    std::cout << "q_x: " << l_pose.at(3) << ' ' << "q_y: " << l_pose.at(4) << ' ' << "q_z: " << l_pose.at(5) << ' ' << "q_w: " << l_pose.at(6) << '\n' << '\n'; 

    std::cout << "Right hand:\n";
    std::cout << "position:\n";
    std::cout << "x: " << r_pose.at(0) << ' ' << "y: " << r_pose.at(1) << ' ' << "z: " << r_pose.at(2) << '\n'; 
    std::cout << "orientation:\n";
    std::cout << "q_x: " << r_pose.at(3) << ' ' << "q_y: " << r_pose.at(4) << ' ' << "q_z: " << r_pose.at(5) << ' ' << "q_w: " << r_pose.at(6) << '\n' << '\n'; 

    //Compute inverse kinematics starting from the pose obtained with fk, initializing with a slight different joint config
    std::array<float, UPPER_LIMB_JOINTS_DIM> q_init = {0.002000, 0.0667, -0.075, 1.43511, 0.273104, 0.0142946, 0.00735462,
        -0.2, -0.00331497, -0.0423436, 0.459666, 0.0179806, -0.483458, -0.0203335, 
        0.0416987};

    std::array<float, UPPER_LIMB_JOINTS_DIM> q_out;
    h1_2.compute_upper_limb_ikin(l_pose, r_pose, q_init, q_out);

    //Check if the output is near to the example joint config used
    std::cout << "Ikin output:\n";
    for(int i=0; i<UPPER_LIMB_JOINTS_DIM; i++)
        std::cout << q_out[i] << ' ';
    */


    //TEST 2: IKIN along a dummy trajectoy, with fake joint feedback data (equal to the commanded ones)
    //Example joint configuration
    /*
    std::array<float, UPPER_LIMB_JOINTS_DIM> q_in = {0.00200272, 0.065944, -0.0749669, 1.43511, 0.273104, 0.0142946, 0.00735462,
        -0.2, -0.00331497, -0.0423436, 0.459666, 0.0179806, -0.483458, -0.0203335, 
        0.0416987};
    std::array<float, UPPER_LIMB_JOINTS_DIM> q_dot_out;


    //Compute fk
    std::array<float, 7> l_pose_init;
    std::array<float, 7> r_pose_init;
    h1_2.compute_upper_limb_fk(q_in, l_pose_init, r_pose_init);

    std::array<float, 7> l_pose_target = l_pose_init;
    std::array<float, 7> r_pose_target = r_pose_init;

    std::array<float, 7> l_pose_reached;
    std::array<float, 7> r_pose_reached;

    

    //Starting from this pose, increase one coordinate of end effectors poses
    float delta = 0.005;
    float dt = 0.01;

    std::array<float, 6> l_twist = {delta/dt,0,delta/dt,0,0,0};
    std::array<float, 6> r_twist = {0,0,0,0,0,0};

    for(int i = 0; i<100; i++){
        //Use fake feedback
        if (i>=97){delta = 0; l_twist.fill(0); r_twist.fill(0);}
        l_pose_target.at(0) = l_pose_target.at(0) + delta; //Increase of 0.0001m at each iteration.

        l_pose_target.at(2) = l_pose_target.at(2) + delta; //Increase of 0.0001m at each iteration.
        h1_2.compute_upper_limb_ikin(l_pose_target, r_pose_target, l_twist, 
                                            r_twist, q_in, q_dot_out,
                                            100, 100, 1e-3);
        for(int i=0; i<q_dot_out.size(); i++)
            q_in.at(i) = q_in.at(i) + dt*q_dot_out.at(i);
        usleep(dt);
    }


    h1_2.compute_upper_limb_fk(q_in, l_pose_reached, r_pose_reached);

    std::cout << "Initial pose:\n";
    for(int i=0; i<l_pose.size(); i++){
        std::cout << l_pose_init.at(i) << ' ';
    }
    std::cout << "\n";
    for(int i=0; i<r_pose.size(); i++){
        std::cout << r_pose_init.at(i) << ' ';
    }

    std::cout << "\n";

    std::cout << "Final target pose:\n";
    for(int i=0; i<l_pose.size(); i++){
        std::cout << l_pose_target.at(i) << ' ';
    }
    std::cout << "\n";
    for(int i=0; i<r_pose.size(); i++){
        std::cout << r_pose_target.at(i) << ' ';
    }

    std::cout << "\n";

    std::cout << "Final reached pose:\n";
    for(int i=0; i<l_pose.size(); i++){
        std::cout << l_pose_reached.at(i) << ' ';
    }
    std::cout << "\n";
    for(int i=0; i<r_pose.size(); i++){
        std::cout << r_pose_reached.at(i) << ' ';
    }
    std::cout << "\n";

    std::cout << "Difference on final pose:\n";
    for(int i=0; i<l_pose.size(); i++){
        std::cout << l_pose_target.at(i) - l_pose_reached.at(i) << ' ';
    }
    std::cout << "\n";
    for(int i=0; i<r_pose.size(); i++){
        std::cout << r_pose_target.at(i) - r_pose_reached.at(i) << ' ';
    }
*/









//TEST3 IKIN along a trajectory

std::array<float, UPPER_LIMB_JOINTS_DIM> q_i = {0.00200272, 0.065944, -0.0749669, 1.43511, 0.273104, 0.0142946, 0.00735462,
    -0.2, -0.00331497, -0.0423436, 0.459666, 0.0179806, -0.483458, -0.0203335, 
    0.0416987};

//time
float t = 0;
float t_f = 10;


//Initial configuration

std::array<float, CARTESIAN_DIM> init_left_ee_pose;
std::array<float, CARTESIAN_DIM> init_right_ee_pose;

h1_2.compute_upper_limb_fk(q_i, init_left_ee_pose, init_right_ee_pose);

std::array<float, CARTESIAN_DIM> target_left_ee_pose = init_left_ee_pose;
std::array<float, CARTESIAN_DIM> target_right_ee_pose = init_right_ee_pose;
target_right_ee_pose.at(1) = target_right_ee_pose.at(1) + 0.2;
// target_right_ee_pose.at(3) = 0; 
// target_right_ee_pose.at(4) = 0;
// target_right_ee_pose.at(5) = 0;
// target_right_ee_pose.at(6) = 1;

// target_left_ee_pose.at(3) = 0; 
// target_left_ee_pose.at(4) = 0;
// target_left_ee_pose.at(5) = 0;
// target_left_ee_pose.at(6) = 1;

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
Eigen::Matrix3d rot = q_l_init.toRotationMatrix().transpose()*q_l_final.toRotationMatrix();
Eigen::AngleAxisd angle_axis_l(rot);
Eigen::Vector3d axis_l = angle_axis_l.axis();
double angle_l_init = 0;
double angle_l_final = angle_axis_l.angle();
Eigen::AngleAxisd angle_axis_r(q_r_init.toRotationMatrix().transpose()*q_r_final.toRotationMatrix());
Eigen::Vector3d axis_r = angle_axis_r.axis();
double angle_r_init = 0;
double angle_r_final = angle_axis_r.angle();


//Planning over the time interval [0, t_f]
while(t<=t_f){
    std::array<float, CARTESIAN_DIM> l_pose;
    std::array<float, CARTESIAN_DIM> r_pose;

    h1_2.compute_upper_limb_fk(q_i, l_pose, r_pose);
    std::cout << "actual left pose:\n";
    for(int i=0; i<l_pose.size(); i++){
        std::cout << l_pose.at(i) << ' ';
    }
    std::cout << "\n";
    std::cout << "actual right pose:\n";
    for(int i=0; i<r_pose.size(); i++){
        std::cout << r_pose.at(i) << ' ';
    }
    std::cout << "\n";
    std::cout << "\n";

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
    //Transform in base frame
    Eigen::Quaterniond q_l_cmd(q_actual_l.toRotationMatrix()* angle_axis_l_cmd.toRotationMatrix());

    double angle_r_cmd = angle_l_init + s*(angle_r_final - angle_r_init);
    Eigen::AngleAxisd angle_axis_r_cmd(angle_r_cmd, axis_r);
    //Transform in base frame
    Eigen::Quaterniond q_r_cmd(q_actual_r.toRotationMatrix()* angle_axis_r_cmd.toRotationMatrix());

    //Angular velocity transformed in base frame
    double angle_l_dot_cmd = s_dot*(angle_l_final - angle_l_init);
    Eigen::Vector3d omega_l_cmd = q_actual_l.toRotationMatrix()*angle_l_dot_cmd*axis_l;
    double angle_r_dot_cmd = s_dot*(angle_r_final - angle_r_init);
    Eigen::Vector3d omega_r_cmd = q_actual_r.toRotationMatrix()*angle_r_dot_cmd*axis_r;
  
    //q_l_cmd = q_l_init;
    //q_r_cmd = q_r_init;

  //Convert to std::array
  std::array<float, CARTESIAN_DIM> left_ee_pose_cmd{(float)p_l_cmd.x(), (float)p_l_cmd.y(), (float)p_l_cmd.z(), (float)q_l_cmd.x(), (float)q_l_cmd.y(), (float)q_l_cmd.z(), (float)q_l_cmd.w()}; 
  std::array<float, CARTESIAN_DIM> right_ee_pose_cmd{(float)p_r_cmd.x(), (float)p_r_cmd.y(), (float)p_r_cmd.z(), (float)q_r_cmd.x(), (float)q_r_cmd.y(), (float)q_r_cmd.z(), (float)q_r_cmd.w()}; 
  std::array<float, 6> left_ee_twist_cmd{(float)p_l_dot_cmd.x(), (float)p_l_dot_cmd.y(), (float)p_l_dot_cmd.z(),(float)omega_l_cmd.x(), (float)omega_l_cmd.y(), (float)omega_l_cmd.z()};
  std::array<float, 6> right_ee_twist_cmd{(float)p_r_dot_cmd.x(), (float)p_r_dot_cmd.y(), (float)p_r_dot_cmd.z(),(float)omega_r_cmd.x(), (float)omega_r_cmd.y(), (float)omega_r_cmd.z()};
  std::array<float, UPPER_LIMB_JOINTS_DIM> q_dot_out;
  h1_2.compute_upper_limb_ikin(left_ee_pose_cmd, right_ee_pose_cmd, left_ee_twist_cmd, 
    right_ee_twist_cmd, q_i, q_dot_out,
    100, 100, 1e-3);
  double control_dt = 0.02;

    for(int i=0; i<q_dot_out.size(); i++)
        q_i.at(i) = q_i.at(i) + control_dt*q_dot_out.at(i);
    // sleep
    usleep(control_dt*1e6);

    t = t + control_dt;
}
std::array<float, CARTESIAN_DIM> final_l_pose;
    std::array<float, CARTESIAN_DIM> final_r_pose;

    h1_2.compute_upper_limb_fk(q_i, final_l_pose, final_r_pose);

std::cout << "Difference on final pose:\n";
    for(int i=0; i<l_pose.size(); i++){
        std::cout << final_l_pose.at(i) - init_left_ee_pose.at(i) << ' ';
    }
    std::cout << "\n";
    for(int i=0; i<r_pose.size(); i++){
        std::cout << final_r_pose.at(i) - init_right_ee_pose.at(i) << ' ';
    }
















}
