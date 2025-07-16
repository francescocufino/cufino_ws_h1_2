#include "arm_motion.h"
#include <csignal>



std::unique_ptr<Arm_motion> h1_motion_ptr = std::make_unique<Arm_motion>();

void stop(int){
    h1_motion_ptr->stop_arms();
    return;
}

int main(){

    std::signal(SIGINT, stop);

    //Move arms to initial position
    //h1_motion_ptr->initialize_arms();

    //TEST 0: get joint angles
    std::array<float, UPPER_LIMB_JOINTS_DIM> q = h1_motion_ptr->get_angles();
    std::cout << "Joint angles:\n";
    for(int i=0; i<q.size(); i++){
        std::cout << q.at(i) << ' ';
    }
    std::cout << "\n";
    std::cout << "\n";

    //////////////////////////////////////////////////////////////////////////

/*
    //TEST 1: get end-effector poses
    std::array<float, 7UL> init_left_ee_pose, init_right_ee_pose;
    h1_motion_ptr->get_end_effectors_poses(init_left_ee_pose, init_right_ee_pose);

    std::cout << "Left initial ee pose:\n";
    for(int i=0; i<init_left_ee_pose.size(); i++){
        std::cout << init_left_ee_pose.at(i) << ' ';
    }
    std::cout << "\n";
    std::cout << "Right initial ee pose:\n";
    for(int i=0; i<init_right_ee_pose.size(); i++){
        std::cout << init_right_ee_pose.at(i) << ' ';
    }
    std::cout << "\n";
    std::cout << "\n";


    //////////////////////////////////////////////////////////////////////////


    //TEST 2: Perform fake right ee linear motion

    std::array<float, 7UL> target_left_ee_pose, target_right_ee_pose;
    target_left_ee_pose = init_left_ee_pose;
    target_right_ee_pose = init_right_ee_pose;
    target_right_ee_pose.at(0) = target_right_ee_pose.at(0)+0.1;

    double t = 10;
    h1_motion_ptr->move_ee_linear(target_left_ee_pose, target_right_ee_pose, t);

    std::array<float, 7UL> reached_left_ee_pose, reached_right_ee_pose;
    h1_motion_ptr->get_end_effectors_poses(reached_left_ee_pose, reached_right_ee_pose);

    std::cout << "Left reached ee pose:\n";
    for(int i=0; i<reached_left_ee_pose.size(); i++){
        std::cout << reached_left_ee_pose.at(i) << ' ';
    }
    std::cout << "\n";
    std::cout << "Right reached ee pose:\n";
    for(int i=0; i<reached_right_ee_pose.size(); i++){
        std::cout << reached_right_ee_pose.at(i) << ' ';
    }
    std::cout << "\n";
    std::cout << "Difference on final poses:\n";
    for(int i=0; i<target_left_ee_pose.size(); i++){
        std::cout << target_left_ee_pose.at(i) - reached_left_ee_pose.at(i) << ' ';
    }
    std::cout << "\n";
    for(int i=0; i<target_right_ee_pose.size(); i++){
        std::cout << target_right_ee_pose.at(i) - reached_right_ee_pose.at(i) << ' ';
    }
    std::cout << "\n";
    std::cout << "\n";

    //Go back to initial position
    h1_motion_ptr->move_ee_linear(init_left_ee_pose, init_right_ee_pose, t);


    //////////////////////////////////////////////////////////////////////////


    //TEST 3: Perform fake left linear motion
    target_left_ee_pose = init_left_ee_pose;
    target_right_ee_pose = init_right_ee_pose;
    target_left_ee_pose.at(0) = target_left_ee_pose.at(0)+0.1;

    h1_motion_ptr->move_ee_linear(target_left_ee_pose, target_right_ee_pose, t);

    h1_motion_ptr->get_end_effectors_poses(reached_left_ee_pose, reached_right_ee_pose);

    std::cout << "Left reached ee pose:\n";
    for(int i=0; i<reached_left_ee_pose.size(); i++){
        std::cout << reached_left_ee_pose.at(i) << ' ';
    }
    std::cout << "\n";
    std::cout << "Right reached ee pose:\n";
    for(int i=0; i<reached_right_ee_pose.size(); i++){
        std::cout << reached_right_ee_pose.at(i) << ' ';
    }
    std::cout << "\n";
    std::cout << "Difference on final poses:\n";
    for(int i=0; i<target_left_ee_pose.size(); i++){
        std::cout << target_left_ee_pose.at(i) - reached_left_ee_pose.at(i) << ' ';
    }
    std::cout << "\n";
    for(int i=0; i<target_right_ee_pose.size(); i++){
        std::cout << target_right_ee_pose.at(i) - reached_right_ee_pose.at(i) << ' ';
    }
    std::cout << "\n";
    std::cout << "\n";

    //Go back to initial position
    h1_motion_ptr->move_ee_linear(init_left_ee_pose, init_right_ee_pose, t);

    //////////////////////////////////////////////////////////////////////////

    //TEST 4 Perform fake rotation of righgt hand
    std::array<float, 7UL> actual_left_ee_pose, actual_right_ee_pose;
    h1_motion_ptr->get_end_effectors_poses(actual_left_ee_pose, actual_right_ee_pose);

    Eigen::Quaterniond q_actual_r(actual_right_ee_pose.at(6), actual_right_ee_pose.at(3), actual_right_ee_pose.at(4), actual_right_ee_pose.at(5));

    // Define 15 degrees in radians
    double angle_deg = 15.0;
    double angle_rad = angle_deg * M_PI / 180.0;

    // Rotation around Z axis (mobile frame)
    Eigen::AngleAxisd rotation(angle_rad, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_rotation(rotation);

    // Apply rotation in the MOBILE frame (post-multiply)
    Eigen::Quaterniond q_new = q_actual_r * q_rotation;

    target_left_ee_pose = actual_left_ee_pose;
    target_right_ee_pose = actual_right_ee_pose;
    target_right_ee_pose.at(3) = (float)q_new.x();
    target_right_ee_pose.at(4) = (float)q_new.y();
    target_right_ee_pose.at(5) = (float)q_new.z();
    target_right_ee_pose.at(6) = (float)q_new.w();

    h1_motion_ptr->move_ee_linear(target_left_ee_pose, target_right_ee_pose, t);

*/


}