#include "h1_2_kdl.h"

int main (){
    H1_2_kdl h1_2;
    std::array<float, UPPER_LIMB_JOINTS_DIM> arm_target_pos = {0.00200272, 0.065944, -0.0749669, 1.43511, 0.273104, 0.0142946, 0.00735462,
        -0.2, -0.00331497, -0.0423436, 0.459666, 0.0179806, -0.483458, -0.0203335, 
        0.0416987};

    std::array<float, 7> l_pose;
    std::array<float, 7> r_pose;

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
}
