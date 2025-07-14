#include "h1_2_kdl.h"

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
    std::array<float, UPPER_LIMB_JOINTS_DIM> q_in = {0.00200272, 0.065944, -0.0749669, 1.43511, 0.273104, 0.0142946, 0.00735462,
        -0.2, -0.00331497, -0.0423436, 0.459666, 0.0179806, -0.483458, -0.0203335, 
        0.0416987};
    std::array<float, UPPER_LIMB_JOINTS_DIM> q_out_old;
    std::array<float, UPPER_LIMB_JOINTS_DIM> q_out;
    std::copy(q_in.begin(), q_in.end(), q_out_old.begin()); 

    //Compute fk
    std::array<float, 7> l_pose_init;
    std::array<float, 7> r_pose_init;
    h1_2.compute_upper_limb_fk(q_in, l_pose_init, r_pose_init);

    std::copy(l_pose_init.begin(), l_pose_init.end(), l_pose.begin()); 
    std::copy(r_pose_init.begin(), r_pose_init.end(), r_pose.begin()); 


    //Starting from this pose, increase one coordinate of end effectors poses
    for(int i = 0; i<5; i++){
        l_pose.at(2) = l_pose.at(2) + 0.0001; //Increase of 0.0001m at each iteration. After 1000 iteration, the final pose
        // std::cout << "Left pose input at iteration :" << i << "\n";
        // for(int j=0; j<CARTESIAN_DIM; j++){
        //     std::cout << l_pose[j] << ' ';
        // }
        // std::cout << "\n";
        //will be increased by 0.0001 * 1000 = 0.1 m
        h1_2.compute_upper_limb_ikin(l_pose, r_pose, q_out_old, q_out);
        if (q_out == q_out_old) {
            std::cout << "Joint configuration did not change at iteration " << i << ".\n";
        }
        std::copy(q_out.begin(), q_out.end(), q_out_old.begin()); 
        // std::cout << "Ikin output iteration :" << i << "\n";
        // for(int j=0; j<UPPER_LIMB_JOINTS_DIM; j++){
        //     std::cout << q_out[j] << ' ';
        // }
        // std::cout << "\n";
        // std::cout << "\n";
    }


    std::cout << "INITIAL:\n";
    std::cout << "Left hand:\n";
    std::cout << "position:\n";
    std::cout << "x: " << l_pose_init.at(0) << ' ' << "y: " << l_pose_init.at(1) << ' ' << "z: " << l_pose_init.at(2) << '\n'; 
    std::cout << "orientation:\n";
    std::cout << "q_x: " << l_pose_init.at(3) << ' ' << "q_y: " << l_pose_init.at(4) << ' ' << "q_z: " << l_pose_init.at(5) << ' ' << "q_w: " << l_pose_init.at(6) << '\n' << '\n'; 

    std::cout << "Right hand:\n";
    std::cout << "position:\n";
    std::cout << "x: " << r_pose_init.at(0) << ' ' << "y: " << r_pose_init.at(1) << ' ' << "z: " << r_pose_init.at(2) << '\n'; 
    std::cout << "orientation:\n";
    std::cout << "q_x: " << r_pose_init.at(3) << ' ' << "q_y: " << r_pose_init.at(4) << ' ' << "q_z: " << r_pose_init.at(5) << ' ' << "q_w: " << r_pose_init.at(6) << '\n' << '\n'; 


    h1_2.compute_upper_limb_fk(q_out, l_pose, r_pose);

    std::cout << "FINAL:\n";
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
