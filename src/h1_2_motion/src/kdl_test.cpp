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
        h1_2.compute_upper_limb_ikin_clik(l_pose_target, r_pose_target, l_twist, 
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

}
