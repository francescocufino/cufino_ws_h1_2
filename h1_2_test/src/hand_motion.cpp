/**
 * @file hand_motion.cpp
 * @brief This contains the implementation of functions for basic inspire hands motion.
 * 
 * @author Francesco Cufino
 * @date 09/02/2024
 * @version 1.0
 */

#include "hand_motion.h"

Hand_motion::Hand_motion(){
    this->InitDDS_();
}

void Hand_motion::move_hands(std::array<float, 12>& angles){
    for(size_t i(0); i<12; i++){
        cmd.cmds()[i].q() = angles.at(i);
    }
    handcmd->Write(cmd);
    usleep(100000);

}

std::array<float, 6> Hand_motion::getRightQ(){
    std::lock_guard<std::mutex> lock(mtx);
    std::array<float, 6> q;
    for(size_t i(0); i<6; i++){
        q.at(i) = state.states()[i].q();
    }
    return q;
}

std::array<float, 6> Hand_motion::getLeftQ(){
    std::lock_guard<std::mutex> lock(mtx);
    std::array<float, 6> q;
    for(size_t i(0); i<6; i++){
        q.at(i) = state.states()[i+6].q();
    }
    return q;
}

std::array<float, 12> Hand_motion::get_angles(){
    std::array<float, 6> q_r;
    std::array<float, 6> q_l;
    std::array<float, 12> q;
    q_r = this->getRightQ();
    q_l = this->getLeftQ();
    std::copy(q_r.begin(), q_r.end(), q.begin());
    std::copy(q_l.begin(), q_l.end(), q.begin() + q_r.size());
    return q;
}


void Hand_motion::InitDDS_(){
    handcmd = std::make_shared<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>>(
        "rt/inspire/cmd");
    handcmd->InitChannel();
    cmd.cmds().resize(12);
    handstate = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>>(
        "rt/inspire/state");
    handstate->InitChannel([this](const void *message){
        std::lock_guard<std::mutex> lock(mtx);
        state = *(unitree_go::msg::dds_::MotorStates_*)message;
    });
    state.states().resize(12);
}


/**
 * Main Function
 */
/*
int main(int argc, char** argv)
{
    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     H1 Hand Example      \n\n";

    // Target label
    std::string label = "close"; // You change this value to other labels

    // Initialize the DDS Channel
    std::string networkInterface = argc > 1 ? argv[1] : "";
    unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);

    // Create the H1 Hand Controller
    auto h1hand = std::make_shared<Hand_motion>();

    int cnt = 0;
    while (true)
    {
        usleep(100000);
        if(cnt++ % 10 == 0)
            label = label == "close" ? "open" : "close";
        h1hand->ctrl(label); // Control the hand
        std::cout << "-- Hand State --\n";
        std::cout << " R: " << h1hand->getRightQ().transpose() << std::endl;
        std::cout << " L: " << h1hand->getLeftQ().transpose() << std::endl;
        std::cout << "\033[3A"; // Move cursor up 3 lines
    }

    return 0;
}
*/