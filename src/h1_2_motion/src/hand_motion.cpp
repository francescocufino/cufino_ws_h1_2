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

void Hand_motion::move_hands(std::array<float, HANDS_JOINTS_DIM>& angles){
    for(size_t i(0); i<HANDS_JOINTS_DIM; i++){
        if(i < L_HAND_JOINTS_DIM)
            cmd_l.angle_set()[i] = angles.at(i);
        else
            cmd_r.angle_set()[i - L_HAND_JOINTS_DIM] = angles.at(i);
    }
    handcmd_l->Write(cmd_l);
    handcmd_r->Write(cmd_r);
    usleep(1000000);

}

std::array<float, R_HAND_JOINTS_DIM> Hand_motion::getRightQ(){
    std::lock_guard<std::mutex> lock(mtx);
    std::array<float, R_HAND_JOINTS_DIM> q;
    for(size_t i(0); i<R_HAND_JOINTS_DIM; i++){
        q.at(i) = state_r.angle_act()[i] ;
    }
    return q;
}

std::array<float, L_HAND_JOINTS_DIM> Hand_motion::getLeftQ(){
    std::lock_guard<std::mutex> lock(mtx);
    std::array<float, L_HAND_JOINTS_DIM> q;
    for(size_t i(0); i<L_HAND_JOINTS_DIM; i++){
        q.at(i) = state_l.angle_act()[i] ;
    }
    return q;
}

std::array<float, HANDS_JOINTS_DIM> Hand_motion::get_angles(){
    std::array<float, R_HAND_JOINTS_DIM> q_r;
    std::array<float, L_HAND_JOINTS_DIM> q_l;
    std::array<float, HANDS_JOINTS_DIM> q;
    q_r = this->getRightQ();
    q_l = this->getLeftQ();
    std::copy(q_r.begin(), q_r.end(), q.begin());
    std::copy(q_l.begin(), q_l.end(), q.begin() + q_r.size());
    return q;
}


void Hand_motion::InitDDS_(){
    handcmd_l = std::make_shared<unitree::robot::ChannelPublisher<inspire::inspire_hand_ctrl>>("rt/inspire_hand/ctrl/l");
    handcmd_l->InitChannel();
    handtouch_l = std::make_shared<unitree::robot::ChannelSubscriber<inspire::inspire_hand_touch>>("rt/inspire_hand/touch/l");
    handstate_l = std::make_shared<unitree::robot::ChannelSubscriber<inspire::inspire_hand_state>>("rt/inspire_hand/state/l");
    handcmd_l->InitChannel();
    handstate_l->InitChannel([this](const void *message){
        state_l = *(inspire::inspire_hand_state*)message; 
    });
    handtouch_l->InitChannel([this](const void *message)
                           {
    touch_l = *(inspire::inspire_hand_touch*)message; 
    });
    cmd_l.angle_set().resize(L_HAND_JOINTS_DIM);
    cmd_l.mode(0b0001);


    handcmd_r = std::make_shared<unitree::robot::ChannelPublisher<inspire::inspire_hand_ctrl>>("rt/inspire_hand/ctrl/r");
    handcmd_r->InitChannel();
    handtouch_r = std::make_shared<unitree::robot::ChannelSubscriber<inspire::inspire_hand_touch>>("rt/inspire_hand/touch/r");
    handstate_r = std::make_shared<unitree::robot::ChannelSubscriber<inspire::inspire_hand_state>>("rt/inspire_hand/state/r");
    handcmd_r->InitChannel();
    handstate_r->InitChannel([this](const void *message)                            {
    state_r = *(inspire::inspire_hand_state*)message; 
    });
    handtouch_r->InitChannel([this](const void *message)
                           {
    touch_r = *(inspire::inspire_hand_touch*)message; 
    });
    cmd_r.angle_set().resize(R_HAND_JOINTS_DIM);
    cmd_r.mode(0b0001);
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