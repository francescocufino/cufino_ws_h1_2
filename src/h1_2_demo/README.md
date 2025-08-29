<!-- GETTING STARTED -->
## Overview
This repo contains an example demo based on h1_2_motion to perform whole body motion.
 

## Execution
0. You can decide the routine of the demo editing the file routine
```sh
nano <YOUR_PATH_TO_WS>/cufino_ws_h1_2/h1_2_demo/config/routine.txt
```
and write `shake` for shake hand, `wave` for waving, `fist_bump` for fist bump for whole body motion.

### Execution using docker
If you are using docker and you built the image from [README.md](https://github.com/francescocufino/unitree_h1_2/README.md), supposing that you have already built the code, and that you are connected to the robot, you can execute the following passages to run the program.

1. Run the container, navigate in the build folder you created when building, and run the hand service. The password for sudo is 'user'.
```sh
docker start unitree_sdk2_container_2
cd ~/cufino_ws_h1_2/src
sudo ./run_drivers.sh
```

2. Attach another terminal to the container and run the program. For the pushing test, run
```sh
docker exec -it unitree_sdk2_container_2 bash
cd ~/cufino_ws_h1_2/build
./bin/motion_test eth0
```
Press ctrl+c to make the robot stop

For wave demo, instead, run ``./bin/whole_body_motion eth0``




   
   
   
   
   
   
   
   
   
   
   
   
   
   
   

