<!-- GETTING STARTED -->
## Overview
This repo contains  contains the code under development for the wheelchair pushing project.
 

### Execution using docker
If you are using docker and you built the image from [README.md](https://github.com/francescocufino/unitree_h1_2/README.md), supposing that you have already built the code, and that you are connected to the robot, you can execute the following passages to run the program.

1. Run the container, navigate in the build folder you created when building, and run the hand service. The password for sudo is 'user'.
```sh
cd <YOUR_PATH_TO_REPO>/unitree_h1_2
source docker_run.sh
cd ~/cufino_ws_h1_2/build
sudo ./bin/hand_service -s /dev/ttyUSB0
```
If this command fails, substitute `ttyUSB1` or `ttyUSB2` in place of `ttyUSB0`, depending on where the hands have been mounted.
> ⚠️ **Note:** It is fundamental to connect the hands BEFORE running the container, otherwise they are not seen from inside the container.


2. Attach another terminal to the container and run the program
```sh
docker exec -it <CONTAINER_ID> bash
cd ~/cufino_ws_h1_2/build
./bin/pushing_wheelchair eth0
```
The <CONTAINER_ID> of the running container can be seen with `docker ps`.
If the command doesn't work, in place of `eth0` you have to substitute your ethernet network interface. You can see it running
```sh
ifconfig
```
on your machine. This command lists all the network interfaces of your machine giving you the respective information. See the network interface corresponding to the subnet of the robot (broadcast 192.168.123.255).

## Execution without docker
If you are not using docker, supposing that you have already built the code, and that you are connected to the robot, you can execute the following passages to run the program.
1. Navigate in the build folder you created when building, and run the hand service.
```sh
cd <YOUR_PATH_TO_WS>/cufino_ws_h1_2/build
sudo ./bin/hand_service -s /dev/ttyUSB0
```
If this command fails, substitute ttyUSB1 or ttyUSB2 in place of ttyUSB0, depending on where the hands have been mounted.


2. Run the program
```sh
cd <YOUR_PATH_TO_REPO>/cufino_ws_h1_2/build
./bin/pushing_wheelchair eth0
```
If the command doesn't work, in place of eth0 you have to substitute your ethernet network interface. You can see it running
```sh
ifconfig
```
on your machine. This command lists all the network interfaces of your machine giving you the respective information. See the network interface corresponding to the subnet of the robot (broadcast 192.168.123.255).


   
   
   
   
   
   
   
   
   
   
   
   
   
   
   

