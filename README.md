<!-- GETTING STARTED -->
## Overview
This repo contains the code for cufino workspace on unitree h1_2. This code requires that unitree_sdk2 is installed.


## Compilation and execution

1. Clone the repo (complete the command)
```sh
git clone https://github.com/francescocufino/cufino_ws_h1_2.git
```

2. Navigate inside the repo and create build folder
```sh
cd cufino_ws_h1_2
mkdir build
```

3. From the build folder, run cmake and make
```sh
cd build
cmake ..
make
```

## Development
If you are on the same network of the robot, you can directly connect to it and develop from the PC2 through SSH. In this case, you can use the extension of vscode SSH.

##Connection to robot PC2
To connect to the robot PC2 through ssh you can do the following procedure:

1. Ensure that the robot PC2 is turned on and the wi fi adapter is connected. Then, it automatically should connect to the Prisma Lab network.
See hosts on your network
```sh
sudo nmap -T4 -sP 192.168.1.0/24
```
and see the IP address assigned to the unitree robot, which has mac address E4:FA:C4:4C:B2:F8 (the corresponding IP is written before).

2. Connect with ssh
```sh
ssh unitree@<IP_ADDRESS_PC2>
```
Password Unitree0408.
Now you are connected.

3. If the robot PC2 is not shown after point 1 and you are not able to connect, either it did not turned on or you should reconnect to the network. In this case, connect through ethernet (the port is the same of low level pc) with
```sh
ssh unitree@192.168.123.162
```
Password Unitree0408.
Now you are connected.

If you want to reconnect the robot to the wi fi network in such way to avoid using the cable, type
```sh
sudo nmcli dev wifi connect "Wi-Fi_Prismalab" password "Prisma_Lab"
```


   
   
   
   
   
   
   
   
   
   
   
   
   
   
   

