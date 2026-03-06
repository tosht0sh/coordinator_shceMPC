# Basic setup and instructions

In this file, there is instructions to setting up ROS package for duckietown for the first time. Basic processes can be followed using the instructions provided here. We will keep the publisher on the bot, and have the subscriber be able to run in both ways. 

Basic things to keep in mind are:
- All the code goes into `packages/` folder. Each module can be in a subfolder inside this folder.
- We do not create a venv for dependencies. We need to mention them in the dependencies files as per the use case.
- Each script that will be run should have a launch file in the `launchers/` folder.


The folder currently packages currently made are:

1. Laptop communication node
2. Wheel encoder and wheel control node
3. Imu reader node

## TODOs:

1. Analyse how to fix SLIP issue in pose estimation, maybe use EKF or complementary filter
2. Add proper clock for coordination

## Running ROSCORE on laptop

In duckiebots we are using ROS1. For the communication we need, we would want the MPC on the bots, and the scheduler and coordinator on a laptop which will act as the main ROSCORE for our project. The version we need to run is ROS Noetic.

Since we already did all our installations on Ubuntu 22.04 which does not have support for ROS1, we will need to use a docker instance of ROS Noetic. Below are instructions to get that to work. If you are using Ubuntu 20.04or a version that can support ROS1, you could directly have the ROSCORE running.

1. In one terminal, go to the root folder of the project, run this command

    ```
    docker run -it --rm --network=host -v "$PWD":/coordinator_shceMPC -w /coordinator_shceMPC --name roscore_noetic ros:noetic-ros-core bash
    ```

    This will start a docker containerwhere we sun ROSCORE. To start the ROSCORE, do the following


    a. Source ROS

    ```
    source /opt/ros/noetic/setup.bash
    ```

    b. Now we need to export the correct IP adresses to ensure communication. Find the adress of your laptop on the network that the bot is connected to. Use that as the IP adress. This setup is done for my laptop. When you work, the adress will change. Run the three following lines to make it work.

    ```
    unset ROS_HOSTNAME
    export ROS_MASTER_URI=http://10.42.0.1:11311
    export ROS_IP=10.42.0.1
    ```


    b. Start roscore

    ```
    roscore
    ```

    Keep this terminal running.

2. In a new terminal, run the following command.

    ```
    docker exec -it roscore_noetic bash
    ```
    
    This will start the bash that can be used to run ROS commands.

    Then we do:

    a. Source ROS

    ```
    source /opt/ros/noetic/setup.bash
    ```

    b. In a similar way to before, we need to set get the network settings into this terminal as well.
    ```
    unset ROS_HOSTNAME
    export ROS_MASTER_URI=http://10.42.0.1:11311
    export ROS_IP=10.42.0.1
    ```
    Now, all the ros commands can be run here.

3. For the duckiebot sided code to work prooperly, you need to put the right network settings into the launch files. This is done in a similar way as before. Only, the ROS_IP will now be the robot's ip address on the network. Just add the followinf code before the dt-launchfile-init line in the launcher. This should enable communication between the bot and the laptop's roscore.

    ```
    unset ROS_HOSTNAME
    export ROS_MASTER_URI=http://10.42.0.1:11311
    export ROS_IP=10.42.0.129
    ```


## Communicatin between Laptop and Bots

The communication of ROS on laptop and DTROS on duckiebots is not easy. The way DTROS works is that is has nodes for sensor data acquistion and wheel actuation. We need to use this to extract data. Using 2 ROSCORE(1 on laptop and 1 on the bots) makes it difficult to have a direct publisher-subscriber architecture for communication. Hence, we changed communication to be using networking protocols as follows:

1. Sending data from duckiebot to laptop
    
    To send data to the laptop, we use UDP protocol at a frequency of 20 Hz. This is because the laptop constantly needs to monitory the state of the robot, and if we do miss a few messages in between due to the UDP protocol, it will not impact the system too much. The code for sending the data can be found in `packages/laptop_communication/pose_sender_udp.py`. The launcher file to launch this script is `launchers/pose_send_to_lap.sh`.

2. Sending commands form laptop to duckiebot

    We will be sending essentially just the inputs for the trajectory to the bot. This can be done only when a new sequence has to be sent to the bot, or can be done at a much slower rate than state monitiring. So, we will be trigerring only when needed for now, but may change to 5-10 Hz if needed. This communication being secure is a requirement, we need to be sure that the new plan is sent to the bot in case of a change. To ensure this, we use a TCP protocol here. TCP protocol ensures that messages sent have been received. 
    
    For now, since the MPC is not ready, we just have a code where we ensure that for the given target pose, the robot reaches the desired x-coordinate of that pose. The code for sending the data can be found in `packages/laptop_communication/target_pose_receiver.py`. The launcher file to launch this script is `launchers/target_receiver.sh`.
    

## Common issues faced in communication

1. Wi-Fi network

    It is geerally an issue to connect to the right networks and getting things to work properly. If there is a new network to connect to, you should connect the duckiebot to a screen and keyboard and add credentials of the network by following the instructions below

    a. Go to folder
        
        
        cd /etc/
        
    b. Open the file

        sudo nano wpa_supplicant.conf

    c. Add the WiFi credentials as follows

        network={
            id_str="network_2" # wifi priority
            ssid="WIFI_NAME"
            psk="WIFI_PASSWORD"
            key_mgmt=WPA-PSK
        }


2. SSH into the bot

    Sometimes you might need to ssh into the bot. For that, run the command `ssh duckie@[DUCK_NAME].local`. The password is quackquack.

3. Docker version

    The docker verstion on the bots is too old for them to run the launchers. For this, there are two ways of tackling it. You can either update the docker on the bot, or you can use an older docker version on you laptop to run it. If you are doing the second way, it might be better to create an alias so that you do not have to type long commands repeatedly.



## Instructions to run files

In this project, a simple publisher-subscriber architechture is made to run on a robot. The code can be founf in `packages/first_pkg`. Here is how to run it:

1. There is two ways to build a project. You can do it only for the robot, or you can do it for the host laptop. ROS is still running on the bot.
    
    To build only on a particular bot, run(this generally needs to be done to work on bots.)
    ```
    dts devel build -H [robot_name] -f
    ```

    To build on the system, run(This is if something will run on laptop)
    ```
    dts devel build -f
    ```

2. Run the node using
    
    ```
    dts devel run -H [ROBOT_NAME] -L [NODE_NAME]
    ```

    To run a node on bot, run

    ```
    dts devel run -H [ROBOT_NAME] -L [NODE_NAME] -n [DOCKER_NAME]
    ```

    NOTE: The -n flag tells dts to run another docker instance of the bot to start the publisher. If we are running multiple scripts on the same system via docker, we need to add the -n and a name along with it.

    To run a node on laptop, run

    ```
    dts devel run -R [ROBOT_NAME] -L [NODE_NAME]
    ```


## Creating a package and nodes.

1. make package using 
    ```
    mkdir -p ./packages/[PACKAGE_NAME]
    ```

2. go to the folder made above and create

    a. package.xml file
    ```
    <package>
    <name>[PACKAGE_NAME]</name>
    <version>0.0.1</version>
    <description>
    My first Catkin package in Duckietown.
    </description>
    <maintainer email="YOUR_EMAIL@EXAMPLE.COM">YOUR_FULL_NAME</maintainer>
    <license>None</license>

    <buildtool_depend>catkin</buildtool_depend>
    </package>
    ```

    NOTE: change email and name.

    b. CMakeLists.txt file
    ```
    cmake_minimum_required(VERSION 2.8.3)
    project([PACKAGE_NAME])

    find_package(catkin REQUIRED COMPONENTS
    rospy
    )

    catkin_package()
    ```

3. create the source folder

    go to the pacakages folder and create

    ```
    mkdir -p ./[PACKAGE_NAME]/src
    ```


Now you can create the speific scripts/nodes in the folder, and then to be able to run them, do

4. Make the script executable

    ```
    chmod +x ./packages/[PACKAGE_NAME]/src/[SCRIPT_NAME]
    ```

5. Make launcher script for script

    Go to the root folder of the project and create `launcher/[LAUNCHER_NAME].sh` and write

    ```
    #!/bin/bash
    source /environment.sh
    dt-launchfile-init
    rosrun my_package [SCRIPT_NAME]
    dt-launchfile-join
    ```

Now, upon building the project, we will be able to run the project.


[https://github.com/AHHHZ975/SLAM-Duckietown]