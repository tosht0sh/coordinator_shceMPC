# Basic setup and instructions

In this file, there is instructions to setting up ROS package for duckietown for the first time. Basic processes can be followed using the instructions provided here. We will keep the publisher on the bot, and have the subscriber be able to run in both ways. 

Basic things to keep in mind are:
- All the code goes into `packages/` folder. Each module can be in a subfolder inside this folder.
- We do not create a venv for dependencies. We need to mention them in the dependencies files as per the use case.
- Each script that will be run should have a launch file in the `launchers/` folder.


The folder currently packages currently made are:

1. Basic publisher-subscriber architecture.
2. Wheel encoder and wheel control node.
3. Imu reader node.

## TODOs:

1. Make proper documentations for the bots with issues faced.
2. Analyse if it is better to merge encoder and imu reading for position estimation. Use the correct one and make code for that.
3. Make the ROS architecture such that ROSCORE is on laptop and robots run only particular nodes that it needs to.
4. Implement MPC on the bot.

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