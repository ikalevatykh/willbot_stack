# Willbot stack

ROS stack for the INRAI's WILLOW group robot.

# Packages

* willbot_ar - package contains launch and configuration files to work with the Alvar AR tag tracking library.

* willbot_bringup - package contains launch files and scripts to configure and start all the core components: hardware, ros_control, MoveIt.

* willbot_description - package contains the description (mechanical, kinematic, visual, etc.) of the WillBot. The files in this package are parsed and used by a variety of other components.

* willbot_envs - package contains utils and scripts to create Gym environments with the real robot. The package also contains the environment server node, that can start environments by demand and the environment client class to communicate with the server from Python2 or Python3.

* willbot_moveit_config - package contains configuration and launch files for using the WillBot with the MoveIt motion planning framework.

* willbot_recognition - package contains launch and configuration files to work with the Object Recognition Kitchen.

* willbot_scripts - package contains some usefull scripts.

* willbot_simulation - package contains stuff to simulate the WillBot in Gazebo. 

* willbot_utils - package contains a python module with common utilities to work the WillBot hardware and sensors, to plan trajectories, to control velocity, and others.


# Start working

* Starting core components with the real robot:

   ```
   roslaunch willbot_bringup willbot_real.launch setup:=<paris|grenoble> speed_preset:=<normal|fast>
   ```

   This script connects to the arm and the gripper thru ros_control starts all necessary controllers and MoveIt framework.

   Parameters:
   * setup (paris* or grenoble): allow choosing a specific configuration for a kinematic model, obstacles on default scene and other specific settings.
   * speed_preset (normal* or fast): allow choosing speed limits for a MoveIt path planning subsystem.
   
 * Starting kinects and cameras:

   ```
   roslaunch willbot_bringup kinect_bringup param1:=value1 ...
   roslaunch willbot_bringup kinect2_bringup param1:=value1 ...
   roslaunch willbot_bringup usbcam_bringup param1:=value1 ...
   ```

   For a specific parameters see [launch scripts](https://github.com/ikalevatykh/willbot_stack/tree/master/willbot_bringup/launch).

 * Satrting the environment server:
 
    ```
   roslaunch willbot_envs server.launch debug:=<False|True>
   ```
   
   Parameters:
   * setup (False* or True): in the debug mode the server produces extended error info and reloads an environment class each time its code was changed.
   
   See an [example](https://github.com/ikalevatykh/willbot_stack/blob/master/willbot_envs/src/willbot_envs/envs/reach_env.py) of an environment.
   
 * Satrting an agent to control an environment:
 
    See an [example](https://github.com/ikalevatykh/willbot_stack/blob/master/willbot_envs/scripts/env_client_node.py) how to connect to and control an environment.
 
 
