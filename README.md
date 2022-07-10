# Nav2_mpc_tracking_controller

The controller plug-in developed for Navigation2 uses MPC to track the trajectory. In order to ensure the control accuracy, in the end point control, the pure tracking algorithm is used to achieve point tracking . The implement of MPC is mainly refer to https://github.com/udacity/CarND-MPC-Quizzes. The difference is that our object is for the differential robot.

## demo

![](https://github.com/lsc12318/Nav2_mpc_tracking_controller/blob/main/pics/demo.gif)



## About  MPC and robot model

Waiting for replenishment.

## How to use

- **Install ROS2 and Navigation2 .** 

  You can refrence https://navigation.ros.org/build_instructions/index.html#install

  (recommend galactic version)

  

- **Clone this repo under Navigation2 folder**

  ```
  git clone https://github.com/lsc12318/Nav2_mpc_tracking_controller.git
  ```

- **Install ipopt and cppad**

  ```
  #ipopt
  sudo chmod +x install_ipopt.sh
  sudo apt-get install gfortran
  sudo apt-get install unzip
  wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
  sudo ./install_ipopt.sh Ipopt-3.12.7
  
  #cppad
  sudo apt-get install cppad
  ```

- **test**

  ```
  cd YOUR_WORK_SPACE/NAVIGATION2
  source install/setup.bash
  export TURTLEBOT3_MODEL=waffle
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/galactic/share/turtlebot3_gazebo/models
  
  ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True params_file:=./src/mpc_tracking_controller/param/my.yaml
  ```



## TODO

- [ ] Add control for location in the end of the paths.
- [ ] Code structure optimization
- [ ] Add response to obstacles
