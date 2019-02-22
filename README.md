# graffiti-drone


# developer\operator installation 

1. Install ROS (kinetic or melodic) as described on ros.org. Install ros-*-desktop-full package. Init ros repo (sudo rosdep init & rosdep update)
2. Install Mavros as described here: https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation . Do not forget to install GeographicLib datasets.
3. Install ros-*-control-toolbox package.
4. Clone this repo. 
5. Build this repo
    cd graffiti-drone/catkin_ws
    catkin_make


# onboard installation

1. Install ROS Kinetic as described on ros.org. Install ros-knetic-base package. Init ros repo (sudo rosdep init & rosdep update)
2. Install Mavros as described here: https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation . Do not forget to install GeographicLib datasets.
3. Install additional packages
    ros-kinetic-control-toolbox
    ros-kinetic-angles
    ros-kinetic-tf2-geometry-msgs
4. Clone this repo. 
5. Build this repo (only onboard packages)
    cd graffiti-drone/catkin_ws
    catkin_make --pkg mavros mavros_extras ros_graffiti_pose ros_graffiti_visual serial sf11_node teraranger_array
6. Start onboard ROS
    graffiti_drone/scripts/graffiti start 

# run operator GS

1. source your ROS solution
    source graffiti-drone/catkin_ws/devel/setup.sh 
2. setup your GS ip and onboard ip
    export ROS_IP=192.168.0.52
    export ROS_MASTER_URI=http://192.168.0.50:11311

    192.168.0.52 is your ground station IP and 192.168.0.50 is IP of your onboard computer.
3. run rviz
    rviz -d graffiti-drone/rviz/rviz.d

# SVG support

Current graffiti solution supports SVG with <path> elemets. Only M (moveto) and L(lineto) commands are supported. Look at svg/circlebox-trck.svg for example.

# run gazebo and SITL on developer computer

1. Run gazebo:
    source graffiti-drone/catkin_ws/devel/setup.sh 
    roslaunch ~/graffiti-drone/gazebo/launch/apm-gazebo.launch fcu_url:=udp://127.0.0.1:14551@
2. Run SITL 
    First you need to clone arducopter-graffiti repo. Then

    cd arducopter-graffiti/ArduCopter/
    ../Tools/autotest/sim_vehicle.py -f gazebo-iris -D 

