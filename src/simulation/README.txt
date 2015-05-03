**Running simulator
- If you have our aliases installed, can run using sim
- Can run vision (alias) to see published camera feed

** Install uwsim:
- apt-get install ros-hydro-uwsim
- Use catkin_make to make the workspace
- Or run the script InstallUWSim.sh in the scrpits folder

** Running uwsim for srmauv:
- roscore
- cd into this directory (roscd Simulation)
- rosrun uwsim uwsim --configfile ./scenes/sauv_scene.xml
- disableShaders are enabled by default in the scene graph (modify by changing the <disableShaders> tag)

** Main published topics:
- /srmauv/srmauv_odom: position of a vehicle on a ROS nav_msgs/Odometry topic
- /srmauv/camera1: image from front camera
- /srmauv/camera2: image from bottom camera
- /srmauv/imu: imu data
- /srmauv/pressure : pressure data
- /srmauv/gps : gps data
- /srmauv/dvl : dvl data
- /srmauv/sonarleft: range sensor data on left (simulate sonar)
- /srmauv/sonarright: range sensor data on right (simulate sonar)

** Subscribed topics:
- /dataNavigator: listens on a ROS nav_msgs/Odometry topic and applies the position/velocity to the vehicle

** Test Node (used when the simulation is running):
- Used to control the vehicle velocities using arrow and WASDQE keys
- In the root dir: source ./devel/setup.bash
- rosrun Simulation keyboard_control
- If the node is not found, under Simulation, create two empty folders called include and simulation
- UP: move forward; DOWN: move backward; LEFT & RIGHT: yaw; A & D: roll; W & S: pitch; Q: sink down; E: float up (Run when the terminal is active!)
- view front camera image: rosrun image_view image_view image:=srmauv/camera1
- view bottom camera image: rosrun image_view image_view image:=srmauv/camera2
