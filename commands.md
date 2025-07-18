# Project commands

## Start the project
1. Build the project:
```
cd project
colcon build --symlink-install
source install/setup.bash
```

2. Start the bringup, on the turtlebot:
```
ssh ubuntu@<turtlebot_ip>                       # ip to find on https://tplinkeap.net/ see # Passwords section below for the credentials
ros2 launch turtlebot3_bringup robot.launch.py
```

3. Start the external nodes we rely on, on the PC:

With our launch file:
```
ros2 launch robonardo tools.launch.py
```

Or one at a time:
```
ros2 launch turtlebot3_cartographer cartographer.launch.py                                          # For SLAM
ros2 launch turtlebot3_navigation2 navigation2.launch.py params_file:='./src/robonardo/config/nav2-custom-params.yaml'   # For Navigation (start it from project/)
ros2 launch ros2_aruco aruco_recognition.launch.py                                                  # For Aruco markers
ros2 launch robonardo turtlebot3_state_publisher.launch.py                                          # For TF from camera, to base (=robot), to map referential
```

If you want to start nav2 with the map : 
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py arams_file:='./src/robonardo/config/nav2-custom-params.yaml map:=./map.yaml
```

4. Run our nodes:
```
ros2 run robonardo robonardo    # For exploration + delivery
ros2 run robonardo aruco        # For aruco marker localization in the map
```

## Debugging tools
### Check Arucos topic message:
```
ros2 topic echo /aruco_markers
```

### Check TF tree:
```
ros2 run tf2_tools view_frames
```

### Check camera view
```
ros2 run rqt_image_view rqt_image_view
```

### Check all the started nodes and topics:
```
rqt_graph
```

### Pause exploration manually
ros2 topic pub /pause_exploration std_msgs/msg/Bool "{data: true}" --once

### Resume exploration manually
ros2 topic pub /pause_exploration std_msgs/msg/Bool "{data: false}" --once

### Sent a delivery request
ros2 topic pub /delivery_locations std_msgs/msg/Int32MultiArray "{data: [2]}" --once

# Passwords
- PC : info0948-2
- Turtlebot3 : turtlebot
- TPLINK EAP (user, password): (admin, chiefbot)

# Tutorials commands (https://docs.ros.org/en/foxy/Tutorials.html)

# Communication mediums
- **Topics**: publish-subscribe, continuous flow, use rqt_graph for debug
- **Services**: ponctual call (e.g. clear, reset,...), request/response, client makes request to node providing service 
- **Actions**: long run with results, streams feedbacks while running the action, can be cancelled, send a final result at the end
- **Parameters** (not used)

# Start turtlebot3 bring-up
`ros2 launch tuto_2 turtlebot3_state_publisher.launch.py`

# List ROS2 packages
`ros2 pkg list`

# Start launch file
`ros2 launch <package_name> <launch_file.py>`

# Start executable
`ros2 run <package_name> <executable_name>`

# Open debug window
`rqt_graph`

# GOTO some package folder
`colcon_cd <package_name>`

# Create a ros workspace
Simply create a folder with a subfolder called "src", create a package in the src folder with the commands below.

# Create a ros2 package
`ros2 pkg create --build-type ament_python <package_name>`

# Build package (from workspace root folder)
```
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select <package_name>
source install/setup.bash #to do in the terminal starting a node
```

# Interface info
`ros2 interface show <type_name>` (e.g `ros2 interface show geometry_msgs/msg/Twist`)

# Show tf2 transform results
`ros2 run tf2_tools view_frames.py`
`ros2 run tf2_ros tf2_echo <source_frame> <target_frame>`
`ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share <executable_name>)/rviz/<rviz_name>.rviz` (e.g. ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz)

# Fix rviz2 issue in VSCode
`unset GTK_PATH`

# Nodes commands
## List nodes
`ros2 node list`
## Infos about a node
`ros2 node info <node_name>`
## Rename node
`ros2 run <package_name> <executable_name> --ros-args --remap __node:=<new_name>`
Note : executable = file being run, node = runtime entity created by executable to perform tasks. Thus an executable can create a node with a different name that their.

# Topics
## List topics
`ros2 topic list -t`
## Infos about a topic
`ros2 topic info <topic_name>`
## Echo topic data
`ros2 topic echo <topic_name>`
## Publish topic message
`ros2 topic pub <topic_name> <msg_type> '<args>'` (e.g. `ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
`)
Note: --once publish one message then exit

# Services commands
## List service
`ros2 service list`
## Infos about a service
`ros2 service info <service_name>`
`ros2 service type <service_name>`
`ros2 service list -t`
## Find services
`ros2 service find <type_name>` (e.g. `ros2 service find std_srvs/srv/Empty`)`
## Call service
`ros2 service call <service_name> <service_type> <arguments>` (e.g. `ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"`)

# Actions commands
## List Actions
`ros2 actions list -t`
## Infos about an action
`ros2 action info /turtle1/rotate_absolute`
## Send action goal
`ros2 action send_goal <action_name> <action_type> <values>` (e.g. ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}")

# Build
```
colcon build --symlink-install
```

```
colcon test
colcon test --packages-select YOUR_PKG_NAME --ctest-args -R <your_test_in_pkg>
```

```
source install/setup.bash
```

```
ros2 pkg create <package_name>
```

# Turtle env
- Packag name: turtlesim
- Executable names: turtlesim_node, turtle_teleop_key
- Nodes: turtlesim_node(my_turtle), turtle_teleop_key

# Start ssh session
```
ifconfig
service ssh start
ssh ubuntu@10.22.188.124
cd /mnt/hgfs/shared
```

# Mount folder
```
sudo vim /etc/fstab
vmhgfs-fuse     /mnt/hgfs       fuse    defaults,allow_other    0       0
sudo mount -a
```
