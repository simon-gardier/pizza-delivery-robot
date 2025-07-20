```
                                                                                    ___
                                                                                    |  ~~--.
                                                                                    |%=@%%/
   ___ _                    _      _ _                                              |o%%%/ 
  / _ (_)__________ _    __| | ___| (_)_   _____ _ __ _   _                      __ |%%o/
 / /_)/ |_  /_  / _` |  / _` |/ _ \ | \ \ / / _ \ '__| | | |               _,--~~ | |(_/ ._
/ ___/| |/ / / / (_| | | (_| |  __/ | |\ V /  __/ |  | |_| |            ,/'  m%%%%| |o/ /  `\.
\/    |_/___/___\__,_|  \__,_|\___|_|_| \_/ \___|_|   \__, |           /' m%%o(_)%| |/ /o%%m `\
                                                      |___/          /' %%@=%o%%%o|   /(_)o%%% `\
                                                                    | %%o%(_)%%%%%o%(_)%%%o%%o%o%% |
```

https://github.com/user-attachments/assets/73a90d16-e35b-4787-9543-a0d41f7cd646

# Quick presentation

The goal of the project is to explore an unknown maze with a Turtlebot3, find arucos (clients to deliver later) with the camero, localize them in the map referential and finally deliver these clients when receiving a notification on a topic. The project implements an occupancy grid exploration algorithms and a delivery scheduling algorithm. Best next target (greedy approach) is decided with A*.

See [report.pdf](report.pdf) for more details.

Project undertaken as part of the INFO0948 course given by Pr. Sacré at ULiège.
Final grade : 17/20.
Note : No copying (even partial) of this code within the scope of the INFO0948 course will be tolerated.

# Structure
- `code`: code folder fot the project.

# Commands
See `commands.md` for the commands to be used in the project and other helpfull commands from the tutorials.

# Main Script

## `robonardo.py`
- **Purpose**: Explore the map using turtlebot3_cartographer + turtlebot3_navigation2 and Delivery logic for the robot.
- **Description**:
  - Check for new frontier to explore each 5s
  - Stop checking when no more frontier to explore (= map complete)
  - Can be paused and resumed using the `/pause_exploration_navigation` topic.
    - Pause manually the exploration, from the terminal:
      ```
      ros2 topic pub /pause_exploration_navigation std_msgs/msg/Bool "{data: true}" --once
      ```
    - Resume manually the exploration, from the terminal:
      ```
      ros2 topic pub /pause_exploration_navigation std_msgs/msg/Bool "{data: false}" --once
      ```

  - Should subscribe to the "/aruco_markers_map" topic to get the poses of the arucos in the map.
  - Keep a list of these markers as "self.client_positions[]"
  - When a new message is published on "aruco_markers_map", call self.schedule_delivery()

  - Should subscribe to the "/delivery_locations" topic to get the client id to which deliver.
  - Keep a list of these requests as "self.delivery_requests[]"
  - Schedule delivery when delivery requested by client:
    - Call self.schedule_delivery()

  - When delivery finished:
    - Set self.delivering to False, set self.client_being_delivered to None
    - Call self.schedule_delivery()

  - Implement self.schedule_delivery():

# Ressources
  - Project page: https://gitlab.uliege.be/info0948-2/info0948-introduction-to-intelligent-robotics/-/tree/main/project
  - Nav2 config file doc: https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/params/nav2_params.yaml
  - Official tuning guide: https://docs.nav2.org/tuning/index.html
  - Non-official tuning guide: https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/
  - Exploration algorithm inspiration: https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2/blob/main/custom_explorer/explorer.py
  - Calibration guide: docs.nav2.org/tutorials/docs/camera_calibration.html
  - Tutorial 2 (Gazebo): https://gitlab.uliege.be/info0948-2/info0948-introduction-to-intelligent-robotics/-/tree/main/project
  - Tutorial 3 (Turtlebot + SLAM): https://gitlab.uliege.be/info0948-2/info0948-introduction-to-intelligent-robotics/-/blob/main/tutorials/tutorial-3.md
  - Tutorial 4 (Turtlebot + Navigation + Aruco): https://gitlab.uliege.be/info0948-2/info0948-introduction-to-intelligent-robotics/-/blob/main/tutorials/tutorial-4.md
  - FollowPath algorithm used in the project: https://docs.ros.org/en/iron/p/nav2_mppi_controller/
  - Intelligent frontier discovery : https://github.com/SeanReg/nav2_wavefront_frontier_exploration/blob/main/nav2_wfd/wavefront_frontier.py

# Credits
- [Simon Gardier](github.com/simon-gardier) (Co-author)
- Arthur Graillet (Co-author)
- Bruce Andriamampianina (Co-author)

