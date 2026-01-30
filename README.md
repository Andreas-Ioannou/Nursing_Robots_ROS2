### Launch Simulation ### (Terminal-1)
ros2 launch simulation_pkg gaz_sim_launch.py

### Launch Localization Server ### (Terminal-2)
ros2 launch localization_server localization.launch.py map_file:=sim_warehouse_map_save.yaml

### Launch Navigation Server ### (Terminal-3)
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=True




### Currently ###
Able to change controller during run time (Terminal-4)

ros2 topic pub -1 /controller_selector std_msgs/msg/String "{data: MPC}" \
  --qos-durability transient_local --qos-reliability reliable

ros2 topic pub -1 /controller_selector std_msgs/msg/String "{data: DWB}" \
  --qos-durability transient_local --qos-reliability reliable

ros2 topic pub -1 /controller_selector std_msgs/msg/String "{data: MPPI}" \
  --qos-durability transient_local --qos-reliability reliable


### Next steps ###
Make a web user interface for the user to control the robot and change controllers at run time

Adjust controller parameters

