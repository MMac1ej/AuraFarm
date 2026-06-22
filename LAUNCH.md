# Team 45 - AuraFarm - Launch Instructions
### Lab laptop credentials:
#### Username: team45
#### Password: moonshine-cashew-gallstone-kissing
---

## Physical robot with gazebo mirroring

Sourcing (terminals T2–T9):
```bash
source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger
```

**T1 - SSH into the robot:**
```bash
ssh ssh turtlebot@{IP_ADDRESS_OF_RASPBERRY_PI}
export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_bringup robot.launch.py
```

| # | Command |
|---|---------|
| T2 | `ros2 launch my_tb3_world new_world.launch.py` |
| T3 | `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=/$HOME$/areaMap.yaml` |
| T4 | `ros2 run aurafarm_navigation_dt robot_dt_bridge` |
| T5 | `ros2 run aurafarm_field_dt plant_simulator` |
| T6 | `ros2 run aurafarm_field_dt dynamic_crop_map` |
| T7 | `ros2 run aurafarm_ripeness_dt twin_state_monitor` |
| T8 | `ros2 run aurafarm_ripeness_dt farmer_input` |
| T9 | `ros2 run aurafarm_navigation_dt nav_to_crop` |

After T3: set **2D Pose Estimate** in RViz where the robot physically is before running T9.  
Run T9 only after T8 confirms thresholds sent.

The map files (.yaml and .pgm) need to be stored in the home catalogue.

---

## Simulation only

> Make sure the Docker container is running and every terminal is attached to it before starting.

Sourcing (every terminal):
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger
```

| # | Command |
|---|---------|
| T1 | `ros2 launch my_tb3_world new_world.launch.py` |
| T2 | `ros2 run aurafarm_navigation_dt sim_topic_relay` |
| T3 | `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/ws/maps/areaMap.yaml params_file:=/ws/params/burger.yaml` |
| T4 | `ros2 run aurafarm_field_dt plant_simulator` |
| T5 | `ros2 run aurafarm_field_dt dynamic_crop_map` |
| T6 | `ros2 run aurafarm_ripeness_dt twin_state_monitor` |
| T7 | `ros2 run aurafarm_ripeness_dt farmer_input` |
| T8 | `ros2 run aurafarm_navigation_dt nav_to_crop` |

After T3: set **2D Pose Estimate** in RViz before running T8.  
Run T8 only after T7 confirms thresholds sent.

---

