# AuraFarm — Berry Scouting Digital Twin
**Team 45 - Six Survivors**  

A digital twin of an autonomous berry harvesting system that detects crop ripeness and optimises plant-level harvesting decisions using a TurtleBot3 and ROS 2 Jazzy.


---

## Prerequisites

- Docker Desktop running in background
- WSL terminal open

---

## Starting the Docker Container

```bash
docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /mnt/e/AuraFarm/AuraFarm:/ws \
--user $(id -u):$(id -g) turtlebot3_ws bash
```

If you get a display error, use this instead:
```bash
docker run --rm -it --name turtlebot3_container --net=host -e DISPLAY=$DISPLAY \
-v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
-v /mnt/e/AuraFarm/AuraFarm:/ws \
--user $(id -u):$(id -g) turtlebot3_ws bash
```

### Attaching extra terminals to the running container
```bash
docker exec -it turtlebot3_container bash
```

### Sourcing (run in every new container terminal)
```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## Building the Workspace

Run this after any code changes:
```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
colcon build
source install/setup.bash
```

To build a single package:
```bash
colcon build --packages-select <package_name>
```

---

## Option A — Simulation Demo (Gazebo)

Open 6 terminals, all attached to the container. Run in order:

### Terminal 1 — Launch simulation world
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch my_tb3_world new_world.launch.py
```

### Terminal 2 — Launch navigation + RViz
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/ws/maps/CleanSimMap.yaml
```
⚠️ Wait for RViz to open before continuing.

### Terminal 3 — Ripeness sensor node
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_field_dt simulated_ripeness_sensor
```

### Terminal 4 — Ripeness map node (Digital Twin state tracker)
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_field_dt ripeness_map
```

### Terminal 5 — Decision node
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_ripeness_dt ripeness_decision
```

### Terminal 6 — Navigation node (starts crop tour)
```bash
cd /ws && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 run aurafarm_navigation_dt nav_to_crop
```

---

## Option B — Physical Robot Demo

### Step 1 — SSH into the robot
Open a WSL terminal (not inside Docker):
```bash
ssh ubuntu@<ROBOT_IP>
```

### Step 2 — Launch bringup on the robot
Run this on the robot:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```
Leave this terminal open.

### Step 3 — Launch navigation with real lab map
In a container terminal on your laptop:
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/ws/maps/map.yaml
```
⚠️ No `use_sim_time:=True` for the real robot.

### Step 4 — Set initial pose in RViz
1. Click **2D Pose Estimate** in RViz toolbar
2. Click on the map where the robot physically is in the lab
3. Hold and drag to set the orientation
4. Wait for the green particle cloud to appear

### Step 5 — Teleop (to move robot manually)
```bash
cd /ws && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 run turtlebot3_teleop teleop_keyboard
```

| Key | Action |
|-----|--------|
| W | Forward |
| X | Backward |
| A | Turn left |
| D | Turn right |
| S / Space | Stop |
| Ctrl+C | Quit |

### Step 6 — Battery monitor
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_ripeness_dt twin_state_monitor
```

Verify battery state is publishing:
```bash
ros2 topic echo /aurafarm/dt_system_status
```

---

## Physical Robot — Harvesting Tour

⚠️ Before running this, update `CROP_POSITIONS` in `nav_to_crop.py` with coordinates from the real lab map. Use the RViz **Publish Point** tool to pick open positions on the map.

### Step 1 — SSH and launch bringup on robot
```bash
ssh ubuntu@<ROBOT_IP>
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

### Step 2 — Launch navigation with real lab map
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/ws/maps/map.yaml
```

### Step 3 — Set 2D Pose Estimate in RViz

### Step 4 — Start sensor and decision nodes
```bash
# Terminal 3
cd /ws && source install/setup.bash && ros2 run aurafarm_field_dt simulated_ripeness_sensor

# Terminal 4
cd /ws && source install/setup.bash && ros2 run aurafarm_field_dt ripeness_map

# Terminal 5
cd /ws && source install/setup.bash && ros2 run aurafarm_ripeness_dt ripeness_decision
```

### Step 5 — Start battery monitor
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_ripeness_dt twin_state_monitor
```

### Step 6 — Start harvesting tour
```bash
cd /ws && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 run aurafarm_navigation_dt nav_to_crop
```

---

## Real Robot → Gazebo Mirroring

As the real robot moves in the lab, its position is mirrored in Gazebo in real time.

### Step 1 — Launch Gazebo simulation world
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch my_tb3_world new_world.launch.py
```

### Step 2 — SSH and launch bringup on robot
```bash
ssh ubuntu@<ROBOT_IP>
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

### Step 3 — Launch navigation with real lab map
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/ws/maps/map.yaml
```

### Step 4 — Set 2D Pose Estimate in RViz

### Step 5 — Start the bridge node
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_navigation_dt robot_dt_bridge
```

### Step 6 — Move the real robot
Use teleop or the harvesting tour. Watch the Gazebo model mirror the real robot's movements.

### Step 7 — Verify mirroring is working
```bash
# Check DT odometry is publishing
ros2 topic echo /aurafarm/dt_odom

# Check DT scan is publishing
ros2 topic echo /aurafarm/dt_scan
```

---

## ROS 2 Topics

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `/aurafarm/crop_arrival` | Robot → DT | `Int32` | Robot arrived at crop ID |
| `/aurafarm/ripeness_data` | Sensor → DT | `String` | `crop_id:colour` |
| `/aurafarm/harvest_decision` | DT → Robot | `String` | `crop_id:HARVEST/SKIP` |
| `/aurafarm/crop_map` | DT → All | `String` | Full crop state map |
| `/aurafarm/dt_battery_state` | DT mirror | `BatteryState` | Mirrored battery state |
| `/aurafarm/dt_system_status` | DT → All | `String` | `battery:%:status` |
| `/aurafarm/dt_battery_alert` | DT → Robot | `String` | Alert when battery low |
| `/aurafarm/dt_odom` | Physical → DT | `Odometry` | Real robot position mirrored to DT |
| `/aurafarm/dt_scan` | Physical → DT | `LaserScan` | Real robot LiDAR mirrored to DT |

---

## Verifying Topics (for TA demo)

Show all AuraFarm topics:
```bash
ros2 topic list | grep aurafarm
```

Echo bidirectional evidence:
```bash
# Direction 1: Robot → DT
ros2 topic echo /aurafarm/crop_arrival

# Direction 2: DT → Robot
ros2 topic echo /aurafarm/harvest_decision
```

Echo battery state:
```bash
ros2 topic echo /aurafarm/dt_system_status
```

Echo position mirroring:
```bash
ros2 topic echo /aurafarm/dt_odom
```

---

Echo DT crop map (updates every second even between crop visits):
```bash
ros2 topic echo /aurafarm/crop_map
```
Note: the RipenessMapNode terminal only logs when a new crop reading arrives.
The crop map topic itself publishes every second — use the echo command above
to see the full DT state updating in real time between crop visits.

## Week 4 Checklist Evidence

| Requirement | Topic | Evidence |
|-------------|-------|----------|
| Entity A → Entity B | `/aurafarm/crop_arrival` | Robot publishes crop ID on arrival |
| Entity B → Entity A | `/aurafarm/harvest_decision` | DT publishes HARVEST/SKIP back |
| Non-motion state | `/aurafarm/dt_system_status` | Battery level mirrored from physical robot |
| Environmental interaction | Obstacle avoidance | Robot navigates around obstacles using LiDAR + Nav2 |

---

## Crop Positions (Simulation)

| Crop | X | Y |
|------|---|---|
| 1 | -1.5 | 0.0 |
| 2 | 0.4 | 0.8 |
| 3 | 0.9 | -1.0 |
| 4 | -2.0 | -3.0 |
| 5 | -2.4 | -0.5 |
| 6 | -1.0 | -1.2 |
| 7 | -1.0 | -3.0 |

---

## Common Errors

**`ros2: command not found`**  
→ Run `source /opt/ros/jazzy/setup.bash`

**`Package not found`**  
→ Run `source /ws/install/setup.bash`

**`No such container: turtlebot3_container`**  
→ Docker container not running — start it with the `docker run` command above

**`qt.qpa.xcb: could not connect to display`**  
→ Use the alternative docker run command with `/mnt/wslg/.X11-unix`

**Robot stuck / not moving**  
→ Set **2D Pose Estimate** in RViz before running the navigation node

**Nav2 waiting for amcl**  
→ Wait 30-60 seconds for Nav2 to fully start before running the nav node

**No decision received for crop X**  
→ Make sure Terminal 3 (sensor) and Terminal 5 (decision) are running before Terminal 6 (nav)