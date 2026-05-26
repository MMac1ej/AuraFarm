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

Open 7 terminals all attached to the container. **Run in this exact order:**

### Terminal 1 — Launch simulation world
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch my_tb3_world new_world.launch.py
```

### Terminal 2 — Launch navigation + RViz
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/ws/maps/CleanSimMap.yaml
```
⚠️ Wait for RViz to open. Set **2D Pose Estimate** before continuing.

### Terminal 3 — Plant simulator (true ripeness)
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_field_dt plant_simulator
```

### Terminal 4 — Dynamic crop map (DT core)
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_field_dt dynamic_crop_map
```

### Terminal 5 — Battery monitor
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_ripeness_dt twin_state_monitor
```

### Terminal 6 — Farmer input (run this before Terminal 7)
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_ripeness_dt farmer_input
```
You will be prompted to enter ripeness thresholds for each plant type:

### Terminal 7 — Navigation node (run after farmer input confirms)
```bash
cd /ws && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 run aurafarm_navigation_dt nav_to_crop
```

## Running the Full System — Physical Robot

## Step 1 — SSH into robot and launch bringup
```bash
ssh ubuntu@<ROBOT_IP>
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

### Step 2 — Launch navigation with real lab map
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/ws/maps/map.yaml
```
⚠️ No `use_sim_time:=True` for the real robot.

### Step 3 — Set 2D Pose Estimate in RViz
1. Click **2D Pose Estimate** in RViz toolbar
2. Click on the map where the robot physically is
3. Hold and drag to set orientation
4. Wait for green particle cloud to appear

### Step 4 — Run all nodes (Terminals 3–7 same as simulation)
⚠️ Update `PLANTS` coordinates in `DynamicCropMapNode.py` and `PlantSimulatorNode.py` to match real lab plant positions before running.

### Step 5 — Teleop (optional, for manual control)
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

---

## Real Robot → Gazebo Mirroring

As the real robot moves, its position is mirrored in Gazebo in real time.

### Step 1 — Launch Gazebo world
```bash
cd /ws && source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch my_tb3_world new_world.launch.py
```

### Step 2 — SSH and launch bringup on robot
```bash
ssh ubuntu@<ROBOT_IP>
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

### Step 3 — Start the bridge node
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_navigation_dt robot_dt_bridge
```

### Step 4 — Verify mirroring
```bash
ros2 topic echo /aurafarm/dt_odom
ros2 topic echo /aurafarm/dt_scan
```

---

## ROS 2 Topics

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `/aurafarm/farmer_thresholds` | Farmer → DT | `String` | `A:0.8,B:0.9` |
| `/aurafarm/crop_arrival` | Robot → DT | `String` | Plant ID robot arrived at |
| `/aurafarm/plant_scan` | Simulator → DT | `String` | `plant_id:true_ripeness` |
| `/aurafarm/next_target` | DT → Robot | `String` | `plant_id:x:y` or `BASE:x:y` |
| `/aurafarm/harvest_command` | DT → Robot/Sim | `String` | `plant_id:HARVEST/CONFIRMED/SKIP` |
| `/aurafarm/harvest_complete` | Robot → DT | `String` | `plant_id` |
| `/aurafarm/harvest_complete` | Robot → DT | `String` | Plant ID harvested |
| `/aurafarm/base_arrived` | Robot → DT | `String` | Robot deposited at base |
| `/aurafarm/robot_status` | Robot → DT | `String` | `x:y:capacity:battery` |
| `/aurafarm/crop_map` | DT → All | `String` | Full plant state map |
| `/aurafarm/phase` | DT → Robot | `String` | `scanning` or `harvesting` |
| `/aurafarm/dt_battery_state` | DT mirror | `BatteryState` | Mirrored battery state |
| `/aurafarm/dt_system_status` | DT → All | `String` | `battery:%:status` |
| `/aurafarm/dt_odom` | Physical → DT | `Odometry` | Real robot position mirrored |
| `/aurafarm/dt_scan` | Physical → DT | `LaserScan` | Real robot LiDAR mirrored |

---

## Verifying Topics (for TA demo)

Show all AuraFarm topics:
```bash
ros2 topic list | grep aurafarm
```

Watch crop map update in real time:
```bash
ros2 topic echo /aurafarm/crop_map | tr '|' '\n'
```

Watch DT sending optimal targets:
```bash
ros2 topic echo /aurafarm/next_target
```

Watch harvest decisions:
```bash
ros2 topic echo /aurafarm/harvest_command
```

Watch bidirectional evidence:
```bash
# Robot → DT
ros2 topic echo /aurafarm/crop_arrival

# DT → Robot
ros2 topic echo /aurafarm/harvest_command
```

Watch battery state:
```bash
ros2 topic echo /aurafarm/dt_system_status
```

---

## Week 4 Checklist Evidence

| Requirement | Topic | Evidence |
|-------------|-------|----------|
| Entity A → Entity B | `/aurafarm/crop_arrival` | Robot publishes plant ID on arrival |
| Entity B → Entity A | `/aurafarm/next_target` | DT publishes optimal harvest target |
| Non-motion state | `/aurafarm/dt_system_status` | Battery level mirrored from physical robot |
| Environmental interaction | Obstacle avoidance | Robot navigates around obstacles using LiDAR + Nav2 |

---

## Week 8 Demo Evidence

| Requirement | What to show | Command |
|-------------|-------------|---------|
| Bidirectional pub/sub | crop_arrival + next_target | `ros2 topic echo /aurafarm/crop_arrival` |
| State synchronisation | crop_map updating in real time | `ros2 topic echo /aurafarm/crop_map \| tr '\|' '\n'` |
| Environmental interaction | Robot avoiding obstacles during tour | Watch Gazebo/RViz |
| DT imperfection | Second scan correcting simulated ripeness | Watch DT terminal logs |

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

**Robot not moving after farmer input**  
→ Make sure Terminal 7 (nav node) is running and Nav2 is fully active

**`No target found — all plants below threshold`**  
→ Plants not ripe yet — wait for growth. Check crop map with `ros2 topic echo /aurafarm/crop_map | tr '|' '\n'`

**Robot keeps going to same plant**  
→ Watchdog firing — check DT terminal for capacity or battery issues

**Nav2 waiting for amcl**  
→ Wait 30–60 seconds for Nav2 to fully start, then set 2D Pose Estimate in RViz

**Initial pose keeps resetting to (0,0)**  
→ Set 2D Pose Estimate in RViz **before** running the nav node

## Battery State Monitor

The battery monitor mirrors the physical robot's battery state to the digital twin
and publishes alerts when battery is low or critical.

### Run the battery monitor
```bash
cd /ws && source install/setup.bash && ros2 run aurafarm_ripeness_dt twin_state_monitor
```

### View battery state in real time
```bash
ros2 topic echo /aurafarm/dt_system_status
```