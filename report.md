# Counter-Strike Bot

Github repo: https://github.com/HandsomeHarry/cs_bot

Harry Yu, Zared Cohen, TsonOn Kwok

## Introduction

Our goal was to create a robot simulation of the popular game franchise, Counter Strike, which is a first-person shooter game where teams of terrorists (Ts) and counter-terrorists (CTs) fight for their specific goals. The Ts must plant a bomb and defend it until detonation, and the CTs must eliminate all Ts or defuse the bomb if planted. If the round time runs out before either goal occurs, the CTs will be victorious. For our implementation, we created a 2v2 version of this game in a simulated environment (gazebo), and a 1v1 version in the real world. We also designed different personalities and guns for each robot (player), which provides a more realistic simulation of the game with an unpredictable outcome.

### Problem statement (objectives)
- Robots can detect enemy teams and shoot 
- Robots can successfully navigate the course and avoid obstacles/other robots
- Server node manages player health, game state, bomb site locations
- Communication between local (robot) nodes and server node 

## What was created
1. There are a few groups of tasks we had to work on:
the gazebo world simulation
2. The creation of the map using SLAM, how bomb site locations, patrol points and spawn points are defined. This must be done manually.
3. AMCL that allows the robots to navigate and avoid obstacles
4. Personalities of robots, robot actions when situations change using FSMs (very complicated)
5. Modularized launch files to be used when needed

### Files Overview

#### `world3.world`
- A Gazebo world map.
- Simplified version of Inferno, a site in Counter-Strike.

#### `publish_point.py`
- **Purpose**: Tool for generating map configuration points.
- **Functionality**:
  - Records clicked points in RViz for map setup.
  - Saves spawn points and bomb site locations to a CSV file.
  - Launches the map server and RViz for point selection.
- **Usage**: Used during the map setup/configuration phase.

#### `map_manager.py`
- **Purpose**: Loads and manages map configuration from CSV files.
- **Functionality**:
  - Handles bomb site locations and spawn points.
  - Publishes visualization markers for map features.
  - Creates RViz markers for bomb sites and spawn locations.
  - Manages the map coordinate system and point conversions.

#### `game_manager.py`
- **Purpose**: The central server node managing the game state.
- **Functionality**:
  - Handles round timers, bomb events, and robot health.
  - Publishes game state updates to all robots.
  - Manages combat events and damage calculations.
  - Tracks dead players and round winners.
  - Controls round phases (e.g., PREP, ACTIVE, BOMB_PLANTED).

#### `robot_controller.py`
- **Purpose**: Base class for all robot behaviors.
- **Functionality**:
  - Handles movement, combat, and state management.
  - Processes visual detection of enemies.
  - Manages navigation and pathfinding.
  - Interfaces with the ROS navigation stack.
- **Specialization**: Parent class for personality-specific behaviors.

#### Child Classes of `robot_controller.py`
- **Examples**:
  - `CT_normie.py`: A defensive robot personality.
  - `T_aggressive.py`: An offensive robot personality.

#### `robot_state_manager.py`
- **Purpose**: Manages individual robot states.
- **Functionality**:
  - Tracks health, position, and status.
  - Handles damage reception and death states.
  - Publishes robot state updates.
  - Manages weapon states and combat capabilities.

#### `gun.py`
- **Purpose**: Defines weapon types and their characteristics.
- **Functionality**:
  - Manages weapon properties (e.g., damage, fire rate, accuracy).
  - Handles ammo management and reload timers.
  - Provides shooting mechanics with accuracy calculations.
  - Returns damage values based on successful hits.


## How to Run the Game (in Gazebo)

### Building the Environment
---
1. Ensure the `transitions` library is installed:
   ```
   pip3 install transitions
   pip3 install pandas
   ```
2. Navigate to your catkin workspace:
   ```
   cd <your-catkin-workspace>
   ```
3. Build the workspace:
   ```
   catkin_make
   ```
   If no error messages appear, proceed to the next step.

### Generating the Map
---
We need a robot to explore the map using SLAM to manually generate the world map for later use.

1. **Launch SLAM** using `gmapping` by running the following command:
   ```
   roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
   ```

2. Open a new terminal and control the robot with the keyboard:
   ```
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
   ```
   Drive the robot around the target area to scan its surroundings.

3. Once the entire target area has been scanned, open another terminal and save the map:
   ```
   rosrun map_server map_saver -f `rospack find cs_bot`/maps/<map_name>
   ```
   This will generate two files: `<map_name>.yaml` and `<map_name>.pgm` in the `cs_bot/maps` directory.

### Defining Areas
---
We will define the bomb sites using our script and `rviz`.

1. Run the following command:
   ```
   rosrun cs_bot publish_point.py
   ```

2. Click on the map according to the prompt. The results will be saved to `points.csv`.

The coordinates saved in `points.csv` can be used by other nodes later.

<!-- fig -->

### Launch Game
---
Run the game by launching 2v2_scenario.launch, which launches all the nodes required including launching 4 robots and starting the game server node:
```
roslaunch cs_bot 2v2_scenario.launch
```
<!-- fig -->

