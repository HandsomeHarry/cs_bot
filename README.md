# cs_bot

This is a ROS implemented 2v2 wingman robot game of shooting opponents digitally and planting/defusing a digital bomb in the bomb site (they dont strike counters)

# Game rules:
- T side (2 turtlebots): plant bomb and defend try getting to bombsite 
- CT side (2 turtlebots): defend bombsite / retake after planting before timer runs out (1m30s before plant and 40s bomb)
- 3.2s for planting, 5s for defuse
- First to 3 wins

Before starting, run the following in console to set up map:
`roslaunch cs_bot test_world.launch`
`roslaunch turtlebot3_slam turtlebot3_slam.launch`
`teleop`
`rosrun map_server map_saver -f $(find cs_bot)/maps/map`
The arena will be bound by walls, after a robot player ‘dies’, it’ll stay put. The robots will try to ‘kill’ each other.


# Generating the world map:

To map the surrouding and generate a `.yaml` world file and a `.pgm` map, follow these steps:

1. **Bring up the robot** to get started.

2. Launch SLAM using `gmapping` by running the following command:

   ```bash
   roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
   ```

3. Open a new terminal and run the command: `teleop` to control the robot with the keyboard. Drive the robot around the target area to scan its surroundings.

4. Once the entire target area has been scanned, open another terminal and execute the following command to save the map:

   ```bash
   rosrun map_server map_saver -f `rospack find cs_bot`/maps/my_map
   ```

5. This will generate two files: `<map_name>.yaml` and `<map_name>.pgm` in `cs_bot/maps`.

# Defining the spawn points and bomb site area

1. run the python file


## Bot node:
- Movement logic
- Be able to tell where it and its teammate currently located with SLAM and report to server node
- Each have their own namespaces (e.g. /bot1/cmd_vel)
- Recognizing other robots, tell if is enemy or teammate (a block of color at a specific height)
- Use camera to detect relative angle of enemy, turns to it and shoot
- More complicated navigation algorithms (getting to bombsite, planting, defending)
- Try applying dynamic costmaps, obstacles are robots


## Personality management
- Aggressive playstyle (rusher)
- Moderate playstyle (normie)
- Passive (baiter)

## Gunfight mechanics
- Rifle 25 dmg, 0.2 speed; Sniper 100 dmg, 0.1 speed; SMG 15 dmg 0.3 speed.
- Must turn to the opponent to shoot them
- Read the server’s broadcast of currently alive players to not shoot at dead robots

# Server node:
- Map (Fixed, inferno A)
- Send location of teammate (RY vs BG)
- Responsible for creating the map
- Distribute timer info, bomb info (time and location)
- Chance of hitting of each shot, hit registering, fire rate, health management, reload time
- Broadcast currently alive players
- Distributes random personalities to robots at start of game
- Tell the T which one has the bomb and if dead drops the bomb / both have bombs?

# Hardest problems:
- Movement strategy of Ts, CTs, the three personalities (6 in total). 
- How would we use the map to navigate in this scenario? How to give the bombsite a heavier weight so that most fights occur there?
- Coordinating all 4 robots in the same roscore in the same map
- Odom (locating and detecting where each robot is on the map)
- Map setup, localization, and simulation (Gazebo, RViz).
- How to set up the bombsite in the map for the robots to know where to go (colored tape?)

