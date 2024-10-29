# cs_bot

This is a ROS implemented 2v2 wingman robot game of shooting opponents digitally and planting/defusing a digital bomb in the bomb site (they dont strike counters)

# Game rules:
- T side (2 turtlebots): plant bomb and defend try getting to bombsite 
- CT side (2 turtlebots): defend bombsite / retake after planting before timer runs out (1m30s before plant and 40s bomb)
- 3.2s for planting, 5s for defuse
- First to 3 wins

Before starting, put the blocks up according to the map making sure fiducials are on the right sides. Map the world by making a single robot walk around the map.
The arena will be bound by colored tape, after a robot player ‘dies’, it’ll move outside the tape to not interfere with the rest of the game. The robots will try to ‘kill’ each other.

## Bot node:
- Movement logic
- Be able to tell where it and its teammate currently located with fiducials on walls and report to server node
- Each have their own namespaces (/robot1/cmd_vel)
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
