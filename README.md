# ME495 Embedded Systems Homework 2
Author: Kyle Thompson

The turtle_brick package describes a turtle_robot which mirrors the movement of the turtle_sim in an arena of the same size. The arena can place a brick in the air, and drop it at a given gravity acceleration. The catcher will detect the falling brick, and move to catch it if the brick is reachable at maximum velocity. If the brick is caught, the turtle_robot will drop the brick back at the origin. 

## Quickstart
1. Use `ros2 launch turtle_brick turtle_arena.launch.py` to start the arena and turtle simulation
2. Use `ros2 service call drop std_srvs/srv/Empty` to drop a brick
3. Here is a video of the turtle when the brick is within catching range
   ${embed video here, it must be playable on github. Upload the video as an issue and link to it}

4. Here is a video of the turtle when the brick cannot be caught

   ${embed video here, it must be playable on github. Upload the video as an issue and link to it}