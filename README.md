# ME495 Embedded Systems Homework 2
Author: Harrison Bounds

This ROS package creates an arena and robot spawned at the origin. It then places, and drops a brick from a specified location. A calculation is then made to determine whether the robot can reach the brick or not.

## Quickstart
1. Use `ros2 launch turtle_brick turtle_arena.launch.py` to start the arena and turtle simulation
2. Use /arena/drop_brick service to drop a brick
3. Here is a video of the turtle when the brick is within catching range

[ES_HW2_CatchesBrick.webm](https://github.com/user-attachments/assets/8b60a1f4-4859-4142-9b8e-f4ec214b85da)

4. Here is a video of the turtle when the brick cannot be caught

[ES_HW2_Unreachable.webm](https://github.com/user-attachments/assets/a779f8b2-a588-411b-9c97-83c537cc33e5)