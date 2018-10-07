### COMS 4733 Lab 2

David Lee (jl4397)  
Rachel Wu (rww2115)  

### To Run:

In one terminal, run:
```
$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=<path_to_bug_file>
```

In another terminal, run:
```
$ cd lab2_rww2115_jl4397
$ python hw2.py
```

This will run the main pathfinder script

### Methods: 
init: starts up constants and runs the pathfinder
get_odom: update the position and rotation from the odometer
run_pathfinder: runs the bug2 algorithm
follow_boundary: called when an object is encountered, and follows the wall on the right
at_hitpt(self): checks if at the hitpoint
obstacle_in_front: checks if there's an obstacle in front
update_odom: updates the odometer variables in the class
face_pos: face a certain position, like the goal
print_slope: prints the slope from current position to target
rotate_amt: rotates a certain amount in the positive radians direction
goal_reached: checks if the goal is reached
print_pos: prints the current position
face_angle: faces an angle in radians
is_mline: checks if on the mline
rotate_inc: rotates incrementally
translate_inc: translates incrementally
print_dists: prints the 3 distances in left, center, and right
scan_call: the most important method because it determines what the orthogonal angle is; updates the distances to objects 
euclidean_distance: calculates the distance between two points
shutdown: standard shutdown ros method


### Videos:
World 0: https://youtu.be/zu5p4SVLhMw  
World 1: https://youtu.be/5JY0yMQ286w  
World 2: https://youtu.be/npU7BWwDPto  
World 3: https://youtu.be/q6B2PYBE_Oo  
World 5:  
 
