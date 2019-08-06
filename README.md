# pathfinding-series
pathplanning method for robotic arm with 6 DOF with classic algorithms RRT and PRM
The file named rrtComplied.py was used to evaluate the performance of three rrt-related algorithms towards the same test set. 

The three algorithms are respectively:
RRT_connect
RRT_Modified
RRT

The evaluation process:
The performance of an algorithm was summarized by an integer ranged from (0 to 100). If an algorithm fails to give a result it will automatically receive a zero. If it does yield a result, then it will receive 50 points. Then, it goes on and evaluate two criterion: steps took to complete the walk, time spent to come up with the solution. Each criteria was given a max of 25 points.

for step evaluation:
If it can complete the walk within 50 steps, it will receive 25 points in full. If steps was between 50 and 80 it will receive 90% of the points. If steps was between 80 and 100 it will receive 80% of the points. If steps was between 100 and 130 it will receive 70% of the points. If steps was between 130 and 150 it will receive 60% of the points. If it took higher than 150 steps it will get 50% of the points.

for time evaluation:
If it took less than 0.1 seconds to complete the calculation, it will receive 25 points in full. If it took between 0.1 and 0.5 seconds, it will receive 90% of the points. If it took between 0.5 and 1 seconds, it will receive 80% of the points. If it took between 1 and 4 seconds, it will receive 70% of the points. If it took between 4 and 8 seconds, it will receive 60% of the points. If it took more than 8 seconds to finish, then it will be given 50% of the points.



