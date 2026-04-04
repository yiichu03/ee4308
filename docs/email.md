Dear all,

 

The feedback for P1R is below. Marks will not be uploaded yet because there is still a need to summarize all reports properly first before assigning points. Most teams have many improvements and it is a difficult to compare.

Deadline for project 2 has been extended by one day for all components until Tuesday. Good luck!

 

P1R General Feedback:

Better teams would:

Include diagrams of runs and experiments.
Make mention of equations where possible to explain the theory.
Justify tuned values from a few basic experiments, keeping in mind the interaction with other values when performing the experiments.
Focuses on explaining "why" and "how" solutions explicitly lead to better performances (e.g. under what situations), rather than relying on descriptive text and phrases like "more stable", "less oscillation". 
Many improvements based on observing the limitations of the basic tasks and explaining how they solve them.
Others (may be useful for project 2):

If something is not implemented but it is stated it in the report, don't mention it. I won't read it. However, if something is implemented and it failed, it would be good to mention why it failed as there may be some analysis there.
Make sure that report file name is labelled correctly, and that there are no names and only matric numbers in the report.
Try to explain why certain solutions are implemented, instead of just stating them, as they do not speak for themselves. For example, many teams just stated the minimum lookahead distance for the vary lookahead heuristic without explaining that this is to make sure that the robot will not come to a stop at low speed situations.
 

Controller:

Notable improvements include:

Some observe only the obstacles in front of the robot for the obstacle heuristic.
Some folks have P controller, or even a sigmoid controller when the robot is near the goal. The controllers control the linear and angular velocities.
Other more common improvements like rotating on the spot, and setting minimum values to prevent the robot from stopping or moving exceptionally slow in some situations.
Again, these improvements are informed by certain limitations in the algorithm.
 

Planner:

Notable improvements / works include:

Many rightfully pointed out that Theta* can only work binary cost situations, hugging the obstacle based on max_access_cost. Interesting solutions include multi-cost Theta* by accumulating the cost of the maps cells over the line, or simply discarding the LOS check when the cells involved are not at the lowest free cost (so a mix of A* and Theta*). If you have done some variant of multi-cost Theta*, congratulations, you are already more powerful than most folks on the internet.
One or two teams did post-processing. Equally valid because it is good enough. One or two noted that pulling in one direction can cause non-tautness, so need to pull from another direction.
Justified m and p in the SG filter with experiments and explanations.
Notable problems / comments include:

"1+ map_cost/255" may cause the path to move closer to the obstacle, because by dividing by 255 the path is allowed to move through inflated cells with less penalty. It will cause the robot to hug the max_access_cost zones, rendering the inflation less useful at robot safety.
LOS can be simply implemented by sampling points at fixed intervals instead of using Bresenham or other, harder-to-implement algorithms. Since the path obtained is "good enough" for a fine resolution of 5cm,  points sampled at an appropriate interval will also lead to similar results as Bresenham, with insignificant differences at the boundary of max_access_cost. The only difference is in profiling the code to see which is faster. In robotics or engineering applications, first satisfy the "good enough", before investigating how to "perfect it".