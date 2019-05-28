# bug_flood
A bug based offline path planner.

### Input
We take 3 inputs: A binary map, obstacle line lists (with lines in order. Note: we will have a tool to generate this input from the binary map.), and sourceGoal.

### Execution
Terminal One: ``roscore``

Terminal Two: ``rviz``
####Eviornment visualization
Terminal Three: ``rosrun bug_flood env_display <sourceGoal File> <obstacleLine file> <map file>``
####Bug Flood
Terminal Four: ``rosrun bug_flood bug_flood <sourceGoal File> <obstacleLine file> <map file>``
