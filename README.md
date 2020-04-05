**NOTE:** This project was done as part of a research/prototype project. The
code is not maintained. Please reach out to the author if you have any questions
regarding this work.


# Bug Flood
A bug based offline path planner.

### Input
We take 3 inputs: A binary map, obstacle line lists (with lines in order. Note: we will have a tool to generate this input from the binary map.), and sourceGoal.

### Execution
Terminal One: ``roscore``

Terminal Two: ``rviz``

#### Eviornment visualization
Terminal Three: ``rosrun bug_flood env_display <sourceGoal File> <obstacleLine file> <map file>``

#### Bug Flood
Terminal Four: ``rosrun bug_flood bug_flood <sourceGoal File> <obstacleLine file> <map file>``
