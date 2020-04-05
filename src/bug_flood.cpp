#include <bug_flood/bug.h>
#include <bug_flood/bug_flood.h>
#include <ros/ros.h>

using namespace std;

int main(int argCount, char **argValues) {
  // check if proper inputs are passed
  if (argCount <
      4) // Check the value of passedArgumentCount. if filename is not passed
  {
    std::cout << "usage -> rosrun path_planning nodeName <sourceGoal> <map> "
                 "<output>\n";
    // Inform the user on how to use the program
    exit(0);
  }
  // path pruning is on by default
  int isPruneON = 1;
  if (argCount >= 5) {
    if (!strcmp("last", argValues[4])) {
      isPruneON = 2;
    }

    if (!strcmp("first", argValues[4])) {
      isPruneON = 1;
    }

    if (!strcmp("none", argValues[4])) {
      isPruneON = 0;
    }
  }

  // initializing ROS
  ros::init(argCount, argValues, "bug_flood");
  ros::NodeHandle n;

  Environment environment(argValues[1], argValues[2]);

  Bug finalBug;
  finalBug.setCost(INT_MAX); // this bug will be used to store the min path's
                             // bug

  vector<Bug> bugList;
  // add the first bug to the list
  bugList.push_back(Bug(environment.getSource()));

#ifdef PUBLISH_PATH
  ros::Publisher pathPublisher = n.advertise<Marker>("bug_flood_path", 1);
  ros::Duration(1).sleep();
  Marker pathTree;
  Marker bugs;
  initMarkers(bugs, pathTree);
#endif

  // start timer
  ros::Time startTime = ros::Time::now();

  while (ros::ok()) {
    // Kill bugs that are done
    KillBugs(bugList, finalBug);

    // if all the bugs are killed
    if (bugList.empty()) {
      break;
    }

    // for all bugs in the bugList
    //		for (Bug &bug: bugList)
    //		{
    //			if (bug.getState() == Bug::TOWARDS_GOAL)
    //			{
    //				bug.TowardsGoal(environment, bugList);
    //				continue; //to avoid running a bug twice when changing transition
    //through TOWARDS_GOAL to BOUNDARY_FOLLOW
    //			}
    //			if(bug.getState() == Bug::BOUNDARY_FOLLOWING)
    //			{
    //				bug.BoundaryFollow(environment);
    //			}
    //		}
    for (int i = 0; i < bugList.size(); ++i) {
      if (bugList[i].getState() == Bug::TOWARDS_GOAL) {
        bugList[i].TowardsGoal(environment, bugList);
        continue; // to avoid running a bug twice when changing transition
                  // through TOWARDS_GOAL to BOUNDARY_FOLLOW
      }
      if (bugList[i].getState() == Bug::BOUNDARY_FOLLOWING) {
        bugList[i].BoundaryFollow(environment);
      }
    }
#ifdef PUBLISH_PATH
    publish_bugs(bugList, pathPublisher, bugs, pathTree);
#endif
  }

  // print the algo's running time
  ros::Time endTime = ros::Time::now();
  int nsec = endTime.nsec - startTime.nsec;
  int sec = endTime.sec - startTime.sec;
  if (nsec < 0) {
    sec -= 1;
    nsec += 1000000000;
  }
  // ROS_INFO("End, Total Time = %d, %d", sec, nsec);

  double mainTime = sec + (nsec / 1000000000.0);
  double pruneTime = 0;

  double before_pruning_cost = finalBug.getCost();

  // Before pruning dump the un-pruned path so that it can be used in OMPL
  // PathSimplifier
  string sgPath(argValues[1]);
  std::size_t found = sgPath.find_last_of("/\\");
  string directoryPath = sgPath.substr(0, found);
  finalBug.DumpPath(directoryPath + "/path" + sgPath.substr(found + 3));

  // do path pruning only if pruning is on
  if (isPruneON > 0) {
    startTime = ros::Time::now();
    vector<Point> resultingPath = finalBug.getpath();
    double cost;

    if (isPruneON == 1)
      prunePathFirst(resultingPath, environment, cost);
    else
      prunePathLast(resultingPath, environment, cost);

    finalBug.setCost(cost);
    finalBug.setPath(resultingPath);
    endTime = ros::Time::now();
    nsec = endTime.nsec - startTime.nsec;
    sec = endTime.sec - startTime.sec;
    if (nsec < 0) {
      sec -= 1;
      nsec += 1000000000;
    }
    //	ROS_INFO("Pruning Time = %d, %d", sec, nsec);
    pruneTime = sec + (nsec / 1000000000.0);
  }

#ifdef KINEMATIC_BUG
  startTime = ros::Time::now();
  vector<Point> resultingPath = finalBug.getpath();
  double cost;

  pruneRabbitCarrot(resultingPath, cost);

  finalBug.setCost(cost);
  endTime = ros::Time::now();
  nsec = endTime.nsec - startTime.nsec;
  sec = endTime.sec - startTime.sec;
  if (nsec < 0) {
    sec -= 1;
    nsec += 1000000000;
  }
  // ROS_INFO("RABBIT CARROT Pruning Time = %d, %d", sec, nsec);
  pruneTime += (sec + (nsec / 1000000000.0));
#endif

  //	ROS_INFO("COST %f", finalBug.getCost());

// if path is to be published
#ifdef PUBLISH_PATH
  publish_bug(finalBug, pathPublisher, pathTree);
  ros::Duration(1).sleep();
  ros::spinOnce();
#endif

  // logfile
  ofstream logFile;
  logFile.open(argValues[3], ofstream::app);

  logFile << before_pruning_cost << " " << mainTime << " " << finalBug.getCost()
          << " " << pruneTime << endl;

  logFile.close();

  // dump final path in a fixed log file
  finalBug.DumpPath("/tmp/bug_flood_path.txt");

  return 1;
}
