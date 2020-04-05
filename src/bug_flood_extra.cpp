//
// Created by nishant on 6/6/16.
//
#include <bug_flood/bug_flood.h>

#ifdef PUBLISH_PATH

void initMarkers(Marker &bugs, Marker &pathTree) {
  // init headers
  bugs.header.frame_id = pathTree.header.frame_id = "path_planner";
  bugs.header.stamp = pathTree.header.stamp = ros::Time::now();
  bugs.ns = pathTree.ns = "path_planner_bug_flood";
  bugs.action = pathTree.action = visualization_msgs::Marker::ADD;
  bugs.pose.orientation.w = pathTree.pose.orientation.w = 1.0;

  // setting id for each marker
  bugs.id = 0;
  pathTree.id = 1;

  // defining types
  bugs.type = visualization_msgs::Marker::POINTS;
  pathTree.type = visualization_msgs::Marker::LINE_LIST;

  // setting scale
  pathTree.scale.x = 1;
  pathTree.scale.y = 1;
  pathTree.scale.z = 1;
  bugs.scale.x = 1;
  bugs.scale.y = 1;
  bugs.scale.z = 1;

  // assigning colors
  bugs.color.r = 1.0f;
  pathTree.color.b = 1.0f;

  bugs.color.a = pathTree.color.a = 1.0f;
}

void publish_bugs(vector<Bug> &bugList, ros::Publisher &publisher, Marker &bugs,
                  Marker &pathTree) {
  // clear old marker data
  bugs.points.clear();

  /**
   * Optimization in plotting. instead of clearing the old data just append the
   * last line of the bug
   */
  // pathTree.points.clear();

  // generate data
  for (Bug &bug : bugList) {
    bugs.points.push_back(bug.getLocation());
    // push the last line of the bug as a line in path tree
    vector<Point> path = bug.getpath();
    pathTree.points.push_back(path[path.size() - 2]);
    pathTree.points.push_back(path[path.size() - 1]);
  }

  // publish
  publisher.publish(bugs);
  ros::Duration(0.1).sleep();
  publisher.publish(pathTree);
  ros::Duration(0.1).sleep();
}

void publish_bug(Bug &bug, ros::Publisher &publisher, Marker &pathTree) {
  // clear old marker data
  pathTree.points.clear();

  // change pathTree's properties
  pathTree.type = visualization_msgs::Marker::LINE_STRIP;

  // generate data
  pathTree.points = bug.getpath();

  // publish
  publisher.publish(pathTree);
  ros::Duration(0.1).sleep();
}

#endif

#ifdef KINEMATIC_BUG

bool updateCarrot(Line line, Point &rabbit, Point &carrot) {
  bool isIncrement = false;

  /** get robot projection on the current line that is being followed **/
  Point rabbitIntersection;
  getPerpendicularLineIntersection(line, rabbit, rabbitIntersection);

  /** place carrot a fixed distance ahead of robot on that line **/
  double carrotHeadingAngle = getDirectionAngleRadian(line.end, line.start);

  Point carrotNewPosition;
  carrotNewPosition.x = rabbitIntersection.x + ((5) * cos(carrotHeadingAngle));
  carrotNewPosition.y = rabbitIntersection.y + ((5) * sin(carrotHeadingAngle));

  /** check if carrot is still on the line segment or not **/
  double carrotToLineStart, lineLength;
  carrotToLineStart = getEuclideanDistance(carrotNewPosition, line.start);
  lineLength = getEuclideanDistance(line.start, line.end);

  /** if carrot is crossing the line segment
                  then carrot location is waypoint value and carrot state is
  reached waypoint else carrot location is the new carrot location
  **/
  if (carrotToLineStart > lineLength) {
    carrot = line.end;
    isIncrement = true;
  } else {
    carrot = carrotNewPosition;
  }
  return isIncrement;
}

void updateRabbit(Point &rabbit, Point carrot) {
  /** update  **/
  double headingAngle = getDirectionAngleRadian(carrot, rabbit);
  rabbit.x = rabbit.x + ((RABBIT_STEP_SIZE)*cos(headingAngle));
  rabbit.y = rabbit.y + ((RABBIT_STEP_SIZE)*sin(headingAngle));
}

void pruneRabbitCarrot(vector<Point> &inPath, double &cost) {
  vector<geometry_msgs::Point> bugPath;
  /**
          Final path was reversed. Reversing it back to make it from source to
     goal;
  */
  int i;
  vector<Point> path;

  /** Rabbit's start point is source and then pushing start point to the new
   * path */
  Point rabbit = bugPath[0];
  path.push_back(rabbit);

  Point carrot;
  bool changeLine = false;
  i = 1;

  /** loop to traverse from source to dest and generate path according to
   * carrot-following algorithm*/
  while (i < inPath.size()) {
    Line line;
    line.start = inPath[i - 1];
    line.end = inPath[i];
    changeLine = updateCarrot(line, rabbit, carrot);
    updateRabbit(rabbit, carrot);
    path.push_back(rabbit);
    if (changeLine) {
      i++;
    }
  }
  path.push_back(bugPath[bugPath.size() - 1]);
  inPath = path;
}

#endif
