#include <bug_flood/environment.h>
#include <bug_flood/helper_functions.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

//#define DEBUG std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;
#define DEBUG ;

typedef visualization_msgs::Marker Marker;

void initializeMarkers(Marker &boundary, Marker &obstacle, Marker &startPoint,
                       Marker &endPoint) {
  // init headers
  boundary.header.frame_id = obstacle.header.frame_id =
      startPoint.header.frame_id = endPoint.header.frame_id = "path_planner";
  boundary.header.stamp = obstacle.header.stamp = startPoint.header.stamp =
      endPoint.header.stamp = ros::Time::now();
  boundary.ns = "path_planner_bound";
  obstacle.ns = "path_planner_obs";
  startPoint.ns = "path_planner_start";
  endPoint.ns = "path_planner_end";
  boundary.action = obstacle.action = startPoint.action = endPoint.action =
      Marker::ADD;
  boundary.pose.orientation.w = obstacle.pose.orientation.w =
      startPoint.pose.orientation.w = endPoint.pose.orientation.w = 1.0;

  // setting id for each marker
  boundary.id = 100;
  obstacle.id = 101;
  startPoint.id = 111;
  endPoint.id = 110;

  // defining types
  boundary.type = Marker::LINE_STRIP;
  obstacle.type = Marker::CUBE_LIST;
  startPoint.type = Marker::SPHERE;
  endPoint.type = Marker::SPHERE;

  // setting scale
  boundary.scale.x = 0.2;

  obstacle.scale.x = 1;
  obstacle.scale.y = 1;

  startPoint.scale.x = 1;
  startPoint.scale.y = 1;
  startPoint.scale.z = 1;

  endPoint.scale.x = 1;
  endPoint.scale.y = 1;
  endPoint.scale.z = 1;

  // assigning colors
  boundary.color.r = obstacle.color.r = 0.0f;
  boundary.color.g = obstacle.color.g = 0.0f;
  boundary.color.b = obstacle.color.b = 0.0f;

  startPoint.color.b = startPoint.color.a = 1.0f;
  endPoint.color.g = endPoint.color.a = 1.0f;

  boundary.color.a = obstacle.color.a = 1.0f;
}

vector<Point> initializeBoundary(int rowSize, int colSize) {
  vector<Point> bondArray;

  Point point;

  // first point
  point.x = 0;
  point.y = 0;
  point.z = 0;

  bondArray.push_back(point);

  // second point
  point.x = 0;
  point.y = colSize;
  point.z = 0;

  bondArray.push_back(point);

  // third point
  point.x = rowSize;
  point.y = colSize;
  point.z = 0;

  bondArray.push_back(point);

  // fourth point
  point.x = rowSize;
  point.y = 0;
  point.z = 0;
  bondArray.push_back(point);

  // first point again to complete the box
  point.x = 0;
  point.y = 0;
  point.z = 0;
  bondArray.push_back(point);

  return bondArray;
}

vector<Point> initializeObstacle(vector<Point> obstacles) {
  vector<Point> obstacleLines;
  for (Point point : obstacles) {
    point.x += 0.5;
    point.y += 0.5;
    obstacleLines.push_back(point);
  }

  return obstacleLines;
}

void initializeMarker(Marker &path) {
  // init headers
  path.header.frame_id = "path_planner";
  path.header.stamp = ros::Time::now();
  path.ns = "path_planner_path";
  path.action = Marker::ADD;
  path.pose.orientation.w = 1.0;

  // setting id for each marker
  path.id = 112;

  // defining types
  path.type = Marker::LINE_STRIP;

  // setting scale
  path.scale.x = 0.1;

  // assigning colors
  path.color.g = 1.0f;
  path.color.a = 1.0f;
}

void populateMarker(Marker &path, string file) {
  ifstream infile(file);
  if (!infile.is_open()) {
    cout << "Cannot Open Map File. Exiting...";
    exit(-1);
  }

  std::string line;
  vector<string> splittedLine;

  while (std::getline(infile, line)) {
    splittedLine = split(line, ' ');
    Point point;
    point.x = stod(splittedLine[0]);
    point.y = stod(splittedLine[1]);
    point.z = 0;
    path.points.push_back(point);
  }
  infile.close();
}

int main(int argc, char **argv) {
  if (argc < 3) {
    cout << "USAGE: rosrun bug_flood env_display <source_goal> <map> "
            "<path(optional)>"
         << endl;
  }

  // initializing ROS
  ros::init(argc, argv, "env_display");
  ros::NodeHandle n;

  // defining Publisher
  ros::Publisher env_publisher = n.advertise<Marker>("path_planner_env", 1);

  DEBUG

  // defining markers
  Marker boundary;
  Marker obstacle;
  Marker startPoint;
  Marker endPoint;

  initializeMarkers(boundary, obstacle, startPoint, endPoint);

  DEBUG

  Environment environment(argv[1], argv[2]);

  DEBUG

  int length = environment.getEnvironmentLength();
  int width = environment.getEnvironmentWidth();
  boundary.points = initializeBoundary(length, width);

  DEBUG

  obstacle.points =
      initializeObstacle(environment.getObstructedLocations(length, width));

  DEBUG

  startPoint.pose.position = environment.getSource();
  endPoint.pose.position = environment.getGoal();

  DEBUG

  Marker path;
  // if we also have to publish the path
  if (argc >= 4) {
    initializeMarker(path);
    populateMarker(path, argv[3]);
  }

  while (ros::ok()) {
    env_publisher.publish(boundary);
    ros::Duration(0.2).sleep();
    env_publisher.publish(obstacle);
    ros::Duration(0.2).sleep();
    env_publisher.publish(startPoint);
    ros::Duration(0.2).sleep();
    env_publisher.publish(endPoint);
    if (argc >= 4) {
      ros::Duration(0.2).sleep();
      env_publisher.publish(path);
    }

    ros::Duration(1).sleep();

#ifdef DEBUG
    cout << "Publishing at 1 Hz." << endl;
#endif
  }

  return 1;
}
