#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <bug_flood/envirnment.h>

//#define DEBUG

typedef visualization_msgs::Marker Marker;

void initializeMarkers(Marker &boundary,
					   Marker &obstacle,
					   Marker &startPoint,
					   Marker &endPoint)
{
	//init headers
	boundary.header.frame_id    = obstacle.header.frame_id    = startPoint.header.frame_id    = endPoint.header.frame_id    = "path_planner";
	boundary.header.stamp       = obstacle.header.stamp       = startPoint.header.stamp       = endPoint.header.stamp       = ros::Time::now();
	boundary.ns                 = "path_planner_bound";
	obstacle.ns                 = "path_planner_obs";
	startPoint.ns               = "path_planner_start";
	endPoint.ns                 = "path_planner_end";
	boundary.action             = obstacle.action             = startPoint.action             = endPoint.action             = Marker::ADD;
	boundary.pose.orientation.w = obstacle.pose.orientation.w = startPoint.pose.orientation.w = endPoint.pose.orientation.w = 1.0;

	//setting id for each marker
	boundary.id    = 100;
	obstacle.id   = 101;
	startPoint.id   = 111;
	endPoint.id   = 110;

	//defining types
	boundary.type  = Marker::LINE_STRIP;
	obstacle.type = Marker::LINE_LIST;
	startPoint.type = Marker::SPHERE;
	endPoint.type = Marker::SPHERE;

	//setting scale
	boundary.scale.x = 1;
	obstacle.scale.x = 1;

	startPoint.scale.x = 2;
	startPoint.scale.y = 2;
	startPoint.scale.z = 2;

	endPoint.scale.x = 2;
	endPoint.scale.y = 2;
	endPoint.scale.z = 2;

	//assigning colors
	boundary.color.r = obstacle.color.r = 0.0f;
	boundary.color.g = obstacle.color.g = 0.0f;
	boundary.color.b = obstacle.color.b = 0.0f;

	startPoint.color.b = startPoint.color.a = 1.0f;
	endPoint.color.g = endPoint.color.a = 1.0f;

	boundary.color.a = obstacle.color.a = 1.0f;
}

vector< Point > initializeBoundary(int rowSize, int colSize)
{
	vector< Point > bondArray;

	Point point;

	//first point
	point.x = 0;
	point.y = 0;
	point.z = 0;

	bondArray.push_back(point);

	//second point
	point.x = 0;
	point.y = colSize;
	point.z = 0;

	bondArray.push_back(point);

	//third point
	point.x = rowSize;
	point.y = colSize;
	point.z = 0;

	bondArray.push_back(point);

	//fourth point
	point.x = rowSize;
	point.y = 0;
	point.z = 0;
	bondArray.push_back(point);

	//first point again to complete the box
	point.x = 0;
	point.y = 0;
	point.z = 0;
	bondArray.push_back(point);

#ifdef DEBUG
	cout << "Exiting initializeBoundary function." << endl;
#endif

	return bondArray;
}

vector< Point > initializeObstacle(ObstacleIO &obstacleIO)
{
	vector<Point> obstacleLines;
	ObstacleList obsList = obstacleIO.getObstacleList();
	for (Obstacle obs : obsList)
	{
		for(Line line : obs)
		{
			obstacleLines.push_back(line.start);
			obstacleLines.push_back(line.end);
		}
	}

	return obstacleLines;
}

int main(int argc,char** argv)
{
	//initializing ROS
	ros::init(argc,argv,"env_display");
	ros::NodeHandle n;

	//defining Publisher
	ros::Publisher env_publisher = n.advertise<Marker>("path_planner_env",1);

#ifdef DEBUG
	cout << "ROS Initialization done." << endl;
#endif

	//defining markers
	Marker boundary;
	Marker obstacle;
	Marker startPoint;
	Marker endPoint;

	initializeMarkers(boundary, obstacle, startPoint, endPoint);

#ifdef DEBUG
	cout << "Marker Initialization done." << endl;
#endif

	ObstacleIO obstacleIO(argv[1],argv[2], argv[3]);

#ifdef DEBUG
	cout << "Obstacles Read." << endl;
#endif

	boundary.points = initializeBoundary(obstacleIO.getRowSize(), obstacleIO.getColSize());

#ifdef DEBUG
	cout << "Boundaries Initialized Read." << endl;
#endif

	obstacle.points = initializeObstacle(obstacleIO);

	#ifdef DEBUG
	cout << "Object Lines Initialized Read." << endl;
#endif

	startPoint.pose.position = obstacleIO.getSource();
	endPoint.pose.position = obstacleIO.getGoal();

#ifdef DEBUG
	cout << "Source Goal Initialized Read." << endl;
#endif

	while(ros::ok())
	{
		env_publisher.publish(boundary);
		ros::Duration(0.2).sleep();
		env_publisher.publish(obstacle);
		ros::Duration(0.2).sleep();
		env_publisher.publish(startPoint);
		ros::Duration(0.2).sleep();
		env_publisher.publish(endPoint);

		ros::Duration(1).sleep();

	#ifdef DEBUG
		cout << "Publishing at 1 Hz." << endl;
	#endif
	}

	return 1;
}
