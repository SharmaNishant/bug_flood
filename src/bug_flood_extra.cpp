//
// Created by nishant on 6/6/16.
//
#include <bug_flood/bug_flood.h>

#ifdef PUBLISH_PATH

void initMarkers(Marker &bugs, Marker &pathTree)
{
	//init headers
	bugs.header.frame_id    = pathTree.header.frame_id    = "path_planner";
	bugs.header.stamp       = pathTree.header.stamp       = ros::Time::now();
	bugs.ns                 = pathTree.ns                 = "path_planner_bug_flood";
	bugs.action             = pathTree.action             = visualization_msgs::Marker::ADD;
	bugs.pose.orientation.w = pathTree.pose.orientation.w = 1.0;

	//setting id for each marker
	bugs.id    = 0;
	pathTree.id      = 1;

	//defining types
	bugs.type  = visualization_msgs::Marker::POINTS;
	pathTree.type = visualization_msgs::Marker::LINE_LIST;

	//setting scale
	pathTree.scale.x     = 1;
	bugs.scale.x   = 2;
	bugs.scale.y   = 2;
	bugs.scale.z   = 2;

	//assigning colors
	bugs.color.r   = 1.0f;
	pathTree.color.b     = 1.0f;

	bugs.color.a = pathTree.color.a = 1.0f;
}

void publish_bugs(vector<Bug> &bugList, ros::Publisher &publisher, Marker &bugs, Marker &pathTree)
{
	//clear old marker data
	bugs.points.clear();

	/**
	 * Optimization in plotting. instead of clearing the old data just append the last line of the bug
	 */
	//pathTree.points.clear();

	//generate data
	for(Bug &bug : bugList)
	{
		bugs.points.push_back(bug.getLocation());

		//push the last line of the bug as a line in path tree
		vector<Point> path = bug.getpath();
		pathTree.points.push_back(path[path.size()-2]);
		pathTree.points.push_back(path[path.size()-1]);
	}

	//publish
	publisher.publish(bugs);
	ros::Duration(0.2).sleep();
	publisher.publish(pathTree);
	ros::Duration(0.2).sleep();
}

void publish_bug(Bug &bug, ros::Publisher &publisher, Marker &pathTree)
{
	//clear old marker data
	pathTree.points.clear();

	//change pathTree's properties
	pathTree.type = visualization_msgs::Marker::LINE_STRIP;

	//generate data
	pathTree.points = bug.getpath();

	//publish
	publisher.publish(pathTree);
	ros::Duration(0.2).sleep();
}

#endif
