//
// Created by nishant on 6/6/16.
//

#ifndef BUG_FLOOD_TYPE_DECLS_H
#define BUG_FLOOD_TYPE_DECLS_H

#include <vector>
#include <geometry_msgs/Point.h>
#include <map>

using namespace std;

typedef geometry_msgs::Point Point;

struct Line
{
	Point start;
	Point end;
};

//struct ObstacleLine
//{
//	int boundaryID;
//	Line line;
//};

//typedef vector<ObstacleLine> Obstacle;
typedef map<int, Line> ObstacleLines;
//typedef vector<Obstacle> ObstacleList;
typedef vector<Point> Path;

struct VisitInfo
{
	Point location;
	double cost;
};

#endif //BUG_FLOOD_TYPE_DECLS_H
