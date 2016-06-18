//
// Created by nishant on 6/6/16.
//

#ifndef BUG_FLOOD_TYPE_DECLS_H
#define BUG_FLOOD_TYPE_DECLS_H

#include <vector>
#include <geometry_msgs/Point.h>
#include <map>
#include <vector>
#include <assert.h>

using namespace std;

//typedef geometry_msgs::Point Point;

class Point
{
public:

	Point operator= (const Point &other)
	{
		Point newObject;
		newObject.x = other.x;
		newObject.y = other.y;
		newObject.z = other.z;
		return newObject;
	}


	Point(const Point &other)
	{
		x = other.x;
		y = other.y;
		z = other.z;
	}

	/*geometry_msgs::Point operator=(const Point &other)
	{
		geometry_msgs::Point newObject;
		newObject.x = other.x;
		newObject.y = other.y;
		newObject.z = other.z;
		return newObject;
	}*/

	Point():x(0.0),y(0.0),z(0.0){};

	virtual ~Point(){};

	double x;
	double y;
	double z;
};

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
