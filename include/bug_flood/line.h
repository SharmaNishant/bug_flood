//
// Created by nishant on 15/5/16.
//

#ifndef BUG_FLOOD_LINE_H
#define BUG_FLOOD_LINE_H
#include <string>
#include <vector>
#include <fstream>
#include <geometry_msgs/Point.h>

using namespace std;


typedef geometry_msgs::Point Point;

class Line
{
public:
	bool CheckIntersection(Line other, Point &intersection);
	bool isAtEnd(Point location);
	bool isAtStart(Point location);

	Point start;
	Point end;
};

#endif //BUG_FLOOD_LINE_H
