//
// Created by nishant on 15/5/16.
//

#ifndef BUG_FLOOD_MAP_H
#define BUG_FLOOD_MAP_H


#include <geometry_msgs/Point.h>

typedef geometry_msgs::Point Point;
/*
struct Location
{
	Point location;
	bool isObstacle;
};*/

class BinaryMap
{
public:
	BinaryMap();
	void BinaryMapInit(int rowSize, int colSize);
	virtual ~BinaryMap();

	bool isObstructed(Point location);
	bool **binary_map;
	int rowSize;
	int colSize;
private:
	bool isAllocated;
};

//typedef BinaryMap Map;

#endif //BUG_FLOOD_MAP_H
