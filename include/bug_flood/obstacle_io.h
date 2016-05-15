//
// Created by nishant on 15/5/16.
//

/**
 *
 */

#ifndef BUG_FLOOD_OBSTACLE_IO_H
#define BUG_FLOOD_OBSTACLE_IO_H

#define OBSTACLE_IO_DEBUG

#include <string>
#include <vector>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <bug_flood/line.h>
#include <bug_flood/map.h>
#include <iostream>

using namespace std;

typedef vector<Line> 			Obstacle;
typedef vector < Obstacle > 	ObstacleList;

class ObstacleIO
{
public:
	ObstacleIO(string sourceGoal, string obstacle);
	ObstacleIO(string sourceGoal, string obstacle, string map);
	virtual ~ObstacleIO() {};

	/* Accessor Functions */
//	BinaryMap getBinaryMap();
	bool isObstructed(Point location);
	ObstacleList getObstacleList();
	Line getObstacleLine(int obstacleIndex, int lineIndex);
	bool getNextLine(int obstacleIndex, int lineIndex, Point location, Line &nextLine);
	Point getSource();
	Point getGoal();
	int getRowSize();
	int getColSize();

private:
	/* Member Variables */
	BinaryMap binaryMap;
	ObstacleList obstacleList;
	Point source;
	Point goal;

	/* Modifiers */
	void ReadBinaryMap(string map);
	void ReadObstacleList(string obstacle);
	void ReadSourceGoal(string sourceGoal);
};



#endif //BUG_FLOOD_OBSTACLE_IO_H
