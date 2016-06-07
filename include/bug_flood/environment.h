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
#include <bug_flood/map.h>
#include <iostream>
#include <bug_flood/type_decls.h>

using namespace std;

class Environment
{
public:
	Environment(string sourceGoal, int length, int width);
	Environment(string sourceGoal, string mapFile);
	virtual ~Environment() {};

	/* Accessor Functions */
	//to be implemented by the client
	virtual bool isObstructed(void* location);
	Point getSource();
	Point getGoal();
	int getEnvironmentLength();
	int getEnvironmentWidth();
	void getEnvironmentDimensions(int &length, int &width);
	vector<Point> getObstructedLocations(int &rowSize, int &colSize);

	bool isVisited	(int row, int col);
	bool isVisited	(Point location);

	void setVisited	(int row, int col);
	void setVisited	(Point location);

	//distance is from start of first line to the intersection point
	bool getObstacleIntersection(Point start, Point end, Point &intersection, double &distance, int &boundaryID);
	bool getObstacleIntersection(Line line, Point &intersection, double &distance, int &boundaryID);

	bool getNextBoundaryLine(Point location, int &boundaryID, Point &tempGoal);

private:
	/* Member Variables */
	Map map;
	Point source;
	Point goal;
	ObstacleList obstacleList;

	void generateObstacleList();

	/* Modifiers */
	void ReadSourceGoal(string sourceGoal);
};



#endif //BUG_FLOOD_OBSTACLE_IO_H
