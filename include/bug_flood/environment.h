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

//#define CONVEX

using namespace std;

class Environment
{
public:
	Environment(string sourceGoal, int length, int width);
	Environment(string sourceGoal, string mapFile);
	virtual ~Environment() {};

	/* Accessor Functions */
	//to be implemented by the client
	virtual bool isObstructed(Point location);
	Point getSource();
	Point getGoal();
	int getEnvironmentLength();
	int getEnvironmentWidth();
	void getEnvironmentDimensions(int &length, int &width);
	vector<Point> getObstructedLocations(int &rowSize, int &colSize);

	//-1 if not visited
	double isVisited(double row, double col);
	double isVisited(Point location);

	void setVisited	(double row, double col, double cost);
	void setVisited	(Point location, double cost);

	//distance is from start of first line to the intersection point
	bool getObstacleIntersection(Point start, Point end, Point &intersection, double &distance, int &boundaryID,Point &location);
	bool getObstacleIntersection(Line line, Point &intersection, double &distance, int &boundaryID, Point &location);

	ObstacleLines getObstacleLines();

	bool getNextBoundaryLine(Point location, int &boundaryID, Point &tempGoal);

	Line getLine(int id);

private:
	/* Member Variables */
	Map map;
	Point source;
	Point goal;
	ObstacleLines lines;

	vector<VisitInfo> visited; //A global point set to keep track of what's visited and what's not

	void generateObstacleLineMap();

	/* Modifiers */
	void ReadSourceGoal(string sourceGoal);
};



#endif //BUG_FLOOD_OBSTACLE_IO_H
