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

private:
	/* Member Variables */
	Map map;
	Point source;
	Point goal;

	/* Modifiers */
	void ReadSourceGoal(string sourceGoal);
};



#endif //BUG_FLOOD_OBSTACLE_IO_H
