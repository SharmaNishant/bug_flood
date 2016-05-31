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
	Environment(string sourceGoal);
	Environment(string sourceGoal, string map);
	virtual ~Environment() {};

	/* Accessor Functions */
	virtual bool isObstructed(Point location);
	Point getSource();
	Point getGoal();

private:
	/* Member Variables */
	Map map;
	Point source;
	Point goal;

	/* Modifiers */
	void ReadBinaryMap(string map);
	void ReadSourceGoal(string sourceGoal);
};



#endif //BUG_FLOOD_OBSTACLE_IO_H
