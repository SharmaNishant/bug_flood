//
// Created by nishant on 30/5/16.
//

#ifndef BUG_FLOOD_BUG_H
#define BUG_FLOOD_BUG_H

#include <bug_flood/envirnment.h>
#include <bug_flood/helper_functions.h>
#include <iostream>
#include <fstream>

/*
 * Environment is assumed to be a global variable
 * Alternative: If you don't have global environment add function parameter
 * */

#define STEP_SIZE 1

class Bug
{
public:

	Bug();
	Bug(Point location);
	virtual ~Bug();

	enum State
	{
		TOWARDS_GOAL,
		BOUNDARY_FOLLOWING,
		FINISHED,
		STUCK
	};

	void BoundaryFollow();
	void TowardsGoal();
	void Stuck();
	void Finished();
	Bug Split();

private:
	int identifier;
	Point location;
	State state;
	double cost;
	double heading;
	vector <Point> path;

	static int IDENTIFIER_COUNTER;
};

static int Bug::IDENTIFIER_COUNTER = 0;
#endif //BUG_FLOOD_BUG_H
