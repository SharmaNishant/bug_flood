//
// Created by nishant on 30/5/16.
//

#ifndef BUG_FLOOD_BUG_H
#define BUG_FLOOD_BUG_H

#include <bug_flood/environment.h>
#include <bug_flood/helper_functions.h>
#include <iostream>
#include <fstream>

/*
 * Environment is assumed to be a global variable
 * Alternative: If you don't have global environment add function parameter
 * */

using namespace std;

#define STEP_SIZE 1
#define HEADING_ERROR 0.0001


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
		STUCK,
		RE_VISITING
	};

	void BoundaryFollow(Environment &environment);
	void TowardsGoal(Environment &environment, vector <Bug> &bugList);
	void DumpPath(string filename);
	Bug split(Environment &environment);


	State getState();
	vector<Point> getpath();
	double getCost();
	Point getLocation();
	void setCost(double cost);
	void setPath(vector<Point> path);

private:
	int identifier;
	Point location;
	State state;
	double cost;
	double heading;
	vector <Point> path;
	int onBoundary;
	Point boundaryGoal;

	static int IDENTIFIER_COUNTER;

	bool isCrossingPaths(Point goal);

	bool canLeaveBoundary(Environment & environment);

	Point simulateStep();
};

void KillBugs(vector<Bug> &bugList, Bug &bug);

#endif //BUG_FLOOD_BUG_H
