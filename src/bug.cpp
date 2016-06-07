//
// Created by nishant on 6/5/16.
//

#include <bug_flood/bug.h>

Bug::Bug()
{
	this->identifier = IDENTIFIER_COUNTER++;
	this->state = TOWARDS_GOAL;
	this->cost = 0;
}

Bug::Bug(Point location)
{
	this->identifier = IDENTIFIER_COUNTER++;
	this->state = TOWARDS_GOAL;
	this->cost = 0;
	this->location = location;
	this->path.push_back(location);
}

Bug::~Bug() {}

void Bug::BoundaryFollow(Environment &environment)
{

}


void Bug::TowardsGoal(Environment &environment, vector<Bug> &bugList)
{
	//1. Calculate new heading for goal
	double newHeading = getDirectionAngleRadian(environment.getGoal(),this->location);

	//if heading is much different than previous heading then add point to path
	if(abs(newHeading - this->heading) > HEADING_ERROR)
	{
		this->path.push_back(this->location);
	}

	this->heading = newHeading;

	Point newLocation =	this->simulateStep();

	bool obstructed = environment.isObstructed((void*)&newLocation);

	/**
	 * If obstructed just set state to boundary follow and then outside
	 * function should be able to handle it
	 */
	if(obstructed)
	{
		this->state = BOUNDARY_FOLLOWING;

	}
	else //not obstructed free to move a point
	{
		this->cost += getEuclideanDistance(this->location,newLocation);
		this->location = newLocation;
	}
}

void Bug::DumpPath(string filename)
{
	ofstream out(filename);
	if(out.is_open())
	{
		for(Point &point: this->path)
		{
			out << point.x << " " << point.y << endl;
		}
		out.close();
	}
	else
	{
		cerr << "Unable to open path dump file. Skipping..." << endl;
	}
}


Point Bug::simulateStep()
{
	Point nextLocation;
	nextLocation.x = this->location.x + (STEP_SIZE * cos(this->heading));
	nextLocation.y = this->location.y + (STEP_SIZE * sin(this->heading));
	return nextLocation;
}


void KillBugs(vector<Bug> &bugList, Bug &bug)
{

}

Bug Bug::Split(Environment &environemnt)
{
	return Bug(this->location);
}


Bug::State Bug::getState()
{
	return this->state;
}

vector<Point> Bug::getpath()
{
	return this->path;
}
double Bug::getCost()
{
	return this->cost;
}
Point Bug::getLocation()
{
	return this->location;
}
void Bug::setCost(double cost)
{
	this->cost = cost;
}
