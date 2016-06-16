//
// Created by nishant on 6/5/16.
//

#include <bug_flood/bug.h>


int Bug::IDENTIFIER_COUNTER = 0;

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
	double distanceTempGoal = getEuclideanDistance(this->location, this->boundaryGoal);
	if(distanceTempGoal > STEP_SIZE)
	{
		this->heading = getDirectionAngleRadian(this->boundaryGoal, this->location);
		Point newLocation = this->simulateStep();
		this->cost += getEuclideanDistance(this->location, newLocation);
		this->location = newLocation;

		/*
		 * We decided to only check on corners
		 */
//		environment.setVisited(this->location, this->cost);

		//check if already visited (off for now)
		/*
		double isVisited = environment.isVisited(newLocation)
		if(isVisited != -1  && this->cost >= isVisited)
		{
			this->RE_VISITING;
		}
		*/

		//can we leave from this point, if yes the function will do it
		this->canLeaveBoundary(environment);
	}
	/**
	 * if we are about to reach the end of the boundary, we will check for visited,
	 * leave for goal, and change the boundary (in that order)
	 */
	else
	{
		//bug is at it's temp goal
		this->cost += getEuclideanDistance(this->location, this->boundaryGoal);
		this->location = this->boundaryGoal;
		//this->path.push_back(this->location);
//		//now we make it a leave point
//		double isVisited = environment.isVisited(this->location);
//		if(isVisited != -1  && this->cost >= isVisited)
//		{
//			this->state = RE_VISITING;
//			return;
//		}
//		environment.setVisited(this->location,this->cost);
		//is visited and can leave boundary
		bool canLeave = this->canLeaveBoundary(environment);
		if(!canLeave) //change direction to other side of the obstacle
		{
			Point tempLocation;
			tempLocation.x = this->location.x;
			tempLocation.y = this->location.y;
			tempLocation.z = this->location.z;
			environment.getNextBoundaryLine(this->location, this->onBoundary, this->boundaryGoal);
		}
		double isVisited = environment.isVisited(this->location);
		if(isVisited != -1  && this->cost > isVisited)
		{
			this->state = RE_VISITING;
		}
		else
		{
			environment.setVisited(this->location, this->cost);
		}

	}
}

bool Bug::canLeaveBoundary(Environment &environment)
{
	this->heading = getDirectionAngleRadian(environment.getGoal(), this->location);
	Point newLocation;
	newLocation.x = this->location.x + (0.1 * cos(this->heading));
	newLocation.y = this->location.y + (0.1 * sin(this->heading));
	//check if we can move towards goal now
	//is route to goal clear
//	Line line;
//	line.start = newLocation;
//	line.end = environment.getGoal();
//
//	Point intersection;
//	double distIntersection;
//	int intersectingBoundaryID = this->onBoundary;
//	bool isRouteBlocked = environment.getObstacleIntersection(line, intersection, distIntersection, intersectingBoundaryID);
	//we can move towards goal
	//the way is clear till goal or we can atleast move one step in goal's direction
	//if(!isRouteBlocked || !environment.isObstructed(newLocation))
	if(!environment.isObstructed(newLocation))
	{
		//now check if we are clear to leave from this point
		bool crossingPaths = this->isCrossingPaths(environment.getGoal());
		if(!crossingPaths)
		{
			//now we make it a leave point
			double isVisited = environment.isVisited(this->location);
			if(isVisited != -1  && this->cost > isVisited)
			{
				this->state = RE_VISITING;
				return true;
			}
			else
			{
				environment.setVisited(this->location, this->cost);
				Point tempLocation;
				tempLocation.x = this->location.x;
				tempLocation.y = this->location.y;
				tempLocation.z = this->location.z;
				this->path.push_back(tempLocation); //because it's a leave point
				//cout << this-> identifier << " " << this->location.x << " " << this->location.y << endl;
				//also move a step
//				this->cost += getEuclideanDistance(this->location, newLocation);
//				this->location = newLocation;
				this->state = TOWARDS_GOAL;
				return true;
			}
		}
		else
		{
			return false;
		}
	}
	return false;
}

bool Bug::isCrossingPaths(Point goal)
{
	Line line;
	line.start = this->location;
	line.end = goal;
	Point intersection;
	double distance;
	bool isIntersecting;
	for (int i = 1; i < this->path.size(); ++i)
	{

		Line pathLine;
		pathLine.start = this->path[i-1];
		pathLine.end = this->path[i];
		isIntersecting = IsIntersecting(line, pathLine, intersection, distance);
		if(isIntersecting)
		{
			return  true;
		}
	}
	return false;
}


void Bug::TowardsGoal(Environment &environment, vector<Bug> &bugList)
{
	//is route to goal clear
	Line line;
	line.start = this->location;
	line.end = environment.getGoal();

	Point intersection;
	double distIntersection;
	int intersectingBoundaryID = this->onBoundary;
	bool isRouteBlocked = environment.getObstacleIntersection(line, intersection, distIntersection, intersectingBoundaryID, this->location);

//	if (this->location.x == intersection.x && this->location.y == intersection.y) {
//		//set which direction it should move in now
//		isRouteBlocked = false;
//	}



	//move bug to goal and set it's state to finish
	if(!isRouteBlocked)
	{
		this->state = FINISHED;

		this->cost += getEuclideanDistance(this->location, environment.getGoal());
		this->location = environment.getGoal();

		this->path.push_back(this->location);
	}

	if (isRouteBlocked) //if there is a line in the way we can move to that line without any issue
	{

		this->state = BOUNDARY_FOLLOWING;
		this->onBoundary = intersectingBoundaryID;
		this->cost += getEuclideanDistance(this->location, intersection);
		this->location = intersection;
		this->path.push_back(this->location);

		//set visited location
		//environment.setVisited(intersection, this->cost);

		//now split and add bug to the list
		Bug newBug = this->split(environment);
		bugList.push_back(newBug);
	}


/**
 * DEPRECATED IMPLEMENTATION TO BE REMOVED AFTER TESTING
 */
//	//1. Calculate new heading for goal
//	double newHeading = getDirectionAngleRadian(environment.getGoal(),this->location);
//
//	//if heading is much different than previous heading then add point to path
//	if(abs(newHeading - this->heading) > HEADING_ERROR)
//	{
//		this->path.push_back(this->location);
//	}
//
//	this->heading = newHeading;
//
//
//
//	Point newLocation =	this->simulateStep();
//
//	bool obstructed = environment.isObstructed((void*)&newLocation);
//
//	/**
//	 * If obstructed just set state to boundary follow and then outside
//	 * function should be able to handle it
//	 */
//	if(obstructed)
//	{
//		this->state = BOUNDARY_FOLLOWING;
//	}
//	else //not obstructed free to move a point
//	{
//		this->cost += getEuclideanDistance(this->location,newLocation);
//		this->location = newLocation;
//	}
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
	for (int i = bugList.size() -1; i >= 0 ; --i)
	{
		if (bugList[i].getState() == Bug::FINISHED)
		{
			if(bugList[i].getCost() < bug.getCost())
				bug = bugList[i];
		}
		//the delete procedure
		if (bugList[i].getState() == Bug::FINISHED || bugList[i].getState() == Bug::STUCK || bugList[i].getState() == Bug::RE_VISITING)
		{
			bugList.erase(bugList.begin() + i);
		}
	}
}

Bug Bug::split(Environment &environment)
{
	Line boundary = environment.getLine(this->onBoundary);
	this->boundaryGoal = boundary.start;

	Bug newBug = *this;
	newBug.identifier = IDENTIFIER_COUNTER++;
	newBug.boundaryGoal = boundary.end;

	return newBug;
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

void Bug::setPath(vector<Point> path)
{
	this->path = path;
}
