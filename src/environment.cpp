//
// Created by nishant on 15/5/16.
//

#include <bug_flood/environment.h>
#include <bug_flood/helper_functions.h>

/**
#ifdef OBSTACLE_IO_DEBUG

#endif
 */

Environment::Environment(string sourceGoal, int length, int width): map(length,width)
{
	ReadSourceGoal(sourceGoal);
}

Environment::Environment(string sourceGoal, string mapFile): map(mapFile)
{
	ReadSourceGoal(sourceGoal);
}


Point Environment::getSource()
{
	return this->source;
}

Point Environment::getGoal()
{
	return this->goal;
}

int Environment::getEnvironmentLength()
{
	return this->map.getRowSize();
}

int Environment::getEnvironmentWidth()
{
	return this->map.getColSize();
}

void Environment::getEnvironmentDimensions(int &length, int &width)
{
	length = getEnvironmentLength();
	width = getEnvironmentWidth();
}

vector<Point> Environment::getObstructedLocations(int &rowSize, int &colSize)
{
	return this->map.getObstructedLocations(rowSize,colSize);
}

bool Environment::isObstructed(void* location)
{
	Point loc = *(Point*)location;
	return this->map.isObstructed(loc);
}


void Environment::ReadSourceGoal(string sourceGoal)
{
	ifstream infile(sourceGoal);
	if(!infile.is_open())
	{
		cout<<"Cannot Open Source Goal file File. Exiting.....";
		exit(-1);
	}

	std::string line;
	vector<string> splittedLine;

	//read source
	getline(infile, line);
	splittedLine = split(line, ' ');
	this->source.x = stoi(splittedLine[0]);
	this->source.y = stoi(splittedLine[1]);
	this->source.z = 0;

	//read goal
	getline(infile, line);
	splittedLine = split(line, ' ');
	this->goal.x = stoi(splittedLine[0]);
	this->goal.y = stoi(splittedLine[1]);
	this->goal.z = 0;
	infile.close();
}

bool Environment::isVisited	(int row, int col)
{
	return this->map.isVisited(row,col);
}

bool Environment::isVisited	(Point location)
{
	return this->map.isVisited(location);
}

void Environment::setVisited(int row, int col)
{
	this->map.setVisited(row,col);
}

void Environment::setVisited(Point location)
{
	this->map.setVisited(location);
}

bool Environment::getObstacleIntersection(Point start, Point end, Point &intersection, double &distance, int &boundaryID)
{
	Line line;
	line.start = start;
	line.end = end;
	return this->getObstacleIntersection(line, intersection, distance, boundaryID);
}

bool Environment::getObstacleIntersection(Line line, Point &intersection, double &distance, int &boundaryID)
{
	bool result;
	double minDistance = INT_MAX;
	Point minIntersection;
	int minBoundaryID;
	for(Obstacle &obstacle : this->obstacleList)
	{
		for(ObstacleLine &obstacleLine : obstacle)
		{
			result = IsIntersecting(line, obstacleLine.line, intersection, distance);
			if(distance < minDistance)
			{
				minDistance = distance;
				minIntersection = intersection;
			}
		}
	}

	//set the min distance once we are done
	distance = minDistance;
	intersection = minIntersection;
	boundaryID = minBoundaryID;
	return  result;
}

bool Environment::getNextBoundaryLine(Point location, int &boundaryID, Point &tempGoal)
{
	//copy boundary id for static reference
	int boundary = boundaryID;
	for(Obstacle &obstacle : this->obstacleList)
	{
		for(ObstacleLine &obstacleLine : obstacle)
		{
			if(obstacleLine.boundaryID == boundary)
			{
				continue;
			}
			if(obstacleLine.line.start.x == location.x && obstacleLine.line.start.y == location.y)
			{
				//set which direction it should move in now
				tempGoal = obstacleLine.line.end;
				boundaryID = obstacleLine.boundaryID;
				return true;
			}
			if(obstacleLine.line.end.x == location.x && obstacleLine.line.end.y == location.y)
			{
				//set which direction it should move in now
				tempGoal = obstacleLine.line.start;
				boundaryID = obstacleLine.boundaryID;
				return true;
			}
		}
	}
	return false;
}


