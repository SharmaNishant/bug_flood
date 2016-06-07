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
	generateObstacleLineMap();
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

bool Environment::isObstructed(Point location)
{
//	Point loc = *(Point*)location;
	return this->map.isObstructed(location);
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

double Environment::isVisited	(double row, double col)
{
//	return this->map.isVisited(row,col); //Not using map based visited information anymore
	/*
	 * New approach
	 */
	for (VisitInfo &point : this->visited)
	{
		if(point.location.x == row && point.location.y == col)
			return point.cost;
	}
	return -1;
}

double Environment::isVisited	(Point location)
{
//	return this->map.isVisited(row,col); //Not using map based visited information anymore
	/*
	 * new approach
	 */
	for (VisitInfo &point : this->visited)
	{
		if(point.location.x == location.x && point.location.y == location.y)
			return point.cost;
	}
	return -1;
}

void Environment::setVisited(double row, double col, double cost)
{
//	this->map.setVisited(row,col);
	Point point;
	point.x = row;
	point.y = col;
	point.z = 0;
	this->setVisited(point,cost);
}

void Environment::setVisited(Point location, double cost)
{
	bool found = false;
	for (int i = 0; i < this->visited.size(); ++i)
	{
		if(visited[i].location.x == location.x && visited[i].location.y == location.y)
		{
			found = true;
			if(cost < visited[i].cost)
			{
				visited[i].cost = cost;
			}
		}
	}

	if(!found)
	{
		VisitInfo visitInfo;
		visitInfo.location = location;
		visitInfo.cost = cost;
		this->visited.push_back(visitInfo);
	}
}

bool Environment::getObstacleIntersection(Point start, Point end, Point &intersection, double &distance, int &boundaryID,Point &location )
{
	Line line;
	line.start = start;
	line.end = end;
	return this->getObstacleIntersection(line, intersection, distance, boundaryID, location);
}

bool Environment::getObstacleIntersection(Line line, Point &intersection, double &distance, int &boundaryID, Point &location)
{
	bool result = false;
	double minDistance = INT_MAX;
	Point minIntersection;
	int minBoundaryID = -1;
	for(auto &obsLine : this->lines)
	{
		if(obsLine.first == boundaryID) continue; //skip test for same line
		bool tresult = IsIntersecting(line, obsLine.second, intersection, distance);

		if (location.x == intersection.x && location.y == intersection.y) {
			//set which direction it should move in now
			continue;
		}


		if(distance < minDistance)
		{
			result = true;
			minDistance = distance;
			minIntersection = intersection;
			minBoundaryID = obsLine.first;
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
	for(auto &obsLine : this->lines)
	{
		if (obsLine.first == boundary) {
			continue;
		}
		if (obsLine.second.start.x == location.x && obsLine.second.start.y == location.y) {
			//set which direction it should move in now
			tempGoal = obsLine.second.end;
			boundaryID = obsLine.first;
			return true;
		}
		if (obsLine.second.end.x == location.x && obsLine.second.end.y == location.y) {
			//set which direction it should move in now
			tempGoal = obsLine.second.start;
			boundaryID = obsLine.first;
			return true;
		}
	}
	return false;
}

Line Environment::getLine(int id)
{
	ObstacleLines::iterator it;
	it = this->lines.find(id);
	if(it != this->lines.end())
	{
		return it->second;
	}
	else
	{
		assert(!"TRYING TO ACCESS A NON EXISTING LINE!!! EXITING");
	}
}

Line GenLine(double oneY, double oneX, double twoY, double twoX)
{
	Line line;
	line.start.x = oneX;
	line.start.y = oneY;
	line.start.z = 0;
	line.end.x = twoX;
	line.end.y = twoY;
	line.end.z = 0;
	return line;
}

void Environment::generateObstacleLineMap()
{
	this->lines[1] 	= GenLine(6  ,2  ,10 ,2);
	this->lines[2] 	= GenLine(6  ,2  ,6  ,4);
	this->lines[3] 	= GenLine(6  ,4  ,10 ,4);
	this->lines[4] 	= GenLine(10 ,4  ,10 ,2);

	this->lines[5] 	= GenLine(4  ,6  ,4  ,13);
	this->lines[6] 	= GenLine(4  ,13 ,13 ,13);
	this->lines[7] 	= GenLine(13 ,13 ,13 ,6);
	this->lines[9] 	= GenLine(13 ,6  ,11 ,6);
	this->lines[10] = GenLine(11 ,6  ,11 ,11);
	this->lines[11] = GenLine(11 ,11 ,6  ,11);
	this->lines[12] = GenLine(6  ,11 ,6  ,6);
	this->lines[8] 	= GenLine(6  ,6  ,4  ,6);


}