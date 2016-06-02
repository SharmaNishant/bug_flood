//
// Created by nishant on 15/5/16.
//

#include <bug_flood/envirnment.h>
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

bool Environment::isObstructed(Point location)
{
	return true;
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
	splittedLine = split(line, ',');
	this->source.x = stoi(splittedLine[0]);
	this->source.y = stoi(splittedLine[1]);
	this->source.z = 0;

	//read goal
	getline(infile, line);
	splittedLine = split(line, ',');
	this->goal.x = stoi(splittedLine[0]);
	this->goal.y = stoi(splittedLine[1]);
	this->goal.z = 0;
	infile.close();
}