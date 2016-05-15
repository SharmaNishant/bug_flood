//
// Created by nishant on 15/5/16.
//

#include <bug_flood/obstacle_io.h>
#include <bug_flood/helper_functions.h>

/**
#ifdef OBSTACLE_IO_DEBUG

#endif
 */

ObstacleIO::ObstacleIO(string sourceGoal, string obstacle)
{
	ReadSourceGoal(sourceGoal);
	ReadObstacleList(obstacle);
}

ObstacleIO::ObstacleIO(string sourceGoal, string obstacle, string map)
{
	ReadSourceGoal(sourceGoal);
	ReadObstacleList(obstacle);
	ReadBinaryMap(map);
}

//BinaryMap ObstacleIO::getBinaryMap()
//{
//	return this->binaryMap;
//}

bool ObstacleIO::isObstructed(Point location)
{
	return this->binaryMap.isObstructed(location);
}


ObstacleList ObstacleIO::getObstacleList()
{
	return this->obstacleList;
}

Line ObstacleIO::getObstacleLine(int obstacleIndex, int lineIndex)
{
	return this->obstacleList.at(obstacleIndex).at(lineIndex);
}

bool ObstacleIO::getNextLine(int obstacleIndex, int lineIndex, Point location, Line &nextLine)
{
	Line currentLine = this->getObstacleLine(obstacleIndex, lineIndex);
	//if bug is at the end of the line return next one from the list
	if(currentLine.isAtEnd(location))
	{
		int nextLineIndex = (lineIndex++) % this->obstacleList.at(obstacleIndex).size();
		nextLine = this->getObstacleLine(obstacleIndex, nextLineIndex);
		return true;
	}
	//if bug is the start of the line return previous one in the list
	if(currentLine.isAtStart(location))
	{
		int nextLineIndex = (lineIndex--) % this->obstacleList.at(obstacleIndex).size();
		nextLine = this->getObstacleLine(obstacleIndex, nextLineIndex);
		return true;
	}
	return false;
}

Point ObstacleIO::getSource()
{
	return this->source;
}

Point ObstacleIO::getGoal()
{
	return this->goal;
}

int ObstacleIO::getRowSize()
{
	return this->binaryMap.rowSize;
}

int ObstacleIO::getColSize()
{
	return this->binaryMap.colSize;
}

void ObstacleIO::ReadBinaryMap(string map)
{
	ifstream infile(map);
	if(!infile.is_open())
	{
		cout<<"Cannot Open Map File. Exiting.....";
		exit(-1);
	}

	std::string line;
	vector<string> splittedLine;
	vector< vector <bool> > _map;

	while (std::getline(infile, line))
	{
		vector<bool> _map_row;
		splittedLine = split(line, ' ');
		for (string str : splittedLine)
		{
			int bit = stoi(str);
			if (bit == 0) _map_row.push_back(false);
			else if (bit == 1) _map_row.push_back(true);
			else assert( (bit == 1 || bit == 0 ) && "Map values are out of range");
		}
		_map.push_back(_map_row);
	}

	int rowSize = _map.size();
	int colSize = _map.at(rowSize-1).size();

	this->binaryMap.BinaryMapInit(rowSize,colSize);
	for(int i=0;i<rowSize;i++)
	{
		for(int j=0;j<colSize;j++)
		{
			this->binaryMap.binary_map[i][j] = _map.at(i).at(j);
		}
	}
}

void ObstacleIO::ReadObstacleList(string obstacle)
{
	ifstream infile(obstacle);
	if(!infile.is_open())
	{
		cout<<"Cannot Open Obstacle file File. Exiting.....";
		exit(-1);
	}

	std::string line;
	vector<string> splittedLine;

	Obstacle obstacleObj;
	while (std::getline(infile, line))
	{
		if(!line.compare("-----------"))
		{
			this->obstacleList.push_back(obstacleObj);
			obstacleObj.clear();
			continue;
		}
		splittedLine = split(line, ',');
		Line lineObj;
		lineObj.start.x = stoi(splittedLine[0]);
		lineObj.start.y = stoi(splittedLine[1]);
		lineObj.start.z = 0;
		lineObj.end.x = stoi(splittedLine[2]);
		lineObj.end.y = stoi(splittedLine[3]);
		lineObj.end.z = 0;
		obstacleObj.push_back(lineObj);
	}
}

void ObstacleIO::ReadSourceGoal(string sourceGoal)
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
}