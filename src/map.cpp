//
// Created by nishant on 15/5/16.
//

#include <bug_flood/map.h>
#include <fstream>
#include <bug_flood/helper_functions.h>
#include <cmath>

Map::Map(int rowSize, int colSize)
{
	this->rowSize 		= rowSize;
	this->colSize 		= colSize;
	this->isAllocated 	= false;
	this->map 			= nullptr;
}

Map::Map(string filename)
{
	readMap(filename);
}

void Map::readMap(string filename)
{
	ifstream infile(filename);
	if(!infile.is_open())
	{
		cout<<"Cannot Open Map File. Exiting...";
		exit(-1);
	}

	std::string line;
	vector<string> splittedLine;

	//read environment size first
	getline(infile, line);
	splittedLine = split(line,' ');
	this->rowSize = stoi(splittedLine[0]);
	this->colSize = stoi(splittedLine[1]);

	//allocate space
	map = new bool [rowSize * colSize];
	this->isAllocated = true;

	//to make sure we don't go beyond the specified ranges
	int rowCounter = 0;
	int colCounter = 0;
	while (std::getline(infile, line))
	{
		if(rowCounter >= rowSize)
			assert(!"Trying to save a bigger map than allocated memory for!!!");

		colCounter = 0;
		splittedLine = split(line, ' ');
		for (string str : splittedLine)
		{
			if(colCounter >= colSize)
			{
				assert(!"Trying to save a bigger map than allocated memory for!!!");
			}

			int bit = stoi(str);

			if (bit == 0)		map[(rowCounter * rowSize) + colCounter] = false;
			else if (bit == 1)	map[(rowCounter * rowSize) + colCounter] = true;
			else assert( (bit == 1 || bit == 0 ) && "Map values are out of range");

			++colCounter;
		}
		++rowCounter;
	}
	infile.close();
}

void Map::MapInit(int rowSize, int colSize)
{
	this->rowSize = rowSize;
	this->colSize = colSize;

	map = new bool [rowSize * colSize];
	this->isAllocated = true;
}

Map::~Map()
{
	//if new was not used in the first place
	if(this->isAllocated)
	{
		delete[] map;
		this->isAllocated = false;
	}
}

void Map::setMap(vector<vector<bool> > &_map)
{
	//check for map dimensions first
	if(rowSize < _map.size())
		assert(!"Trying to save a bigger map than allocated memory for!!!");
	if(!_map.empty() && colSize < _map[0].size())
		assert(!"Trying to save a bigger map than allocated memory for!!!");

	for(int row = 0; row < _map.size() ; row++)
	{
		for (int col = 0; col < _map[row].size() ; col++)
		{
			map[(row * rowSize) + col]  =  _map[row][col];
		}
	}
}

vector<Point> Map::getObstructedLocations(int &rowSize, int &colSize)
{
	vector <Point> obstructed;
	for (int i = 0; i < rowSize; ++i)
	{
		for (int j = 0; j < colSize; ++j)
		{
			if(map[(i * rowSize)+j])
			{
				Point point;
				point.x = i;
				point.y = j;
				point.z = 0;
				obstructed.push_back(point);
			}
		}
	}

	rowSize = this->rowSize;
	colSize = this->colSize;

	return obstructed;
}

bool Map::operator()(int row, int col)
{
	//outside of environment is always obstacle
	if ((0 > row || row > this->rowSize) && (0 > col || col > this->colSize))
	{
		return true;
	}

	return map[(row * rowSize) + col];
}

//we should floor function here
bool Map::operator()(Point location)
{
	int x = (int) floor(location.x);
	int y = (int) floor(location.y);
	//outside of environment is always obstacle
	if ((0 > x || x > this->rowSize) && (0 > y || y > this->colSize))
	{
		return true;
	}

	return map[ ((int)floor(location.x) * rowSize) + (int)floor(location.y)];
}


bool Map::at(int row, int col)
{
	//outside of environment is always obstacle
	if ((0 > row || row > this->rowSize) && (0 > col || col > this->colSize))
	{
		return true;
	}
	return map[(row * rowSize) + col];
}

//we should floor function here
bool Map::at(Point location)
{
	int x = (int) floor(location.x);
	int y = (int) floor(location.y);
	//outside of environment is always obstacle
	if ((0 > x || x > this->rowSize) && (0 > y || y > this->colSize))
	{
		return true;
	}

	return map[ ((int)floor(location.x) * rowSize) + (int)floor(location.y)];
}


bool Map::isObstructed(int row, int col)
{
	//outside of environment is always obstacle
	if ((0 > row || row > this->rowSize) && (0 > col || col > this->colSize))
	{
		return true;
	}
	return map[(row * rowSize) + col];
}

//we should floor function here
bool Map::isObstructed(Point location)
{
	int x = (int) floor(location.x);
	int y = (int) floor(location.y);
	//outside of environment is always obstacle
	if ((0 > x || x > this->rowSize) && (0 > y || y > this->colSize))
	{
		return true;
	}

	return map[ ((int)floor(location.x) * rowSize) + (int)floor(location.y)];
}

int Map::getColSize()
{
	return colSize;
}

int Map::getRowSize()
{
	return rowSize;
}