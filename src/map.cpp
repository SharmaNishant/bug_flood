//
// Created by nishant on 15/5/16.
//

#include <bug_flood/map.h>
#include <fstream>
#include <bug_flood/helper_functions.h>


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

vector<Point> Map::getObstructedLocations(int &rowSize, int colSize);




bool BinaryMap::isObstructed(Point location)
{
	int x = (int) floor(location.x);
	int y = (int) floor(location.y);

	if ((0 > x || x > this->rowSize) && (0 > y || y > this->colSize))
	{
		return true;
	}

	return binary_map[x][y];
}