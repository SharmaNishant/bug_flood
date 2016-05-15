//
// Created by nishant on 15/5/16.
//

#include <bug_flood/map.h>
#include <bug_flood/line.h>

BinaryMap::BinaryMap()
{
	this->rowSize = 0;
	this->colSize = 0;
	this->isAllocated = false;
}

void BinaryMap::BinaryMapInit(int rowSize, int colSize)
{
	this->rowSize = rowSize;
	this->colSize = colSize;

	binary_map = new bool*[colSize];
	for(int i = 0; i < rowSize; ++i)
		binary_map[i] = new bool[rowSize];
	this->isAllocated = true;
}

BinaryMap::~BinaryMap()
{
	//if new was not used in the first place
	if(this->isAllocated)
	{
		for (int i = 0; i < this->rowSize; ++i) {
			delete[] binary_map[i];
		}
		delete[] binary_map;
		this->isAllocated = false;
	}
}

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