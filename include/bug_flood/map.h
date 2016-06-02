//
// Created by nishant on 15/5/16.
//

#ifndef BUG_FLOOD_MAP_H
#define BUG_FLOOD_MAP_H

#include <vector>
#include <geometry_msgs/Point.h>

using namespace std;

typedef geometry_msgs::Point Point;


/***
 * NEVER GIVE THE POINTER OF THE map MEMBER VARIABLE TO ANYONE ELSE. OTHERWISE BRACE SEGFAULTS.
 *
 * If We want to give someone a getMAP function we need to do it without giving out the pointer.
 * Current implementation will return a vector of locations (bottom left) of a 1x1 square of the
 * 		map with rowSize and col size as referenced variables.
 */
class Map
{
public:
	/*
	 * You can either create a blank map and then populate it later or read a map.
	 */

	Map(int rowSize, int colSize);
	Map(string filename);
	virtual ~Map();

	/* public functions */
	void setMap		(vector < vector <bool> > &map);
	vector<Point> getObstructedLocations(int &rowSize, int &colSize);

	bool operator() 	(int row, int col);
	bool operator() 	(Point location); //we will take floor for double values

	bool at				(Point location);
	bool at				(int row, int col);

	bool isObstructed	(int row, int col);
	bool isObstructed	(Point location);

	int  getRowSize		();
	int  getColSize		();

private:
	bool	*map; //2d array allocated as one block
	bool	isAllocated;
	int		rowSize;
	int		colSize;
	void	MapInit(int rowSize, int colSize);
	void 	readMap(string filename);
};

//typedef BinaryMap Map;

#endif //BUG_FLOOD_MAP_H
