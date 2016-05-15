//
// Created by nishant on 15/5/16.
//

#include <bug_flood/line.h>
#include <bug_flood/helper_functions.h>

#define MAX_ERROR 0.005

bool Line::CheckIntersection(Line other, Point &intersection)
{
	return IsIntersecting(*this, other, intersection);
}



bool Line::isAtEnd(Point location)
{
	if(		(abs(this->end.x - location.x) < MAX_ERROR)
		&&  (abs(this->end.y - location.y) < MAX_ERROR))
	{
		return true;
	}
	return false;
}

bool Line::isAtStart(Point location)
{
	if(	(abs(this->start.x - location.x) < MAX_ERROR)
	   &&  (abs(this->start.y - location.y) < MAX_ERROR))
	{
		return true;
	}
	return false;
}


