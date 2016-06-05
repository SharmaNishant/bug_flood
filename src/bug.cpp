//
// Created by nishant on 6/5/16.
//

#include <bug_flood/bug.h>

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
}

Bug::~Bug()
{

}

Bug Bug::Split()
{
	Bug newBug = *this;
	return newBug;
}
















