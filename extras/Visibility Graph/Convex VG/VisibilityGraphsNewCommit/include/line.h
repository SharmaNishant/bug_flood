#ifndef LINE_H_INCLUDED
#define LINE_H_INCLUDED

#include <iostream>
#include "point.h"
#include "geometry.h"
#include <cmath>
#include <cstdlib>
#include "boostHelper.h"

class Line: public Geometry
{
 public:
	Point * a;
    Point * b;
    tLinestring line;

	int id;
	long double dist; // distance from center
	double theta_cache; // used for deciding if the dist cache needs to be refreshed
	double m; // slope of line
	double y_intercept; // y-intercept of line
	
	Line();
	Line(double _x1, double _y1, double _x2, double _y2);
	Line(Point* p1,Point* p2);
	Line (const Line &other)
	{
		this->a = new Point(other.a->x,other.a->y);
		this->b = new Point(other.b->x,other.b->y);
		this->id = other.id;
		this->line= other.line;
		this->dist = other.dist;
		this->theta_cache=other.theta_cache;
		this->m=other.m;
		this->y_intercept=other.y_intercept;
		std::cout<<"Line Copy called"<<std::endl;
	};
	~Line();
	virtual void print();
    virtual double value();

	void updateCalcs();
	void distance();
	void center_intercept(double &xi, double &yi);
};


// This global needs to be visible to classes:
extern Point * center;
extern Line * center_line;
extern double atomic;
extern double atomic_space;

#endif
