//
// Created by nishant on 15/5/16.
//

#include <bug_flood/helper_functions.h>

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}


std::vector<std::string> split(const std::string &s, char delim)
{
std::vector<std::string> elems;
split(s, delim, elems);
return elems;
}

//bool IsIntersecting(Line first, Line second, Point &intersection)
//{
//
//	//defining start points
//	float p0_x = first.start.x;
//	float p0_y = first.start.y;
//	float p1_x = first.end.x;
//	float p1_y = first.end.y;
//
//	//defining end points
//	float p2_x = second.start.x;
//	float p2_y = second.start.y;
//	float p3_x = second.end.x;
//	float p3_y = second.end.y;
//
//	//resetting intersection to a predefined null value
//	intersection.x = INT_MAX;
//	intersection.y = INT_MAX;
//	intersection.z = 0;
//
//	float s1_x, s1_y, s2_x, s2_y;
//	s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
//	s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;
//
//	float s, t;
//	s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
//	t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);
//
//	float i_x,i_y;
//
//	if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
//	{
//		// Collision detected
//		i_x = p0_x + (t * s1_x);
//		i_y = p0_y + (t * s1_y);
//
//		intersection.x = i_x;
//		intersection.y = i_y;
//		return true;
//	}
//
//	return false; // No collision
//}

double getEuclideanDistance(Point first, Point second)
{
	return sqrt(pow(first.x - second.x, 2) + pow(first.y - second.y, 2));
}