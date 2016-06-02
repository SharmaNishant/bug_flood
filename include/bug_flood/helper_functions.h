//
// Created by nishant on 15/5/16.
//

#ifndef BUG_FLOOD_HELPER_FUNCTIONS_H
#define BUG_FLOOD_HELPER_FUNCTIONS_H

#include <string>
#include <sstream>
#include <vector>

#include <geometry_msgs/Point.h>

typedef geometry_msgs::Point Point;

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim) ;


//bool IsIntersecting(Line first, Line second, Point &intersection);
double getEuclideanDistance(Point first, Point second);

#endif //BUG_FLOOD_HELPER_FUNCTIONS_H
