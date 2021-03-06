//
// Created by nishant on 15/5/16.
//

#ifndef BUG_FLOOD_HELPER_FUNCTIONS_H
#define BUG_FLOOD_HELPER_FUNCTIONS_H

#include <bug_flood/environment.h>
#include <bug_flood/type_decls.h>
#include <sstream>
#include <string>
#include <vector>

std::vector<std::string> &split(const std::string &s, char delim,
                                std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim);

bool IsIntersecting(Line first, Line second, Point &intersection,
                    double &distance);
void getPerpendicularLineIntersection(Line line, Point point,
                                      Point &intersection);

double getEuclideanDistance(Point first, Point second);
double getDirectionAngleRadian(Point goal, Point source);
double getDirectionAngleDegrees(Point goal, Point source);
void prunePathLast(vector<Point> &inPath, Environment &environment,
                   double &cost);
void prunePathFirst(vector<Point> &inPath, Environment &environment,
                    double &cost);

#endif // BUG_FLOOD_HELPER_FUNCTIONS_H
