//
// Created by nishant on 15/5/16.
//

#ifndef BUG_FLOOD_BUG_FLOOD_H
#define BUG_FLOOD_BUG_FLOOD_H

#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <bug_flood/bug.h>
#include <bug_flood/type_decls.h>
typedef visualization_msgs::Marker Marker;

using namespace std;

#define PUBLISH_PATH

#ifdef PUBLISH_PATH
void initMarkers(Marker &bugs, Marker &pathTree);
void publish_bugs(vector<Bug> &bugList, ros::Publisher &publisher, Marker &bugs, Marker &pathTree);
void publish_bug(Bug &bug, ros::Publisher &publisher, Marker &pathTree);
#endif

#define KINEMATIC_BUG

#ifdef KINEMATIC_BUG

#define RABBIT_STEP_SIZE 0.5

bool updateCarrotWhenMovingOnLine(Line line, Point &rabbit, Point &carrot);
void updateRabbit(Point &rabbit, Point carrot);
void pruneRabbitCarrot(vector<Point> &inPath, double &cost);
#endif

#endif //BUG_FLOOD_BUG_FLOOD_H
