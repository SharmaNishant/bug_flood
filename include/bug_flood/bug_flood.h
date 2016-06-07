//
// Created by nishant on 15/5/16.
//

#ifndef BUG_FLOOD_BUG_FLOOD_H
#define BUG_FLOOD_BUG_FLOOD_H

#define PUBLISH_PATH

#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <bug_flood/bug.h>
typedef visualization_msgs::Marker Marker;

using namespace std;

#ifdef PUBLISH_PATH
void initMarkers(Marker &bugs, Marker &pathTree);
void publish_bugs(vector<Bug> &bugList, ros::Publisher &publisher, Marker &bugs, Marker &pathTree);
void publish_bug(Bug &bug, ros::Publisher &publisher, Marker &pathTree);
#endif


#endif //BUG_FLOOD_BUG_FLOOD_H
