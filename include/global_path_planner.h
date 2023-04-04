#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

using namespace std;

struct Node {
    int x, y;
    double f, g, h;

    Node(int x, int y, double g, double h) : x(x), y(y), g(g), h(h), f(g + h) {}
};

struct CompareNode {
    bool operator()(const Node &a, const Node &b) {
        return a.f > b.f;
    }
};

class AstarPath
{
  public:
    vector<pair<int, int> > a_star();

  private:
    void map_callback();
    bool is_valid();
    double heuristic();
    void map_callback(const nav_msgs::OccuancyGrid::ConstPtr &msg);
    vector<std::vector<int>> map_grid;
    pair<int, int> start;
    pair<int, int> goal;
    vector<std::vector<int>> global_path;


    int hz;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber sub_map;
    ros::Publisher pub_map;
    ros::Publisher pub_path;
    ros::Publisher pub_goal;

    nav_msgs::OccupancyGrid the_map;         //house map
    nav_msgs::Path global_path;             //マップ全体でのパス
    nav_msgs::Path checkpoint_path;               //チェックポイントまでのパス
    geometry_msgs::PoseStamped goal_point;

#endif