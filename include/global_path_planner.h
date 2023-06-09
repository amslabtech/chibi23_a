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
    AstarPath();
    void process();
    void convert_map_position_to_node_index();

  private:
    vector<pair<int, int> > a_star(vector<vector<int>> &map_grid, pair<int, int> start, pair<int, int> goal);
    vector<pair<int, int> > path_for_multi_goal();
    bool                    isValid(int x, int y, int rows, int cols);
    double                  heuristic(int x1, int y1, int x2, int y2);
    void                    map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void                    update_map();
    void                    assign_global_path_msgs();

    int hz;
    vector<vector<int>>    map_grid;
    vector<pair<int, int>> global_path;
    pair<double, double>         origin; // マップの原点
    pair<int, int>         origin_node;
    bool                   map_check;
    pair<double, double>         start_position;
    vector<pair<double, double>> goal_positions;
    pair<int, int>         start;
    vector<pair<int, int>> goals;
    float                  map_resolution;
    int                    map_row_length;
    int                    map_col_length;

    ros::NodeHandle            nh;
    ros::NodeHandle            private_nh;
    ros::Subscriber            sub_map;
    ros::Publisher             pub_map;
    ros::Publisher             pub_path;
    ros::Publisher             pub_goal;
    nav_msgs::OccupancyGrid    the_map;
    nav_msgs::Path             global_path_msgs;
};

#endif
