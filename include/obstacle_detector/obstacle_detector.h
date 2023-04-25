#ifndef obstacle_detector_H
#define obstacle_detector_H

#include "ros/ros.h"
#include "tf2/utils.h"
//#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

class ObstacleMapCreator {
public:
    ObstacleMapCreator();

    void process();

private:
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);

    void odo_callback(const nav_msgs::Odometry::ConstPtr &msg);

    void add_obstacles_to_map(double angle, double laser_range);

    void update_obstacle_poses();

    void create_obstacle_map();

    void init_map();

    int xy_to_map_index(double x, double y);

    bool is_within_obstacle_map(double x, double y);

    bool is_ignore_angle(double angle);

    int hz_;
    double map_size_;
    double map_gridSize_;

    bool first_scan_done_ = false;
    bool is_map_initialized_ = false;
    bool first_odometry_got_ = false;
    bool second_odometry_got_ = false;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber odo_sub_;
    ros::Subscriber laser_sub_;
    ros::Publisher obstacle_map_pub_;

    sensor_msgs::LaserScan laser_scan_;
    nav_msgs::OccupancyGrid obstacle_map_;
    nav_msgs::Odometry current_odo_;
    nav_msgs::Odometry previous_odo_;
    geometry_msgs::Pose diff_;
    geometry_msgs::PoseArray obstacle_poses_;
    geometry_msgs::PoseArray updated_obstacle_poses_;
    geometry_msgs::Pose obstacle_pose_;
    geometry_msgs::Pose updated_obstacle_pose_;
};

#endif

