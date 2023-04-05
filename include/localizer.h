#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/utils.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <algorithm>
#include <random>

#include "roomba_500driver_meiji/RoombaCtrl.h"

class Particle
{
public:
    Particle(double x=0.0, double y=0.0, double yaw=0.0, double weight=0.0);

    double get_pose_x() const {return x_;}
    double get_pose_y() const {return y_;}
    double get_pose_yaw() const {return yaw_;}
    double get_pose_weight() const {return weight_;}

private:
    double x_;
    double y_;
    double yaw_;
    double weight_;
}

class Localizer
{
public:
    Localizer();
    void process;

private:
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    double set_noise(double mu, double dev);

    void initialize();

    void publish_particles();

    int hz_;
    int num_;
    double init_x_;
    double init_y_;
    double init_yaw_;
    double init_dev_;

    bool odometry_got_ = false;
    bool map_got_ = false;
    bool laser_got_ = false;
    bool can_move_ = false;

    Particle estimated_pose_;
    std::vector<Particle> particles_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_laser_;
    ros::Subscriber sub_odometry_;
    ros::Subscriber sub_map_;

    ros::Publisher pub_estimated_pose;
    ros::Publisher pub_particle_cloud;

    tf::TrasformBroadcaster roomba_state_broadcaster_;

    geometry_msgs::PoseStamped estimated_pose_msg_;
    geometry_msgs::PoseArray particle_cloud_msg_;

    nav_msgs::Odometry current_odometry_;
    nav_msgs::Odometry previous_odometry_;
    nav_msgs::OccupancyGrid map_;

    sensor_msgs::LaserScan laser_;

};

#endif
