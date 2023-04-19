#include "obstacle_detector/obstacle_detector.h"

//chibi22_bのソースコードを参考に作成
ObstacleMapCreator::ObstacleMapCreator() : private_nh_("~") {
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("map_size", map_size_, {6}); //周囲の障害物情報を保持するマップ（obstacle_map）の大きさ
    private_nh_.param("map_gridSize", map_gridSize_, {0.02}); //obstacle_mapの1マスの大きさ

    odo_sub_ = nh_.subscribe("/roomba/odometry", 100, &ObstacleMapCreator::odo_callback, this);
    laser_sub_ = nh_.subscribe("scan", 10, &ObstacleMapCreator::laser_callback, this);
    obstacle_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("obstacle_map", 10);

    obstacle_poses_.header.frame_id = "base_link";
    obstacle_map_.header.frame_id = "base_link";
    obstacle_map_.info.resolution = map_gridSize_;
    obstacle_map_.info.width = map_size_ / map_gridSize_;
    obstacle_map_.info.height = map_size_ / map_gridSize_;
    obstacle_map_.info.origin.position.x = -map_size_ / 2;
    obstacle_map_.info.origin.position.y = -map_size_ / 2;
    obstacle_map_.data.reserve(obstacle_map_.info.width * obstacle_map_.info.height);
}

void ObstacleMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser_scan_ = *msg;
    //ROS_INFO("scan"); //for debug
    
    //初回スキャン時のみマップを初期化
    if (!is_map_initialized_) {
        init_map();
        is_map_initialized_ = true;
    }

    //初回のスキャンが完了し、/roomba/odometryの購読が2回以上行われた時、マップを更新
    if (first_scan_done_ && second_odometry_got_) update_obstacle_poses();

    create_obstacle_map();
    first_scan_done_ = true;
}

void ObstacleMapCreator::odo_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_odo_ = *msg;

    if (first_odometry_got_) {
        diff_.position.x = current_odo_.pose.pose.position.x - previous_odo_.pose.pose.position.x;
        diff_.position.y = current_odo_.pose.pose.position.y - previous_odo_.pose.pose.position.y;
        second_odometry_got_ = true;
    } else {
        first_odometry_got_ = true;
    }

    previous_odo_ = current_odo_;
}

void ObstacleMapCreator::init_map() {
    obstacle_map_.data.clear();
    int size = obstacle_map_.info.width * obstacle_map_.info.height;
    for (int i = 0; i < size; i++) {
        obstacle_map_.data.push_back(100);
    }
}

int ObstacleMapCreator::xy_to_map_index(double x, double y) {
    int x_index = (x - obstacle_map_.info.origin.position.x) / map_gridSize_;
    int y_index = (y - obstacle_map_.info.origin.position.y) / map_gridSize_;

    return x_index + y_index * obstacle_map_.info.width;
}

//受け取ったx,yが、あらかじめ決めたobstacle_mapのサイズにおさまっているか判定
bool ObstacleMapCreator::is_within_obstacle_map(double x, double y) {
    double x_min = obstacle_map_.info.origin.position.x;
    double y_min = obstacle_map_.info.origin.position.y;
    double x_max = x_min + obstacle_map_.info.width * obstacle_map_.info.resolution;
    double y_max = y_min + obstacle_map_.info.height * obstacle_map_.info.resolution;

    if ((x_min < x) && (x_max > x) && (y_min < y) && (y_max > y)) {
        return true;
    } else {
        return false;
    }
}


//受け取った角度が、車体の柱の部分かどうか判定
bool ObstacleMapCreator::is_ignore_angle(double angle) {
    angle = abs(angle);

    if ((angle > M_PI * 2 / 16) && (angle < M_PI * 5 / 16))  //柱の位置
    {
        return false;
    } else if (angle > M_PI * 11 / 16)                       //柱の位置
    {
        return false;
    } else                                                //柱の位置ではない
    {
        return true;
    }
}

void ObstacleMapCreator::add_obstacles_to_map(double angle, double laser_range) {
    if (!is_ignore_angle(angle)) {
        laser_range = map_size_;
    }

    for (double distance = 0; distance < map_size_; distance += map_gridSize_) {
        double x_now = distance * std::cos(angle);
        double y_now = distance * std::sin(angle);

        if (!is_within_obstacle_map(x_now, y_now)) {
            return;
        }

        //ROS_INFO("ang %f dist %f", angle, distance);

        int map_index = xy_to_map_index(x_now, y_now);

        if (second_odometry_got_) { //second_odometry_got_ == true => updated_obstacle_poses is not empty
            for (const auto &upd_ob_pose: updated_obstacle_poses_.poses) {
                double upd_ob_x_now = upd_ob_pose.position.x;
                double upd_ob_y_now = upd_ob_pose.position.y;
                int upd_ob_map_index = xy_to_map_index(upd_ob_x_now, upd_ob_y_now);

                if (map_index == upd_ob_map_index) {
                    laser_range = hypot(upd_ob_x_now, upd_ob_y_now);
                }
            }
        }

        if (distance >= laser_range) {
            obstacle_map_.data[map_index] = 100;

            obstacle_pose_.position.x = x_now;
            obstacle_pose_.position.y = y_now;
            obstacle_poses_.poses.push_back(obstacle_pose_);
            //return;
        } else {
            obstacle_map_.data[map_index] = 0;
        }
    }
}

void ObstacleMapCreator::update_obstacle_poses() {
    for (const auto &ob_pose: obstacle_poses_.poses) {
        double x = ob_pose.position.x - diff_.position.x;
        double y = ob_pose.position.y - diff_.position.y;


        double current_yaw = tf2::getYaw(current_odo_.pose.pose.orientation);
        double previous_yaw = tf2::getYaw(previous_odo_.pose.pose.orientation);
        double theta = previous_yaw - current_yaw;

        updated_obstacle_pose_.position.x = x * cos(theta) - y * sin(theta);
        updated_obstacle_pose_.position.y = x * sin(theta) + y * cos(theta);

        double angle = atan2(updated_obstacle_pose_.position.y, updated_obstacle_pose_.position.x);

        if (!is_ignore_angle(angle)) {
            updated_obstacle_poses_.poses.push_back(updated_obstacle_pose_);
        }
    }
}

void ObstacleMapCreator::create_obstacle_map() {
    double angle_size = laser_scan_.angle_max - laser_scan_.angle_min;
    int angle_step = int((laser_scan_.ranges.size() * M_PI) / angle_size / 180 / 2);

    obstacle_poses_.poses.clear();
    for (int i = 0; i < int(laser_scan_.ranges.size()); i += angle_step) {
        double angle = i * laser_scan_.angle_increment + laser_scan_.angle_min;
        add_obstacles_to_map(angle, laser_scan_.ranges[i]);
    }
    updated_obstacle_poses_.poses.clear();
}

void ObstacleMapCreator::process() {
    ros::Rate loop_rate(hz_);

    //int i = 0;

    while (ros::ok()) {
        if (first_scan_done_) {
            obstacle_map_pub_.publish(obstacle_map_);
            //ROS_INFO("pub");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_map_creator");
    ObstacleMapCreator obstacleMapCreator;
    obstacleMapCreator.process();

    return 0;
}
