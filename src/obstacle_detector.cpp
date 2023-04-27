#include "obstacle_detector/obstacle_detector.h"

//chibi22_bのソースコードを参考に作成
ObstacleMapCreator::ObstacleMapCreator() : private_nh_("~") {
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("map_size", map_size_, {6}); //周囲の障害物情報を保持するマップ（obstacle_map_）の大きさ
    private_nh_.param("map_gridSize", map_gridSize_, {0.02}); //obstacle_map_の1マスの大きさ

    odo_sub_ = nh_.subscribe("/roomba/odometry", 100, &ObstacleMapCreator::odo_callback, this);
    laser_sub_ = nh_.subscribe("scan", 10, &ObstacleMapCreator::laser_callback, this);
    obstacle_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("obstacle_map", 10);
//    pub_obs_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/local_map/obstacle", 1);
    pub_obs_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/local_map/obstacle", 1);


    obstacle_poses_.header.frame_id = "base_link";
    obstacle_map_.header.frame_id = "base_link";
    obstacle_map_.info.resolution = map_gridSize_;
    obstacle_map_.info.width = map_size_ / map_gridSize_;
    obstacle_map_.info.height = map_size_ / map_gridSize_;
    obstacle_map_.info.origin.position.x = -map_size_ / 2;
    obstacle_map_.info.origin.position.y = -map_size_ / 2;
    obstacle_map_.data.reserve(obstacle_map_.info.width * obstacle_map_.info.height);
}

//LiDARのスキャンデータからobstacle_map_を作成
void ObstacleMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser_scan_ = *msg;

    if (!is_map_initialized_) { //初回スキャン時のみobstacle_map_を初期化
        init_map();
        is_map_initialized_ = true;
    }

    //初回のスキャンが完了し、/roomba/odometryの購読が2回以上行われた時、obstacle_map_を更新
    if (first_scan_done_ && second_odometry_got_) update_obstacle_poses();

    create_obstacle_map();
    first_scan_done_ = true;
}

//roombaのオドメトリデータを取得し、移動量を計算
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

//obstacle_map_全領域の占有状態を不明とする
void ObstacleMapCreator::init_map() {
    obstacle_map_.data.clear();
    int size = obstacle_map_.info.width * obstacle_map_.info.height;
    for (int i = 0; i < size; i++) {
        obstacle_map_.data.push_back(100);
    }
}

//roombaを中心としたxy座標をobstace_map_.dataのインデックスに変換
int ObstacleMapCreator::xy_to_map_index(double x, double y) {
    int x_index = (x - obstacle_map_.info.origin.position.x) / map_gridSize_;
    int y_index = (y - obstacle_map_.info.origin.position.y) / map_gridSize_;

    return x_index + y_index * obstacle_map_.info.width;
}

//受け取ったx,yが、あらかじめ決めたobstacle_map_のサイズにおさまっているか判定
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

//    if ((angle > M_PI * 14 / 64) && (angle < M_PI * 18 / 64))  //柱の位置
//    {
//        return false;
//    } else if (angle > M_PI * 45 / 64)                       //柱の位置
//    {
//        return false;
//    } else                                                //柱の位置ではない
//    {
//        return true;
//    }

    if ((angle > M_PI * 12 / 64) && (angle < M_PI * 20 / 64))  //柱の位置
    {
        return false;
    } else if (angle > M_PI * 43 / 64)                       //柱の位置
    {
        return false;
    } else                                                //柱の位置ではない
    {
        return true;
    }
}

//スキャンによる障害物情報をobstacle_map_に反映する
void ObstacleMapCreator::add_obstacles_to_map(double angle, double laser_range) {
    is_nearest_obstacle_record_ = false; //障害物記録判定リセット

    if (!is_ignore_angle(angle)) { //angleが車体の柱の部分の場合、laser_rangeの値を無視して柱の影がobstacle_map_に映りこまないようにする
        laser_range = map_size_;
    }

    for (double distance = 0; distance < map_size_; distance += map_gridSize_) {
        double x_now = distance * std::cos(angle);
        double y_now = distance * std::sin(angle);

        if (!is_within_obstacle_map(x_now, y_now)) {
            return;
        }

        int map_index = xy_to_map_index(x_now, y_now);

        if (second_odometry_got_) { //second_odometry_got_がtrueの時、updated_obstacle_posesは空ではない
            for (const auto &upd_ob_pose: updated_obstacle_poses_.poses) {//柱によって隠れている障害物を見落とさないよう、前回のスキャンで作成した障害物の位置情報リストを参照する
                double upd_ob_x_now = upd_ob_pose.position.x;
                double upd_ob_y_now = upd_ob_pose.position.y;
                int upd_ob_map_index = xy_to_map_index(upd_ob_x_now, upd_ob_y_now);

                if (map_index == upd_ob_map_index) {
                    laser_range = hypot(upd_ob_x_now, upd_ob_y_now);
                }
            }
        }

        if (distance >= laser_range) { //obstacle_map_において、roombaからの距離が障害物までの距離（laser_range[i]）を超える領域には障害物があるものとする
            obstacle_map_.data[map_index] = 100;

            if (!is_nearest_obstacle_record_) { //それぞれの角度において、最も近い障害物のみ記録する
                obstacle_pose_.position.x = x_now;
                obstacle_pose_.position.y = y_now;
                obstacle_poses_.poses.push_back(obstacle_pose_);
                is_nearest_obstacle_record_ = true;
            }
        } else {
            obstacle_map_.data[map_index] = 0;
        }
    }
}

//roombaの移動量・旋回量を障害物の位置情報に反映する
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

//LiDARによるスキャンデータから、obstacle_map_を作成する
void ObstacleMapCreator::create_obstacle_map() {
    double angle_size = laser_scan_.angle_max - laser_scan_.angle_min;
    int angle_step = int((laser_scan_.ranges.size() * M_PI) / angle_size / 180 / 2);

    obstacle_poses_.poses.clear();
    for (int i = 0; i < int(laser_scan_.ranges.size()); i += angle_step) { //それぞれ角度の障害物までの距離をobstacle_map_に反映する
        double angle = i * laser_scan_.angle_increment + laser_scan_.angle_min;
        double laser_range = laser_scan_.ranges[i];
        if (laser_range < 0.015) continue;
        add_obstacles_to_map(angle, laser_range);
    }
    updated_obstacle_poses_.poses.clear();
}

void ObstacleMapCreator::process() {
    ros::Rate loop_rate(hz_);

    while (ros::ok()) {
        if (first_scan_done_) {
            obstacle_map_pub_.publish(obstacle_map_);
            pub_obs_poses_.publish(obstacle_poses_);
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
