#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

class LocalGoalCreator
{
    public:
        LocalGoalCreator();
        void process();

    private:
        //コールバック関数
        void global_path_callback(const nav_msgs::Path::ConstPtr& msg);
        void estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        //引数なし関数
        void update_local_goal();  //local_goalの更新
        double calc_dist_to_local_goal();  //現在の位置とlocal_goalまでの距離を計算

        //yamlファイルで設定可能な変数
        int hz_;        //ループ周波数[Hz]
        double dist_to_update_local_goal_;  //local_goalを更新する基準となるゴールまでの距離
        int global_path_index_;  //global_path_plannerからpublishされるパスのインデックス
        int index_step_;  //1回で更新するインデックス数

        //msgの受け取り判定用
        bool flag_global_path = false;

        //NodeHandle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        //Subscriber
        ros::Subscriber sub_global_path_;
        ros::Subscriber sub_estimated_pose_;

        //Publisher
        ros::Publisher pub_local_goal_;

        nav_msgs::Path global_path_;
        geometry_msgs::PoseStamped estimated_pose_;
        geometry_msgs::PointStamped local_goal_;
};

#endif
