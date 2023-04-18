#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("dist_to_update_local_goal", dist_to_update_local_goal_, {1.5});
    private_nh_.param("global_path_index", global_path_index_, {50});
    private_nh_.param("index_step", index_step_, {10});

    //Subscriber
    sub_global_path_ = nh_.subscribe("/global_path", 1, &LocalGoalCreator::global_path_callback, this);
    sub_estimated_pose_ = nh_.subscribe("/estimated_pose", 1, &LocalGoalCreator::estimated_pose_callback, this);

    //Publisher
    pub_local_goal_ = nh_.advertise<geometry_msgs::PointStamped>("local_goal", 1);
}

//global_pathのコールバック関数
void LocalGoalCreator::global_path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    global_path_ = *msg;
    flag_global_path = true;
}

//estimated_poseのコールバック関数
void LocalGoalCreator::estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    estimated_pose_ = *msg;
}

//local_goalの更新
void LocalGoalCreator::update_local_goal()
{
    double dist_to_local_goal = calc_dist_to_local_goal();

    while(dist_to_local_goal < dist_to_update_local_goal_)
    {
        global_path_index_ += index_step_;  //パスのインデックスを進める
        dist_to_local_goal = calc_dist_to_local_goal();

        //global_path_indexがglobal_path_.posesの配列の要素数を超えたら、local_goalとしてglobal_pathのゴールを設定
        if(global_path_index_ >= global_path_.poses.size())
        {
            global_path_index_ = global_path_.poses.size() - 1;
            break;
        }
    }

    //local_goalを更新してpublish
    local_goal_.header.stamp = ros::Time::now();
    local_goal_.header.frame_id = "map";

    local_goal_.point.x = global_path_.poses[global_path_index_].pose.position.x;
    local_goal_.point.y = global_path_.poses[global_path_index_].pose.position.y;

    pub_local_goal_.publish(local_goal_);
}

//現在の位置とlocal_goalまでの距離を計算
double LocalGoalCreator::calc_dist_to_local_goal()
{
    double dx = global_path_.poses[global_path_index_].pose.position.x - estimated_pose_.pose.position.x;
    double dy = global_path_.poses[global_path_index_].pose.position.y - estimated_pose_.pose.position.y;

    return hypot(dx, dy);
}

//メイン文で実行する関数
void LocalGoalCreator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        //ローカルゴールの更新
        if(flag_global_path == true)
            update_local_goal();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

//メイン関数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_goal_creator");
    LocalGoalCreator local_goal_creator;
    local_goal_creator.process();
    return 0;
}
