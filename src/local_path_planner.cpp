/*
 * velocity(vel)  ：並進速度
 * yawrate        ：旋回速度
 */

#include "local_path_planner/local_path_planner.h"

DWA::DWA():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("dt", dt_, {0.1});
    private_nh_.param("goal_tolerance", goal_tolerance_, {0.1});
    private_nh_.param("max_vel", max_vel_, {0.2});
    private_nh_.param("min_vel", min_vel_, {0.0});
    private_nh_.param("max_yawrate", max_yawrate_, {0.8});
    private_nh_.param("max_yawrate", max_accel_, {1.0});
    private_nh_.param("max_yawaccel", max_yawaccel_, {1.0});
    private_nh_.param("predict_time", predict_time_, {1.0});
    private_nh_.param("weight_heading", weight_heading_, {1.0});
    private_nh_.param("weight_distance", weight_distance_, {1.0});
    private_nh_.param("weight_velocity", weight_velocity_, {1.0});
    private_nh_.param("search_range", search_range_, {5.0});
    private_nh_.param("roomba_radius", roomba_radius_, {0.3});
    private_nh_.param("radius_margin", radius_margin_, {0.2});
    private_nh_.param("vel_step", vel_step_, {0.01});
    private_nh_.param("yawrate_step", yawrate_step_, {0.01});
    private_nh_.param("visualize_check", visualize_check_, {true});
   // odom_sub_ = nh_.subscribe("/roomba/odometry", 1, &DWA::odometry_callback, this);
    //laser_sub_ = nh_.subscribe("/scan", 1, &DWA::laser_callback, this);

    //Subscriber
    sub_waypoints_ = nh_.subscribe("/local_goal", 1, &DWA::waypoints_callback, this);  //"/waypoints"のところは名前変える必要あるかも？
    sub_ob_position_ = nh_.subscribe("/local_map/obstacle", 1, &DWA::ob_position_callback, this);  //"/local_map/obstacle"のところは名前変える必要あるかも？

    //Publisher
    pub_cmd_vel_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
    pub_predict_path_ = nh_.advertise<nav_msgs::Path>("/predict_paths", 1);
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/optimal_local_path", 1);
}

//waypointsのコールバック関数
void DWA::waypoints_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    geometry_msgs::TransformStamped transform;
    // ROS_INFO("catch waypoints_data");  //デバック用

    //waypoints_の座標系をbase_linkに合わせる
    try
    {
        transform = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));
        flag_waypoints_ = true;
    }
    catch(tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        flag_waypoints_ = false;
        // ROS_INFO("No waypoints_data");  //デバック用
        return;
    }

    tf2::doTransform(*msg, waypoints_, transform);

}

//ob_position_のコールバック関数
void DWA::ob_position_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    ob_position_ = *msg;
    // ROS_INFO("catch ob_position_data");  //デバック用
    flag_ob_position_ = true;
}

//途中のゴール(waypoints)に着くまでtrueを返す
bool DWA::goal_check()
{
    //msg受信済みか確認
    if((flag_ob_position_ || flag_waypoints_) == false)
    {
        // ROS_INFO("Data catch false");  //デバック用
        return false;
    }

    // ROS_INFO("Data catch success!");  //デバック用

    double dx = waypoints_.point.x - roomba_.x;
    double dy = waypoints_.point.y - roomba_.y;
    double dist_to_goal = hypot(dx, dy);

    // ROS_INFO("calc dist_to_goal");  //デバック用

    if(dist_to_goal > goal_tolerance_)
        return true;
    else
        return false;
}

//Dynamic Windowの計算
void DWA::calc_dynamic_window()
{
    //制御可能範囲(Vs)
    double Vs[] = {min_vel_, max_vel_, -max_yawrate_, max_yawrate_};

    //動的制御可能範囲(Vr)
    double Vr[] = {roomba_.velocity - max_accel_ * dt_,
                   roomba_.velocity + max_accel_ * dt_,
                   roomba_.yawrate - max_yawaccel_ * dt_,
                   roomba_.yawrate + max_yawaccel_ * dt_};

    //Dynamic Window
    dw_.min_vel = std::max(Vs[0], Vr[0]);
    dw_.max_vel = std::min(Vs[1], Vr[1]);
    dw_.min_yawrate = std::max(Vs[2], Vr[2]);
    dw_.max_yawrate = std::min(Vs[3], Vr[3]);
}

//仮想ロボットを移動
void DWA::move_image(State& imstate, double velocity, double yawrate)
{
    imstate.yaw += yawrate * dt_;
    imstate.x += velocity * dt_ * cos(imstate.yaw);
    imstate.y += velocity * dt_ * sin(imstate.yaw);
    imstate.velocity = velocity;
    imstate.yawrate = yawrate;
}

//予測軌道を作成
std::vector<State> DWA::predict_trajectory(double velocity, double yawrate)
{
    std::vector<State> traj;  //軌道格納用の動的配列(サイズが可変)
    State imstate = {0.0, 0.0, 0.0, 0.0, 0.0};  //仮想ロボット

    //軌道を格納
    for(double t=0.0; t<=predict_time_; t+=dt_)
    {
        move_image(imstate, velocity, yawrate);
        traj.push_back(imstate);  //trajの末尾に挿入
    }

    return traj;
}

//評価関数を計算する
double DWA::calc_evaluation(std::vector<State>& traj)
{
    double heading_value = weight_heading_ * calc_heading_eval(traj);
    // ROS_INFO("calc heading_value success!");  //デバック用
    double distance_value = weight_distance_ * calc_distance_eval(traj);
    // ROS_INFO("calc distance_value success!");  //デバック用
    double velocity_value = weight_velocity_ * calc_velocity_eval(traj);
    // ROS_INFO("calc velocity_value success!");  //デバック用

    double total_score = heading_value + distance_value + velocity_value;
    // ROS_INFO("calc total_score success!");  //デバック用

    return total_score;
}

//heading(1項目)の評価関数を計算する
double DWA::calc_heading_eval(std::vector<State>& traj)
{
    //最終時刻でのロボットの方向
    double theta = traj.back().yaw;

    //最終時刻での位置に対するゴール方向
    double goal_theta = atan2(waypoints_.point.y - traj.back().y, waypoints_.point.x - traj.back().x);

    //ゴールまでの方位差分
    double target_theta = 0.0;  //初期化
    if(goal_theta > theta)
        target_theta = goal_theta - theta;
    else if(goal_theta < theta)
        target_theta = theta -goal_theta;

    //headingの評価値を計算
    double heading_eval = (M_PI - abs(optimize_angle(target_theta))) / M_PI;  //正規化

    return heading_eval;
}

//distance(2項目)の評価関数を計算する
double DWA::calc_distance_eval(std::vector<State>& traj)
{
    double dist_to_ob = 0.0;  //障害物までの距離
    //roombaの軌道上に障害物がないか探索
    for(auto& state : traj)
    {
        for(auto& obstacle : ob_position_.poses)  //ob_position_の部分は名前を直す必要があるかも
        {
            //障害物までの距離を計算
            double dx = obstacle.position.x - state.x;
            double dy = obstacle.position.y - state.y;
            dist_to_ob = hypot(dx, dy);

            //distanceの評価値を計算
            if(dist_to_ob <= roomba_radius_ + radius_margin_)
                return -100.0;  //壁に衝突したパスにはマイナス値を返す
        }
    }

    return dist_to_ob / search_range_;  //正規化
}

//velocity(3項目)の評価関数を計算する
double DWA::calc_velocity_eval(std::vector<State>& traj)
{
    if(traj.back().velocity > 0.0)  //前進
        return traj.back().velocity / max_vel_;  //正規化
    else                            //後退
        return 0.0;
}

//適切な角度(-M_PI~M_PI)に変換
double DWA::optimize_angle(double theta)
{
    if(theta > M_PI)
        theta -= 2.0 * M_PI;
    if(theta < -M_PI)
        theta += 2.0 * M_PI;

    return theta;
}

//最適な制御入力を計算
std::vector<double> DWA::calc_input()
{
    //input[0] = velocity, input[1] = yawrate;
    std::vector<double> input{0.0, 0.0};
    dt_ = 1.0 / hz_;
    // ROS_INFO("calc hz_");  //デバック用

    //ダイナミックウィンドウを計算
    calc_dynamic_window();
    // ROS_INFO("calc dynamic_window success!");  //デバック用

    double one_score;  //計算したスコア格納用
    std::vector<double> score_yawrate;  //計算したスコア一次保存用
    std::vector< std::vector<double> > scores;  //すべての制御入力に対する評価値格納用
    std::vector< std::vector<State> > trajectories;  //すべての軌跡格納用

    int i = 0;  //カウンタ変数
    int j = 0;  //カウンタ変数

    int vel_size = 0;  //velocityの分割個数
    int yawrate_size = 0;  //yawrateの分割個数

    //並進速度と旋回速度のすべての組み合わせを評価
    for(double velocity=dw_.min_vel; velocity<=dw_.max_vel; velocity+=vel_step_)
    {
        for(double yawrate=dw_.min_yawrate; yawrate<=dw_.max_yawrate; yawrate+=yawrate_step_)
        {
            std::vector<State> traj = predict_trajectory(velocity, yawrate);  //予測軌道を生成
            // ROS_INFO("create predict_trajectory success!");  //デバック用
                                                             //
            one_score = calc_evaluation(traj);  //予測軌道に評価関数を適用
            score_yawrate.push_back(one_score);
            // ROS_INFO("velocity: %lf", velocity);  //デバック用
            // ROS_INFO("yawrate : %lf", yawrate);  //デバック用
            // ROS_INFO("score   : %lf", one_score);  //デバック用
            trajectories.push_back(traj);

            j++;
        }

        scores.push_back(score_yawrate);

        //yawrateの分割個数を格納
        if(i == 0)
        {
            yawrate_size = j;
            ROS_INFO("yawrate_size = %d", yawrate_size);  //デバック用
        }

        i++;
    }

    // ROS_INFO("calc all score finish! ");  //デバック用

    //velocityの分割個数を格納
    vel_size = i;
    ROS_INFO("vel_size = %d", vel_size);  //デバック用

    //評価値に対してスムージング関数を適用
    double smoothing_score;  //スムージング関数適用後評価値格納用
    // std::vector< std::vector<double> > smoothing_score;  //スムージング関数適用後評価値格納用
    // smoothing_score = scores;

    double score_sum;  //隣接する評価値との合計値を格納
    int k = 0;  //カウンタ変数
    // int l = 0;  //カウンタ変数

    double max_score = -1000.0;  //評価値の最大値格納用
    int max_vel_score_index = 0;  //評価値が最大となる速度のインデックス格納用
    int max_yawrate_score_index = 0;  //評価値が最大となる旋回速度のインデックス格納用
    int max_score_index = 0;  //評価値が最大のときのインデックス格納用

    for(i=1; i<vel_size; i++)  //隣接するデータ数が減ってしまう端のデータは使わない
    {
        for(j=1; j<yawrate_size; j++)
        {
            score_sum = 0.0;

            for(int m=i-1; m<=i+1; m++)
            {
                for(int n=j-1; n<=j+1; n++)
                {
                    score_sum += scores[m][n];
                    ROS_INFO("calc score_sum");  //デバック用
                }
            }

            smoothing_score = score_sum / 9.0;
            // ROS_INFO("k = %d, l = %d, smoothing_score = %lf", k, l, smoothing_score[k][l]);  //デバック用

            //評価値が一番大きいデータの探索
            if(max_score < smoothing_score)
            {
                max_score = smoothing_score;
                max_vel_score_index = i;
                max_yawrate_score_index = j;
                max_vel_score_index = k;
            }

            k++;
        }
    }

    // ROS_INFO("get smoothing_score!");  //デバック用

    /*//評価値が一番大きいデータの探索
    int max_yawrate_score_index = 0;  //評価値が最大となる旋回速度のインデックス格納用

    for(i=0; i<=vel_size-2; i++)
    {
        for(j=0; j<=yawrate_size-2; j++)
        {
            //最大値の更新
            if(max_score < smoothing_score[i][j])
            {
                max_score = smoothing_score[i][j];
                max_vel_score_index = i;
                max_yawrate_score_index = j;
            }
        }
    }*/

    ROS_INFO("max_score: %lf", max_score);  //デバック用

    //最適な制御入力を格納
    ROS_INFO("max_vel_score_index: %d", max_vel_score_index);  //デバック用
    ROS_INFO("max_yawrate_score_index: %d", max_yawrate_score_index);  //デバック用

    input[0] = dw_.min_vel + vel_step_ * (max_vel_score_index + 1);
    input[1] = dw_.min_yawrate + yawrate_step_ * (max_yawrate_score_index + 1);

    ROS_INFO("roomba_.velocity: %lf", input[0]);  //デバック用
    ROS_INFO("roomba_.yawrate : %lf", input[1]);  //デバック用

    //現在速度の記録
    roomba_.velocity = input[0];
    roomba_.yawrate = input[1];

    //パスを可視化して適切なパスが選択できているかを評価
    if(visualize_check_ = true)
    {
        ros::Time now = ros::Time::now();

        for(i=0; i<trajectories.size(); i++)
        {
            if(i == max_score_index)
            {
                visualize_traj(trajectories[i], pub_optimal_path_, now);
                ROS_INFO("This is optimal_path!");  //デバック用
            }
            else
                visualize_traj(trajectories[i], pub_predict_path_, now);

            ROS_INFO("visualize_traj success!");  //デバック用
        }
    }
    /*if(visualize_check_ = true)
    {
        ros::Time now = ros::Time::now();

        for(i=0; i<=vel_size-2; i++)
        {
            for(j=0; j<=yawrate_size-2; j++)
            {
                if((i ==  max_vel_score_index) && (j == max_yawrate_score_index))
                    visualize_traj(trajectories[i*j+j], pub_optimal_path_, now);
                else
                    visualize_traj(trajectories[i*j+j], pub_predict_path_, now);
            }
        }

        ROS_INFO("visualize_traj success!");  //デバック用
    }*/

    return input;
}

//軌道の可視化
void DWA::visualize_traj(std::vector<State>& traj, const ros::Publisher& pub_local_path, ros::Time now)
{
    nav_msgs::Path local_path;
    local_path.header.stamp = now;
    local_path.header.frame_id = "base_link";

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "base_link";

    //すべての軌道を格納
    for(auto& state : traj)
    {
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        local_path.poses.push_back(pose);
    }

    pub_local_path.publish(local_path);
}

//roombaの制御入力
void DWA::roomba_control(double velocity, double yawrate)
{
    cmd_vel_.mode = 11;  //mode11:速度と角速度を設定
    cmd_vel_.cntl.linear.x = velocity;
    cmd_vel_.cntl.angular.z = yawrate;

    pub_cmd_vel_.publish(cmd_vel_);
}

//メイン文で実行する関数
void DWA::process()
{
    ros::Rate loop_rate(hz_);
    tf2_ros::TransformListener tf_listener(tf_buffer_);  //waypoints_の情報を取得

    while(ros::ok())
    {
        if(goal_check())
        {
            ROS_INFO("calc_input start");  //デバック用
            std::vector<double> input = calc_input();
            roomba_control(input[0], input[1]);
            ROS_INFO("yattane!");  //デバック用
        }
        else
        {
            roomba_control(0.0, 0.0);
            ROS_INFO("Can't move!");  //デバック用
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

//メイン関数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_path_planner");
    DWA dwa;
    dwa.process();
    return 0;
}
