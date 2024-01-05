#ifndef LOCAL_PATH_PLANNER_H
#define LOCAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>


//データをまとめて扱えるように構造体にする
struct State
{
    double x;  //x座標[m]
    double y;  //y座標[m]
    double yaw;  //角度[rad]
    double velocity;  //並進速度[m/s]
    double yawrate;  //旋回速度[m/s]
};

struct Dynamic_Window
{
    double min_vel;
    double max_vel;
    double min_yawrate;
    double max_yawrate;
};

class DWA
{
    public:
        DWA();
        void process();


    private:
        //コールバック関数
        void local_goal_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void ob_position_callback(const geometry_msgs::PoseArray::ConstPtr& msg);

        //引数あり関数
        void move_image(State& imstate, double velocity, double yawrate);  //仮想ロボットを移動
        std::vector<State> predict_trajectory(double velocity, double yawrate);  //予測軌道を作成
        double calc_evaluation(std::vector<State>& traj, double yawrate);  //評価関数を計算する
        double calc_heading_eval(std::vector<State>& traj);  //heading(1項目)の評価関数を計算する
        double calc_distance_eval(std::vector<State>& traj);  //distance(2項目)の評価関数を計算する
        double calc_velocity_eval(std::vector<State>& traj);  //velocity(3項目)の評価関数を計算する
        double calc_yawrate_eval(double yawrate);  //左右交互にパスを選択しないようにする(評価関数4項目)
        double optimize_angle(double theta);  //適切な角度(-M_PI~M_PI)に変換
        void visualize_traj(std::vector<State>& traj, const ros::Publisher& pub_local_path, ros::Time now);  //軌道を可視化
        void roomba_control(double velocity, double yawrate);  //roombaの制御入力

        //引数なし関数
        bool goal_check();  //途中のゴール(waypoints)に着くまでtrueを返す
        void calc_dynamic_window();  //Dynamic Windowの計算
        std::vector<double> calc_input();  //最適な制御入力を計算

        //yamlファイルで設定可能な変数
        int hz_;        //ループ周波数[Hz]
        double dt_;       //微小時間[s]
        double goal_tolerance_;  //local_goal_に対する許容誤差[m]
        double min_vel_;     //最低並進速度[m/s]
        double max_vel_;     //最高並進速度[m/s]
        double max_yawrate_;  //最高旋回速度[rad/s]
        double max_accel_;    //最高並進加速度[m/s^2]
        double max_yawaccel_;  //最高旋回加速度[rad/s^2]
        double predict_time_;  //軌道予測時間[s]
        double weight_heading_;  //評価関数1項目　重みづけ定数
        double weight_distance_;  //評価関数2項目　重みづけ定数
        double weight_velocity_;  //評価関数3項目　重みづけ定数
        double weight_yawrate_;  //評価関数4項目　重みづけ定数
        double search_range_;  //評価関数２項目(distance)探索範囲[m]
        double roomba_radius_;  //ルンバの半径[m]
        double radius_margin_;  //衝突半径の余白[m]
        double vel_step_;  //最適な並進速度を計算するときの刻み幅[m/s]
        double yawrate_step_;  //最適な旋回速度を計算するときの刻み幅[rad/s]
        bool visualize_check_;  //パスを可視化するかどうかの設定用
        bool smoothing_check_;  //スムージング関数を適用するかどうかの設定用
        int stop_counter_;  //ルンバの出力値が(0,0)になっているループ数カウント用
        int stop_time_;  //ルンバの出力値が(0,0)になってから動き出すまでのループ数
        int move_time_;  //ルンバの出力値が(0,0)を選択できないループ数

        //msgの受け取り判定用
        bool flag_local_goal_ = false;
        bool flag_ob_position_ = false;

        //NodeHandle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        //構造体
        State roomba_;
        Dynamic_Window dw_;

        //Subscriber
        ros::Subscriber sub_local_goal_;
        ros::Subscriber sub_ob_position_;

        //Publisher
        ros::Publisher pub_cmd_vel_;
        ros::Publisher pub_predict_path_;
        ros::Publisher pub_optimal_path_;

        geometry_msgs::PointStamped local_goal_;
        geometry_msgs::PoseArray ob_position_;

        //tf
        tf2_ros::Buffer tf_buffer_;

        //制御入力
        roomba_500driver_meiji::RoombaCtrl cmd_vel_;
};

#endif
