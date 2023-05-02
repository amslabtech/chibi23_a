#include "localizer.h"
//2
std::random_device seed_gen;
std::mt19937 engine(seed_gen());

Particle::Particle(double x, double y, double yaw, double weight)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    weight_ = weight;
}

void Particle::set_pose(double x, double y, double yaw)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
}

void Particle::set_weight(double weight)
{
    weight_ = weight;
}

Localizer::Localizer():private_nh_("~")
{
    private_nh_.getParam("hz",hz_);
    private_nh_.getParam("num",num_);
    private_nh_.getParam("init_x",init_x_);
    private_nh_.getParam("init_y",init_y_);
    private_nh_.getParam("init_yaw",init_yaw_);
    private_nh_.getParam("init_dev",init_dev_);

    // private_nh_.param("intercept", intercept, {0.080});

    private_nh_.getParam("laser_dev_per_dist", laser_dev_per_dist_);
    private_nh_.getParam("laser_step", laser_step_);
    private_nh_.getParam("laser_ignore_range", laser_ignore_range_);

    private_nh_.getParam("alpha_th", alpha_th_);
    private_nh_.getParam("alpha_th_expansion", alpha_th_expansion_);
    private_nh_.getParam("alpha_slow_th", alpha_slow_th_);
    private_nh_.getParam("alpha_fast_th", alpha_fast_th_);

    private_nh_.getParam("expansion_limit", expansion_limit_);
    private_nh_.getParam("expansion_reset_dev", expansion_reset_dev_);

    sub_laser_ = nh_.subscribe("/scan", 10, &Localizer::laser_callback, this);
    sub_odometry_ = nh_.subscribe("/roomba/odometry", 10, &Localizer::odometry_callback, this);
    sub_map_ = nh_.subscribe("/map", 10, &Localizer::map_callback, this);

    pub_estimated_pose = nh_.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
    pub_particle_cloud = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1);

}

// -- callback --
void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(!odometry_got_)
    {
        previous_odometry_ = *msg;
        odometry_got_ = true;
    }
    else
    {
        previous_odometry_ = current_odometry_;
    }

    if(current_odometry_ != previous_odometry_)
        can_move_ = true;

    current_odometry_ = *msg;
}

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_ = *msg;
    map_got_ = true;
    // ROS_INFO("%d", map_.info.width);
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
    laser_got_ = true;
    // ROS_INFO("%lf\n", laser_.ranges[540]);
}

// -- common_function --
double Localizer::set_noise(double mu, double dev)
{
    std::normal_distribution<> dist(mu, dev);
    return dist(engine);
}

double Localizer::optimize_angle(double angle) //ok
{
    if(angle > M_PI)
    {
        angle -= 2*M_PI;
    }

    if(angle < -M_PI)
    {
        angle += 2*M_PI;
    }

    return angle;
}

double Localizer::gaussian(double x, double mu, double dev)
{
    return exp(-pow(x - mu, 2.0) / (2.0 * pow(dev, 2.0))) / (sqrt(2.0 * M_PI) * dev);
}

double Localizer::liner(double x, double mu)
{
    double slope = -1 * intercept / mu;
    return slope * x + intercept;
}

// -- make particles --
void Localizer::initialize()
{
    for(int i=0; i<num_; i++)
    {
        double x = set_noise(init_x_, init_dev_);
        double y = set_noise(init_y_, init_dev_);
        double yaw = set_noise(init_yaw_, init_dev_);
        Particle p(x, y, yaw);
        particles_.push_back(p);
    }
}

void Localizer::publish_particles()
{
    particle_cloud_msg_.header.stamp = ros::Time::now();
    particle_cloud_msg_.header.frame_id = "map";
    particle_cloud_msg_.poses.resize(particles_.size());

    for(int i=0; i < particles_.size(); i++)
    {
        particle_cloud_msg_.poses[i].position.x = particles_[i].get_pose_x();
        particle_cloud_msg_.poses[i].position.y = particles_[i].get_pose_y();
        particle_cloud_msg_.poses[i].position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, particles_[i].get_pose_yaw());
        tf2::convert(q, particle_cloud_msg_.poses[i].orientation);
    }

    pub_particle_cloud.publish(particle_cloud_msg_);
}

void Localizer::publish_estimated_pose()
{
    estimated_pose_msg_.header.stamp = ros::Time::now();
    estimated_pose_msg_.header.frame_id = "map";

    estimated_pose_msg_.pose.position.x = estimated_pose_.get_pose_x();
    estimated_pose_msg_.pose.position.y = estimated_pose_.get_pose_y();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, estimated_pose_.get_pose_yaw());
    tf2::convert(q, estimated_pose_msg_.pose.orientation);

    pub_estimated_pose.publish(estimated_pose_msg_);
}

// -- move particles --
void Localizer::get_motion(Particle& p, double distance, double direction, double rotation)
{
    distance += set_noise(0.0, distance * 0.25);
    direction += set_noise(0.0, direction * 0.25);
    rotation += set_noise(0.0, rotation * 0.25);

    double x = p.get_pose_x() + distance * cos(optimize_angle(direction + p.get_pose_yaw()));
    double y = p.get_pose_y() + distance * sin(optimize_angle(direction + p.get_pose_yaw()));
    double yaw = optimize_angle(rotation + p.get_pose_yaw());

    /*if(get_map_data(x, y) == -1)
    {
        while(get_map_data(x, y) != -1)
        {
            x = set_noise(x, 0.25);
            y = set_noise(y, 0.25);
        }
    }*/

    p.set_pose(x, y, yaw);
}

void Localizer::move()
{
    double dx = current_odometry_.pose.pose.position.x - previous_odometry_.pose.pose.position.x;
    double dy = current_odometry_.pose.pose.position.y - previous_odometry_.pose.pose.position.y;
    double dyaw = optimize_angle(tf2::getYaw(current_odometry_.pose.pose.orientation) - tf2::getYaw(previous_odometry_.pose.pose.orientation));

    double distance = hypot(dx, dy);
    double direction = optimize_angle(atan2(dy, dx) - tf2::getYaw(previous_odometry_.pose.pose.orientation));

    for(auto& p : particles_)
    {
        get_motion(p, distance, direction, dyaw);
    }
}


// -- get_map_distance --
int Localizer::get_map_data(double x, double y)
{
    double resolution = map_.info.resolution;
    double width = map_.info.width;

    int mx = (int)floor((x - map_.info.origin.position.x ) / resolution);
    int my = (int)floor((y - map_.info.origin.position.y ) / resolution);

    int wall = map_.data[mx + my * width];

    return wall;
}
/*
double Localizer::dist_on_map(double map_x, double map_y, double laser_angle, double laser_length)
{
    double search_limit = laser_length/2;

    double x_dist = search_limit * cos(laser_angle);
    double y_dist = search_limit * sin(laser_angle);

    double x_now = map_x;
    double y_now = map_y;

    double distance = 0.0;

    map_x += x_dist;
    map_y += y_dist;

    int map_flag = get_map_data(map_x, map_y);

    if(map_flag != 100)
    {
        for(int i = 0; i <= 50; i++) //50は適当に決めた
        {
            map_flag = get_map_data(map_x, map_y);
            x_dist /= 2;
            y_dist /= 2;

            if(map_flag == 0)
            {
                map_x += x_dist;
                map_y += y_dist;
            }
            else if(map_flag == -1)
            {
                map_x -= x_dist;
                map_y -= y_dist;
            }
            else if(map_flag > 0) //50は適当に決めた
            {
                distance = hypot(map_x - x_now, map_y - y_now);
                return distance;
            }

        }
        //ROS_INFO("over_calculation_limit\n");
        return hypot(map_x - x_now, map_y - y_now);

    }else return search_limit;

}*/

double Localizer::dist_on_map(double map_x, double map_y, double laser_angle, double laser_length)
{
    double search_step = map_.info.resolution;
    double search_limit = laser_.range_max / 2.0;

    for(double distance=0.0; distance<= search_limit; distance += search_step)
    {
        map_x += search_step * cos(laser_angle);
        map_y += search_step * sin(laser_angle);

        int map_flag = get_map_data(map_x, map_y);

        if(map_flag != 0) return distance;
    }

    return search_limit;
}
/*
double Localizer::dist_on_map(double map_x, double map_y, double laser_angle, double laser_length)
{
    double search_step = map_.info.resolution;
    double search_limit = laser_.range_max / 2.0;
    double x_now = map_x;
    double y_now = map_y;
    double x_dist = search_step * cos(laser_angle);
    double y_dist = search_step * sin(laser_angle);
    double distance;
    int map_flag;

    for(distance=0.0; distance<= search_limit; distance += search_step)
    {
        map_x += x_dist;
        map_y += y_dist;

        map_flag = get_map_data(map_x, map_y);

        if(map_flag != 0) break;
    }

    if(map_flag != 1)
    {
        for(int i = 0; i <= 10; i++) //50は適当に決めた
        {
            map_flag = get_map_data(map_x, map_y);
            x_dist /= 2;
            y_dist /= 2;

            if(map_flag == 0)
            {
                map_x += x_dist;
                map_y += y_dist;
            }
            else if(map_flag == -1)
            {
                map_x -= x_dist;
                map_y -= y_dist;
            }
            else if(map_flag > 0) //50は適当に決めた
            {
                distance = hypot(map_x - x_now, map_y - y_now);
                return distance;
            }

        }
        //ROS_INFO("over_calculation_limit\n");
        return hypot(map_x - x_now, map_y - y_now);

    }else return search_limit;

}*/

// -- weight --
double Localizer::likelihood_model(double x, double mu, double dev)
{
    double ans = 0.0;
    /*if(x >= laser_.range_max / 2.0)
    {
        ans = 0.0075; //適当
    }else
    {
        ans = 0.005; //適当
    }

    if(ans < liner(x, mu)) ans = liner(x, mu);
    if(ans < gaussian(x, mu, dev))*/ ans = gaussian(x, mu, dev);

    return ans;
}

double Localizer::calc_weight(Particle& p)
{
    double weight = 0.0;
    double angle = optimize_angle(p.get_pose_yaw() + laser_.angle_min);
    double angle_step = laser_.angle_increment;
    int limit = laser_.ranges.size();


    if(get_map_data(p.get_pose_x(), p.get_pose_y()) == -1)
    {
        return 0;
    }
    else
    {
        for(int i = 0; i < limit; i += laser_step_)
        {
            if(laser_.ranges[i] > laser_ignore_range_)
            {
                double map_dist = 0.0;
                map_dist = dist_on_map(p.get_pose_x(), p.get_pose_y(), angle, laser_.ranges[i]);

                double sigma = laser_.ranges[i] * laser_dev_per_dist_;

                weight += likelihood_model(map_dist, laser_.ranges[i], sigma);
            }

        angle = optimize_angle(angle + angle_step * laser_step_);
        // ROS_INFO("laser=%lf, distance=%lf, weight=%lf\n",laser_.ranges[i], map_dist, likelihood_model(map_dist, laser_.ranges[i],sigma));
        }

    // ROS_INFO("%d\n", limit/laser_step_);
    // ROS_INFO("this particle's weight is %lf\n",weight);
    return weight;
    }
}

void Localizer::normalize_weight()
{
    alpha_ = 0.0;
    for(const auto &p : particles_)
    {
        alpha_ += p.get_weight();
    }

    // ROS_INFO("alpha_=%lf\n",alpha_);

    for(auto &p : particles_)
    {
        double new_weight = p.get_weight() / alpha_;
        p.set_weight(new_weight);
    }
}

//need to check
void Localizer::reset_weight()
{
    for(auto &p : particles_)
    {
        p.set_weight(1.0 / particles_.size());
    }
}

// -- measurement --
void Localizer::estimate_pose()
{
    int data = 0.0;

    max_weight = 0.0;

    /*for(int i=0; i < particles_.size(); i++)
    {
        if(particles_[i].get_weight() > max_weight)
        {
            max_weight = particles_[i].get_weight();
            data = i;
        }
    }

    Particle estimated_pose_(particles_[data].get_pose_x(), particles_[data].get_pose_y(), particles_[data].get_pose_yaw(), particles_[data].get_weight());*/

    double x=0.0;
    double y=0.0;
    double yaw=0.0;

    for(int i=0; i < particles_.size(); i++)
    {
        if(particles_[i].get_weight() > max_weight)
        {
            max_weight = particles_[i].get_weight();
            data = i;
        }
    }

    x = particles_[data].get_pose_x();
    y = particles_[data].get_pose_y();
    yaw = particles_[data].get_pose_yaw();

    /*for(const auto &p : particles_)
    {
        x += p.get_pose_x() * p.get_weight();
        y += p.get_pose_y() * p.get_weight();
        if(p.get_weight()==max_weight)
        {
            yaw=p.get_pose_yaw();
        }
    }*/

    estimated_pose_.set_pose(x, y, yaw);
    // ROS_INFO("x=%lf , y=%lf, weight=%lf \n", particles_[data].get_pose_x(), particles_[data].get_pose_y(), particles_[data].get_weight());
}

void Localizer::measurement()
{
    for(auto &p : particles_)
    {
        double new_weight = calc_weight(p);
        p.set_weight(new_weight);
    }

    normalize_weight();

    estimate_pose();
    // ROS_INFO("weight=%lf\n", max_weight);
}

// -- resampling --
/*//need to check
int Localizer::particle_number(int num, double previous_alpha_, double current_alpha_)
{
    int new_num_ = num_ * ( previous_alpha_ / current_alpha_);
    return new_num_;
}*/

void Localizer::sampling()
{
    double alpha_mean = alpha_ / (laser_.ranges.size() / laser_step_ * num_);

    // ROS_INFO("ok_1\n");
    if(alpha_mean < alpha_th_/*expansion_*/ && expansion_count_ < expansion_limit_ )
    {
        expansion_count_ ++;
        // ROS_INFO("ok\n");
        expansion_reset();
        // ROS_INFO("expansion_reset is ok\n");
    }
    else //if(alpha_mean < alpha_th_ /*|| max_weight < 0.40*/)
    {
        expansion_count_ = 0;
        resampling();
        // ROS_INFO("resampling is ok\n");
    }

   // ROS_INFO("alpha_mean = %lf\n", alpha_mean);
    // for(int i=0; i<particles_.size(); i++)
    // {
        // ROS_INFO("p[%d]\n", i);
        // ROS_INFO("x=%lf, y=%lf, weight=%lf\n", particles_[i].get_pose_x(), particles_[i].get_pose_y(), particles_[i].get_weight());
    // }
}

void Localizer::resampling()
{
    std::vector<Particle> new_particles;
    new_particles.reserve(num_);

    std::uniform_int_distribution<> int_dist(0, num_ - 1);
    std::uniform_real_distribution<> double_dist(0.0, max_weight*2.0);

    double sampling_width = 1 / alpha_;
    int index = int_dist(engine);

    alpha_slow_ += alpha_slow_th_ * (alpha_ - alpha_slow_);
    alpha_fast_ += alpha_fast_th_ * (alpha_ - alpha_fast_);

    replace_num_ = (int)(num_ * std::max(0.0, 1.0 - alpha_fast_/alpha_slow_));

    double r=0;

    /*for(int i=0; i<num_ - replace_num_; i++)
    {
        r += sampling_width;

        while(r > particles_[index].get_weight())
        {
            r -= particles_[index].get_weight();
            index = (index + 1) % num_;
        }

        new_particles.push_back(particles_[index]);
    }

    while(new_particles.size() < num_)
    {
        double x = set_noise(estimated_pose_.get_pose_x(), 0.25);
        double y = set_noise(estimated_pose_.get_pose_y(), 0.25);
        while(get_map_data(x, y) != -1)
        {
            x = set_noise(estimated_pose_.get_pose_x(), 0.25);
            y = set_noise(estimated_pose_.get_pose_y(), 0.25);
        }*/
        /*if(get_map_data(x,y) == -1)
        {
            while(get_map_data(x, y) != -1)
            {
                x = set_noise(estimated_pose_.get_pose_x(), 0.01);
                y = set_noise(estimated_pose_.get_pose_y(), 0.01);
            }
        }

        double yaw = set_noise(estimated_pose_.get_pose_yaw(), 0.25);
        Particle p(x, y, yaw);
        new_particles.push_back(p);
    }*/


    for(int i=0; i<num_; i++)
    {
        if(new_particles.size() < replace_num_)
        {
            double x = set_noise(estimated_pose_.get_pose_x(), 0.25);
            double y = set_noise(estimated_pose_.get_pose_y(), 0.25);
            double yaw = set_noise(estimated_pose_.get_pose_yaw(), 0.25);
            Particle p(x, y, yaw);
            new_particles.push_back(p);
        }
        else
        {
            r += double_dist(engine); //sampling_width;

            while(r > particles_[index].get_weight())
            {
                r -= particles_[index].get_weight();
                index = (index + 1) % num_;
            }

            new_particles.push_back(particles_[index]);
        }
    }

    // ROS_INFO("%d\n",laser_reset());
    particles_ = new_particles;
    reset_weight();
}
/*
int Localizer::laser_reset()
{
    alpha_ = alpha_;
    alpha_slow_ += alpha_slow_th_ * (alpha_ - alpha_slow_);
    alpha_fast_ += alpha_fast_th_ * (alpha_ - alpha_fast_);

    replace_num_ = (int) num_ * std::max(0.0, 1 - alpha_fast_ / alpha_slow_);
    return replace_num_;
}*/

void Localizer::expansion_reset()
{
    for(int i=0; i<num_; i++)
    {
        double x = set_noise(particles_[i].get_pose_x(), expansion_reset_dev_);
        double y = set_noise(particles_[i].get_pose_y(), expansion_reset_dev_);
        double yaw = set_noise(particles_[i].get_pose_yaw(), expansion_reset_dev_);

        /*if(get_map_data(x,y) == -1)
        {
            for(int i=0; i<3; i++)
            {
                x = set_noise(particles_[i].get_pose_x(), expansion_reset_dev_);
                y = set_noise(particles_[i].get_pose_y(), expansion_reset_dev_);
                yaw = set_noise(particles_[i].get_pose_yaw(), expansion_reset_dev_);
            }

            if(get_map_data(x, y) == -1)
            {
                x = set_noise(estimated_pose_.get_pose_x(), 0.01);
                y = set_noise(estimated_pose_.get_pose_y(), 0.01);
                yaw = set_noise(estimated_pose_.get_pose_yaw(), 0.01);
            }
        }*/

        particles_[i].set_pose(x, y, yaw);
    }

    reset_weight();
}

// -- broadcast --
void Localizer::broadcast_roomba_state()
{
    const double map_to_base_x = estimated_pose_.get_pose_x();
    const double map_to_base_y = estimated_pose_.get_pose_y();
    const double map_to_base_yaw = estimated_pose_.get_pose_yaw();

    const double odom_to_base_x = current_odometry_.pose.pose.position.x;
    const double odom_to_base_y = current_odometry_.pose.pose.position.y;
    const double odom_to_base_yaw = tf2::getYaw(current_odometry_.pose.pose.orientation);

    const double roomba_state_yaw = optimize_angle(map_to_base_yaw - odom_to_base_yaw);
    const double roomba_state_x = map_to_base_x - odom_to_base_x * cos(roomba_state_yaw) + odom_to_base_y * sin(roomba_state_yaw);
    const double roomba_state_y = map_to_base_y - odom_to_base_x * sin(roomba_state_yaw) - odom_to_base_y * cos(roomba_state_yaw);

    geometry_msgs::Quaternion roomba_state_q;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(roomba_state_yaw), roomba_state_q);

    geometry_msgs::TransformStamped roomba_state;
    roomba_state.header.stamp = ros::Time::now();

    roomba_state.header.frame_id = "map";
    roomba_state.child_frame_id = "odom";  //gitのやつがうまくいかなかったらこれ使う　1行下はコメントアウト
    // roomba_state.child_frame_id = "roomba4/odom";  //うまくいかんかった…

    roomba_state.transform.translation.x = roomba_state_x;
    roomba_state.transform.translation.y = roomba_state_y;
    roomba_state.transform.translation.z = 0.0;
    roomba_state.transform.rotation.x = roomba_state_q.x;
    roomba_state.transform.rotation.y = roomba_state_q.y;
    roomba_state.transform.rotation.z = roomba_state_q.z;
    roomba_state.transform.rotation.w = roomba_state_q.w;

    roomba_state_broadcaster_.sendTransform(roomba_state);
}

// -- process --
void Localizer::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(map_got_ && odometry_got_ && laser_got_)
        {
            if(init_request_particle_)
            {
                initialize();
                // publish_particles();
                init_request_particle_ = false;
            }

            if(can_move_)
            {
                move();
                // publish_particles();

                measurement();

                // ROS_INFO("ok\n");

                sampling();
            }

            if(is_visible_)
            {
                publish_particles();
            }

            publish_estimated_pose();

            try
            {
                broadcast_roomba_state();
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

// -- for_check --
/*
for(int i=0; i<particles_.size(); i++)
{
    ROS_INFO("%lf\n", dist_on_map(particles_[i].get_pose_x(), particles_[i].get_pose_y(), laser_.angle_min));
}

ROS_INFO("ok_\n");

int i = 0;
for(auto &p : particles_)
{
    ROS_INFO("p[%d]=%lf\n", i, calc_weight(p));
    i++;
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer");
    Localizer localizer;
    localizer.process();
}
