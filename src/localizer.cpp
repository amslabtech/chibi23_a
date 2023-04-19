#include "localizer.h"

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
    private_nh_.param("hz",hz_, {10});
    private_nh_.param("num",num_, {600});
    private_nh_.param("init_x",init_x_, {0.0});
    private_nh_.param("init_y",init_y_, {0.0});
    private_nh_.param("init_yaw",init_yaw_, {0.0});
    private_nh_.param("init_dev",init_dev_, {0.40});

    private_nh_.param("intercept", intercept, {0.40});

    private_nh_.param("laser_dev_per_dist", laser_dev_per_dist_, {0.10});
    private_nh_.param("laser_step", laser_step_, {20});

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
    return exp(-pow(x - mu, 2) / (2.0 * pow(dev, 2))) / (sqrt(2.0 * M_PI) * dev);
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

// -- move particles --
void Localizer::get_motion(Particle& p, double distance, double direction, double rotation)
{
    //distance, direction, rotationにnoiseを付与
    //各値が大きくなるほど大きくぶれるように
    distance += set_noise(0.0, distance);
    direction += set_noise(0.0, direction);
    rotation += set_noise(0.0, rotation);

    double x = p.get_pose_x() + distance * cos(optimize_angle(direction + p.get_pose_yaw()));
    double y = p.get_pose_y() + distance * sin(optimize_angle(direction + p.get_pose_yaw()));
    double yaw = optimize_angle(direction + p.get_pose_yaw());

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

double Localizer::dist_on_map(double map_x, double map_y, double laser_angle)
{
    double search_limit = laser_.range_max / 2.0;

    double x_dist = search_limit * cos(laser_angle);
    double y_dist = search_limit * sin(laser_angle);

    double x_now = map_x;
    double y_now = map_y;

    double distance = 0.0;

    map_x += x_dist;
    map_y += y_dist;

    int map_flag = get_map_data(map_x, map_y);

    if(map_flag != 0)
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
            else if(map_flag > 10) //50は適当に決めた
            {
                distance = hypot(map_x - x_now, map_y - y_now);
                return distance;
            }

        }
        //ROS_INFO("over_calculation_limit\n");
        return hypot(map_x - x_now, map_y - y_now);

    }else return search_limit;

}

// -- weight --
double Localizer::likelihood_model(double x, double mu, double dev)
{
    double ans = 0.0;
    if(x >= laser_.range_max / 2.0)
    {
        ans = 0.1; //適当
    }else
    {
        ans = 0.01; //適当
    }

    if(ans < liner(x, mu)) ans = liner(x, mu);
    if(ans < gaussian(x, mu, dev)) ans = gaussian(x, mu, dev);

    return ans;
}

double Localizer::calc_weight(Particle& p)
{
    double weight = 0.0;
    double angle = optimize_angle(p.get_pose_yaw() + laser_.angle_min);
    double angle_step = laser_.angle_increment;
    int limit = laser_.ranges.size();

    for(int i = 0; i < limit; i += laser_step_)
    {
        double map_dist = 0.0;
        map_dist = dist_on_map(p.get_pose_x(), p.get_pose_y(), angle);

        double sigma = laser_.ranges[i] * laser_dev_per_dist_;

        weight += likelihood_model(map_dist, laser_.ranges[i], sigma);

        angle = optimize_angle(angle + angle_step * laser_step_);
    }

    return weight;

}

void Localizer::normalize_weight()
{
    sum_weight = 0.0;
    for(int i=0; i<particles_.size(); i++)
    {
        current_sum_weight += particles_[i].get_weight();
    }

    // ROS_INFO("sum_weight=%lf\n",sum_weight);

    for(auto &p : particles_)
    {
        double new_weight = p.get_weight() / current_sum_weight;
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
    double max_weight = 0.0;
    int data = 0.0;

    for(int i=0; i < particles_.size(); i++)
    {
        if(particles_[i].get_weight() > max_weight)
        {
            max_weight = particles_[i].get_weight();
            data = i;
        }
    }

    Particle estimated_pose_(particles_[data].get_pose_x(), particles_[data].get_pose_y(), particles_[data].get_pose_yaw(), particles_[data].get_weight());

    // ROS_INFO("p[%d] is the most likely particle.\n", data);
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
}

// -- resamling --
//need to check
int Localizer::particle_number(int num, double previous_sum_weight, double current_sum_weight)
{
    int new_num_ = num_ * ( previous_sum_weight / current_sum_weight);
    return new_num_;
}

//need to check
void Localizer::sampling()
{
    std::vector<Particle> new_particles;

    int new_num_ = particle_number(num_, previous_sum_weight, current_sum_weight);
    new_particles.reserve(new_num_);

    ROS_INFO("new_num is %d\n", new_num_);

    std::uniform_int_distribution<> int_dist(0, num_ - 1);
    //合計が決まった値になるまで，ランダムでparticleを選ぶ
    //選んだparticleの尤度からばら撒く数を決定
    //決定した数だけ選んだparticleを中心に再度ばら撒く
    int p = 0;
    while(p < new_num_)
    {
        int index = int_dist(engine);
        specific_num_ = (int)(new_num_ * (particles_[index].get_weight()));
        ROS_INFO("specific_num is %d\n", specific_num_);
        double resamling_dev = 1 - particles_[index].get_weight();
        for(int i=0; i<specific_num_; i++)
        {
            double x = set_noise(particles_[index].get_pose_x(), resamling_dev);
            double y = set_noise(particles_[index].get_pose_y(), resamling_dev);
            double yaw = set_noise(particles_[index].get_pose_yaw(), resamling_dev);
            Particle p(x, y, yaw);
            new_particles.push_back(p);
        }
        p += specific_num_;
    }

    particles_ = new_particles;
    previous_sum_weight = current_sum_weight;
    reset_weight();
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
                publish_particles();
                init_request_particle_ = false;
            }

            if(can_move_)
            {
                move();
                publish_particles();

                measurement();

                if(init_request_sum_weight_)
                {
                    previous_sum_weight = current_sum_weight;
                    init_request_sum_weight_ = false;
                }

                sampling();
                publish_particles();
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
