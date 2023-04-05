#include "localizer/localizer.h"

std::random_device seed_gen;
std::mt19937 engine(seed_gen());

Particle::Particle(double x, double y, double yaw, double weight)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
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

    sub_laser_ = nh_.subscribe("/scan", 10, &Localizer::laser_callback, this);
    sub_odometry_ = nh_.subscribe("/roomba/odometry", 10, &Localizer::odometry_callback, this);
    sub_map_ = nh_.subscribe("/map", 10, &Localizer::map_callback, this);

    pub_estimated_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
    pub_particle_cloud_ = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1);

}

void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{

}

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_ = *msg;
    map_got_ = true;
}

double Localizer::set_noise(double mu, double dev)
{
    std::normal_distribution<> dist(mu, dev);
    return dist(engine);
}

void Localizer::initializer()
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
    particle_cloud_msg_.heder.stamp = ros::time::now();
    particle_cloud_msg_.heder.frame_id = "map";

    for(int i=0; i<particles_.size(); i++)
    {
        particle_cloud_msg_.poses[i].position.x = particles_[i].get_pose_x();
        particle_cloud_msg_.poses[i].position.y = particles_[i].get_pose_y();
        particle_cloud_msg_.poses[i].position.z = 0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, particles_[i].get_pose_yaw());
        tf2::convert(q, particle_cloud_msg_.poses[i].orientation);
    }

    pub_particle_cloud_.publish(particle_cloud_msg_);
}

void Localizer::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(map_got_)
        {
            initializer();
            publish_particles();
        }
    }
    ros::spinOnce();
    loop_rate.sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer");
    Localizer localizer;
    localizer.process();
}
