#include <sensors_lab3_2018/laser_subscriber_node.h>

using namespace std;


bool LaserSubscriberNode::save_data_callback(std_srvs::Empty::Request  &req,
        std_srvs::Empty::Response &res ) {
    //TODO
    ROS_INFO("Saving data to %s", output_file_name_.c_str());
}

void LaserSubscriberNode::marker_pub_callback(const ros::TimerEvent &te) {
    //TODO
    ROS_INFO("Publishing markers");
}

void LaserSubscriberNode::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //TODO
    ROS_INFO("Got a laser scan message with %zd range measurements", msg->ranges.size());
}

LaserSubscriberNode::LaserSubscriberNode() {
    nh_ = ros::NodeHandle("~");
    n_ = ros::NodeHandle();

    nh_.param<std::string>("laser_topic",laser_topic_,"/scan");
    nh_.param<std::string>("marker_topic",marker_topic_,"viz");
    nh_.param<std::string>("outfile",output_file_name_,"test.m");

}

void LaserSubscriberNode::subscribeAndAdvertise() {
    //advertise publisher
    marker_pub_ = nh_.advertise<visualization_msgs::Marker> (marker_topic_,10);
    //subscribe to laser messages
    laser_sub_ = n_.subscribe(laser_topic_, 100, &LaserSubscriberNode::laser_scan_callback, this);
    //advertise service
    save_data_srv_ = nh_.advertiseService("save_data", &LaserSubscriberNode::save_data_callback, this);
    //setup timer
    marker_pub_timer_ = nh_.createTimer(ros::Duration(1.0), &LaserSubscriberNode::marker_pub_callback, this);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_sub_node");

    LaserSubscriberNode laser_node;
    ROS_INFO("Laser Subscriber node created");
    laser_node.subscribeAndAdvertise();
    ROS_INFO("Laser Subscriber node initialized");

    ros::spin();

    return 0;
}

