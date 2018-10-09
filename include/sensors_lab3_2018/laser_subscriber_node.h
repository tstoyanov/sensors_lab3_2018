#ifndef LAB3_LASER_SUBSCRIBER_NODE_H //include guards
#define LAB3_LASER_SUBSCRIBER_NODE_H

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>

class LaserSubscriberNode {
    //members
    private:
        //node pointing to the global namespace /
        ros::NodeHandle n_;
        //node pointing to the local namespace ~
        ros::NodeHandle nh_;

        //subscriber to laser messages
        ros::Subscriber laser_sub_;

        //publishes a marker at a fixed frequency
        ros::Publisher marker_pub_;

        //service server for saving data to a file
        ros::ServiceServer save_data_srv_;

        //timer that trigers marker publishing
        ros::Timer marker_pub_timer_;

        //parameters to be loaded from the param server
        std::string laser_topic_, marker_topic_, output_file_name_;

        //message to be published at every iteration
        visualization_msgs::Marker marker_msg_;

    //functions
    private:
        //callback for service server
        bool save_data_callback(std_srvs::Empty::Request  &req,
                std_srvs::Empty::Response &res );
        //callback for publishing timer
        void marker_pub_callback(const ros::TimerEvent &te);
        //callback for laser message
        void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    public:
        //default constructor
        LaserSubscriberNode();

        //subscribes to topics and advertises publishers
        void subscribeAndAdvertise();

};

#endif
