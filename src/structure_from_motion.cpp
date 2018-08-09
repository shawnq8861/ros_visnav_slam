#include <ros/ros.h>
//
// test the include for sfm module
//
#define CERES_FOUND 1
#include <opencv2/sfm.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "structure_from_motion");
    ros::NodeHandle nh;



    ROS_INFO("Hello world!");
}
