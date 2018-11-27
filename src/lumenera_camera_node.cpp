#include <ros/ros.h>
#include <nodelet/loader.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    //
    // standard node code...
    //
    ros::init(argc, argv, "camera_acquire");
    //
    // nodelet manager code
    //
    // cofigure and load each node
    //
    // first the acquire and publish nodelet
    //
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name,
                 "ros_visnav_slam_camera/CameraAcquireNodelet",
                 remap,
                 nargv);
    //
    // spin the node
    //
    ros::spin();
}
