#include <ros/ros.h>
#include <nodelet/loader.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    //
    // standard node code...
    //
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    //
    // code to allow loading the nodelet plugin dynamically form a running node
    //
    std::string nodelet_name = "camera_nodelet";
    std::string nodelet_topic = "/in";
    nodelet_topic = nodelet_name + nodelet_topic;
    ros::Publisher test_pub = nh.advertise<std_msgs::String>(nodelet_topic, 10);
    ROS_INFO_STREAM("camera node...");
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load(nodelet_name, "ros_visnav_slam_camera/CameraNodelet", remap, nargv);
    ros::Rate loop_rate = 1;
    std_msgs::String message;
    message.data = "hello from camera node";
    //
    // spin the node, publish the node message
    //
    while (ros::ok()) {
        test_pub.publish(message);
        ros::spinOnce();
        loop_rate.sleep();
    }

}
