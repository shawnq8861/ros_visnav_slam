#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

namespace ros_visnav_slam_camera
{

class LumeneraCameraNodelet : public nodelet::Nodelet
{
public:
    LumeneraCameraNodelet()
    {}

private:
    virtual void onInit()
    {
        //
        // grab a node handle
        //
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        sub = private_nh.subscribe("image_raw", 10, &LumeneraCameraNodelet::subscrCB, this);
        timer = private_nh.createTimer(ros::Duration(2.0),
                                       boost::bind(&LumeneraCameraNodelet::timerCB, this, _1) );
    }

    void subscrCB(const sensor_msgs::ImageConstPtr& image)
    {
        NODELET_INFO_STREAM("image height: " << image->height);
        NODELET_INFO_STREAM("image width: " << image->width);
    }

    void timerCB(const ros::TimerEvent& timer_event)
    {
        //
        // print out simple message
        //
        NODELET_INFO_STREAM("The time is now " << timer_event.current_real);
    }
    ros::Subscriber sub;
    ros::Timer timer;
};

} // namespace ros_visnav_slam_camera

PLUGINLIB_EXPORT_CLASS(ros_visnav_slam_camera::LumeneraCameraNodelet, nodelet::Nodelet)
