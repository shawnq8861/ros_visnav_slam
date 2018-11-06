#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>

namespace ros_visnav_slam_camera
{

class CameraNodelet : public nodelet::Nodelet
{
public:
    CameraNodelet()
    {}

private:
    virtual void onInit()
    {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        pub = private_nh.advertise<std_msgs::String>("out", 10);
        sub = private_nh.subscribe("in", 10, &CameraNodelet::subscrCB, this);
        timer = private_nh.createTimer(ros::Duration(1.0),
                                       boost::bind(&CameraNodelet::timerCB, this, _1) );
    }

    void subscrCB(const std_msgs::StringConstPtr& input)
    {
        std_msgs::StringPtr output(new std_msgs::String());
        output->data = input->data;
        output->data += " and from camera_nodelet";
        NODELET_DEBUG("re-publishing message with add on");
        pub.publish(output);
    }

    void timerCB(const ros::TimerEvent& timer_event) {
        //
        // print out simple message
        //
        NODELET_INFO_STREAM("The time is now " << timer_event.current_real);
    }
    //
    // declare member elements
    //
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Timer timer;
};

} // namespace ros_visnav_slam_camera

PLUGINLIB_EXPORT_CLASS(ros_visnav_slam_camera::CameraNodelet, nodelet::Nodelet)
