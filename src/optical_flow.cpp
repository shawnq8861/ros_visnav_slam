#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

static cv::Mat frame;

//
// subscribe to the image_raw topic to be notified when a new image
// is available
//
void getImage(const sensor_msgs::Image raw_image)
{
    //
    // convert raw image to Mat data type using cv bridge
    //


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optical_flow");
    ros::NodeHandle nh;
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 10;
    //
    // instantiate the subscriber for rest messages
    //
    ros::Subscriber image_sub = nh.subscribe("image_raw", 1000, getImage);
    //
    // loop while processing images and tracking keypoint motion
    // create overlay to depict motion and show image with overlay
    //
    int count = 0;
    while(ros::ok()) {
        //
        // example code form lkdemo.cpp goues in here
        //
        ++count;
        ros::spinOnce();
        loop_rate.sleep();
    }

}
