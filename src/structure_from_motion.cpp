//
// subscribe to a topic the starts image reading and 3D reconstruction
// read in 5 images
// read the camera calibration info
// call reconstruct
// publish results for view using rviz
//

#include <ros/ros.h>
#include <camera_calibration_parsers/parse.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <string>
#include <vector>
#include <stdlib.h>
#include <sstream>
#define CERES_FOUND 1
#include <opencv2/sfm.hpp>

static sensor_msgs::CameraInfo cameraInfo;
static const std::string cameraName = "lumenera_camera";
static const std::string calibrationFileName = "calibration_params.yaml";
static const int numImages = 3;
static const std::string relativePath = "/Pictures/SFM/";
static std::vector<std::string> imagePaths;
static cv::Matx33d cameraIntrinsics;

//
// subscribe to the save_image topic to be notified when to save an image
// to a file
//
void performReconstruction(std_msgs::Float32 focalLength)
{
    //
    // read in the images
    //
    ROS_INFO_STREAM("focal length: " << focalLength);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "structure_from_motion");
    ros::NodeHandle nh;
    //
    // initialize the image file paths
    //
    char *home = getenv("HOME");
    std::string absolutePath = (std::string)home + relativePath;
    const std::string imageFileBase = "sfm_image";
    for (int i = 0; i < numImages; ++i) {
        std::stringstream number;
        number << (i + 1);
        std::string numberStr;
        number >> numberStr;
        std::string imagePath = absolutePath +
                imageFileBase + numberStr + ".jpg";
        ROS_INFO_STREAM("file path" << i + 1 << ": " << imagePath);
        imagePaths.push_back(imagePath);
    }
    ROS_INFO_STREAM("file path vector size: " << imagePaths.size());
    //
    // get the camera Info
    //
    if(camera_calibration_parsers::readCalibration(
                calibrationFileName,
                (std::string&)cameraName,
                cameraInfo)) {
        ROS_INFO_STREAM("calibration file read...");
    }
    //
    // build the camera calibration matrix
    //
    cv::Matx33d K = cv::Matx33d( f, 0, cx,
                                 0, f, cy,
                                 0, 0,  1);

    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 10;
    //
    // instantiate the subscriber for reconstruct messages
    //
    ros::Subscriber reconstruct_sub = nh.subscribe("structure_from_motion/reconstruct",
                                              1000,
                                              &performReconstruction);

    //
    // loop while waiting for publisher to request reconstruction
    //
    while(ros::ok()) {
        //
        // process callbacks and check for messages
        //
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}
