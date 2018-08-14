//
// subscribe to a topic the starts image reading and 3D reconstruction
// read in 3 to 5 images
// read the camera calibration info
// call reconstruct
// publish results for view using rviz
//

#include <ros/ros.h>
#include <camera_calibration_parsers/parse.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
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
static std::vector<cv::String> imagePaths;
static cv::Matx33d cameraIntrinsics;

//
// subscribe to the save_image topic to be notified when to save an image
// to a file
//
void performReconstruction(std_msgs::Int8 imageCount)
{
    //
    // read in the images
    //
    ROS_INFO_STREAM("image count: " << (int)imageCount.data);
    //
    // build the camera calibration matrix
    //
    // initialize the image file paths
    //
    char *home = getenv("HOME");
    std::string absolutePath = (std::string)home + relativePath;
    const std::string imageFileBase = "sfm_image";
    for (int i = 0; i < (int)(imageCount.data); ++i) {
        std::stringstream number;
        number << (i + 1);
        std::string numberStr;
        number >> numberStr;
        std::string imagePath = absolutePath +
                imageFileBase + numberStr + ".jpg";
        ROS_INFO_STREAM("file path" << i + 1 << ": " << imagePath);
        imagePaths.push_back(imagePath);
    }
    //
    // map 1 x 9 to 3 x 3
    // 0,0 -> 0
    // 1,1 -> 4
    // 0,2 -> 2
    // 1,2 -> 5
    //
    double fx = cameraInfo.K[0];
    double fy = cameraInfo.K[4];
    double cx = cameraInfo.K[2];
    double cy = cameraInfo.K[5];
    ROS_INFO_STREAM("fx = " << fx);
    ROS_INFO_STREAM("fy = " << fy);
    ROS_INFO_STREAM("cx = " << cx);
    ROS_INFO_STREAM("cy = " << cy);
    //
    // build the camera calibration matrix
    //
    cv::Matx33d K = cv::Matx33d( fx,  0.0, cx,
                                 0.0, fy,  cy,
                                 0.0, 0.0, 1.0);
    bool is_projective = true;
    std::vector<cv::Mat> Rs;
    std::vector<cv::Mat> Ts;
    std::vector<cv::Mat> points3d;
    ROS_INFO_STREAM("fx = " << K(0,0));
    ROS_INFO_STREAM("cx = " << K(0,2));
    ROS_INFO_STREAM("fy = " << K(1,1));
    ROS_INFO_STREAM("cy = " << K(1,2));
    ROS_INFO_STREAM("K[2,0] = " << K(2,0));
    cv::sfm::reconstruct(imagePaths,
                         Rs,
                         Ts,
                         K,
                         points3d,
                         is_projective);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "structure_from_motion");
    ros::NodeHandle nh;
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
