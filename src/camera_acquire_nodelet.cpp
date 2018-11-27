#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "../include/camera_header.hpp"

namespace ros_visnav_slam_camera
{

class CameraAcquireNodelet : public nodelet::Nodelet
{
public:
    CameraAcquireNodelet()
    {}

private:
    virtual void onInit()
    {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        //
        // create the raw_image topic to publish image data to
        //
        image_pub = private_nh.advertise<sensor_msgs::Image>("image_raw", 10);
        //
        // instantiate a publisher for camera calibration info
        //
        camera_info_pub =
                private_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 10);
        //
        // configure the camera
        //
        configureCamera(hCamera, imageWidth, imageHeight, frameRate);
        //
        // set size of raw data vector
        //
        rawImageData.resize(imageHeight*imageWidth, 0);
        //
        // create the timer callback in which the images will be published
        //
        timer = private_nh.createTimer(ros::Duration(1.0/frameRate),
                               boost::bind(&CameraAcquireNodelet::timerCB,
                               this, _1) );
        //
        // instantiate a service to be called to save an image to file
        //
        save_image_service = private_nh.advertiseService
                <ros_visnav_slam::SaveImageRequest,
                ros_visnav_slam::SaveImageResponse>(
                    "save_image",
                    boost::bind(saveImageCB,
                                _1, _2,
                                &imageHeight, &imageWidth, &frame));
        //
        // initialize camera calibration data
        //
        if(EXIT_FAILURE == initializeCameraCalibrationData(cameraInfo,
                                                           imageHeight,
                                                           imageWidth)) {
            NODELET_ERROR_STREAM("calibration data initialization failed...");
        }
        //
        // instantiate a service to be called after the camera calibrated
        //
        calibration_service = private_nh.advertiseService<sensor_msgs::SetCameraInfoRequest,
                sensor_msgs::SetCameraInfoResponse>(
                    "set_camera_info",
                    boost::bind(setAndSaveCameraCalibrationData,
                                _1, _2, &cameraInfo,
                                &imageHeight, &imageWidth));
    }

    //
    // this takes the place of the while loop in the non-nodelet version
    //
    void timerCB(const ros::TimerEvent& timer_event) {
        //
        // publish the raw image data
        //
        // set one shot auto exposure target
        //
        UCHAR brightnessTarget = TARGET_BRIGHTNESS;
        ULONG startX = 0;
        ULONG startY = 0;
        //
        // set auto exposure
        //
        LucamOneShotAutoExposure(hCamera,
                                 brightnessTarget,
                                 startX,
                                 startY,
                                 imageWidth,
                                 imageHeight);
        //
        // set auto gain
        //
        LucamOneShotAutoGain(hCamera,
                             brightnessTarget,
                             startX,
                             startY,
                             imageWidth,
                             imageHeight);
        //
        // set auto white balance
        //
        LucamOneShotAutoWhiteBalance(hCamera,
                                     startX,
                                     startY,
                                     imageWidth,
                                     imageHeight);
        //
        // grab a snap shot
        //
        LONG singleFrame = 1;
        if(LucamTakeVideo(hCamera, singleFrame, (BYTE *)rawImageData.data()) == FALSE) {
            ROS_ERROR_STREAM("Failed to capture image");
        }
        //
        // declare an image message object to hold the data
        //
        sensor_msgs::Image image;
        //
        // update camera info time stamp
        //
        cameraInfo.header.stamp = ros::Time::now();
        //
        // configure the image message
        //
        image.header.stamp = cameraInfo.header.stamp;
        image.data = rawImageData;
        image.height = imageHeight;
        image.width = imageWidth;
        image.step = imageWidth;
        image.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
        //
        // save the snap shot to OpenCV Mat
        //
        frame = cv::Mat(cv::Size(imageWidth, imageHeight),
                        CV_8UC1,
                        rawImageData.data(),
                        cv::Mat::AUTO_STEP);
        //
        // publish the image to an image_view data type
        //
        image_pub.publish(image);
        //
        // publish the camera info
        //
        camera_info_pub.publish(cameraInfo);
    }

    //
    // declare member elements
    //
    ros::Publisher image_pub;
    ros::Publisher camera_info_pub;
    ros::Timer timer;
    ros::ServiceServer save_image_service;
    ros::ServiceServer calibration_service;
    //
    // camera parameters
    //
    HANDLE hCamera;
    ULONG imageWidth;
    ULONG imageHeight;
    float frameRate;
    sensor_msgs::CameraInfo cameraInfo;
    //
    // create vector and cv::Mat to hold image data
    //
    std::vector<unsigned char> rawImageData;
    cv::Mat frame;
};

} // namespace ros_visnav_slam_camera

PLUGINLIB_EXPORT_CLASS(ros_visnav_slam_camera::CameraAcquireNodelet, nodelet::Nodelet)
