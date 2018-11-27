#include "../include/camera_header.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lumenera_camera");
    ros::NodeHandle nh;
    //
    // check the calibration file path, return if not valid
    //
    if (!checkCalibrationFilePath()) {
        return EXIT_FAILURE;
    }
    //
    // instantiate a publisher for camera calibration info
    //
    ros::Publisher camera_info_pub =
            nh.advertise<sensor_msgs::CameraInfo>("lumenera_camera/camera_info", 10);
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = LOOP_AND_FRAME_RATE;
    //
    // instantiate a service to be called to save an image to file
    //
    ros::ServiceServer save_image_service = private_nh.advertiseService<ros_visnav_slam::SaveImageRequest,
            ros_visnav_slam::SaveImageResponse>(
                "save_image",
                boost::bind(saveImageCB, _1, _2, &imageHeight, &imageWidth, &frame));


            save_image_service = nh.advertiseService(
                "lumenera_camera/save_image",
                &saveImageCB);
    //
    // initialize camera calibration data
    //
    sensor_msgs::CameraInfo cameraInfo;
    if(EXIT_FAILURE == initializeCameraCalibrationData(cameraInfo)) {
        ROS_ERROR_STREAM("calibration data initialization failed...");
        return EXIT_FAILURE;
    }
    //
    // instantiate a service to be called after the camera calibrated
    //
    ros::ServiceServer calibration_service = nh.advertiseService<sensor_msgs::SetCameraInfoRequest,
            sensor_msgs::SetCameraInfoResponse>(
                "set_camera_info",
                boost::bind(setAndSaveCameraCalibrationData,
                            _1, _2, &cameraInfo, &imageHeight, &imageWidth) );

    //
    // get the number of cameras
    //
    LONG numCameras = LucamNumCameras();
    ROS_INFO_STREAM("number of cameras: " << numCameras);
    //
    // attempt to get a camera handle
    //
    HANDLE hCamera = LucamCameraOpen(1);
    if (NULL == hCamera) {
        ROS_ERROR_STREAM("ERROR %u: Unable to open camera.\n"
                         << (unsigned int)LucamGetLastError());
        return EXIT_FAILURE;
    }
    //
    // get the frame format
    //
    LUCAM_FRAME_FORMAT frameFormat;
    float frameRate = -1.0f;
    LucamGetFormat(hCamera, &frameFormat, &frameRate);
    imageWidth = (frameFormat.width / frameFormat.subSampleX);
    imageHeight = (frameFormat.height / frameFormat.subSampleY);
    LUCAM_CONVERSION conversionParams;
    conversionParams.CorrectionMatrix = LUCAM_CM_FLUORESCENT;
    conversionParams.DemosaicMethod = LUCAM_DM_HIGHER_QUALITY;
    //
    // display current frame rate
    //
    ROS_INFO_STREAM("current frame rate: " << frameRate);
    //
    // set new frame rate
    //
    LucamSetFormat(hCamera, &frameFormat, LOOP_AND_FRAME_RATE);
    //
    // display current frame rate
    //
    LucamGetFormat(hCamera, &frameFormat, &frameRate);
    ROS_INFO_STREAM("current frame rate: " << frameRate);
    loop_rate = (ros::Rate)frameRate;
    //
    // create vectors to hold image data
    //
    std::vector<unsigned char> rawImageData(imageHeight * imageWidth);
    //
    // start the video stream, NULL window handle
    //
    if (LucamStreamVideoControl(hCamera, START_STREAMING, NULL) == FALSE) {
       ROS_INFO_STREAM("Failed to start streaming");
    }
    //
    // code to allow loading the nodelet plugin dynamically form a running node
    //
    std::string nodelet_name = "lumenera_camera_nodelet";
    std::string nodelet_topic = "/image_raw";
    nodelet_topic = nodelet_name + nodelet_topic;
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load(nodelet_name, "ros_visnav_slam_camera/LumeneraCameraNodelet", remap, nargv);
    //
    // instantiate a publisher for camera images
    //
    ros::Publisher image_pub =
            nh.advertise<sensor_msgs::Image>(nodelet_topic, 10);

    //
    // loop while acquiring image frames from the stream
    //
    int count = 0;
    while(ros::ok()) {
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
        // reset the counter
        //
        count = 0;
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
        //
        // process callbacks and check for messages
        //
        ros::spinOnce();
        loop_rate.sleep();
    }

    LucamCameraClose(hCamera);

}
