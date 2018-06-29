#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <lumenera/lucamapi.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <exception>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lumenera_camera");
    ros::NodeHandle nh;
    //
    // instantiate a publisher for camera images
    //
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("image_raw", 10);
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 10;
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
    const ULONG imageWidth = (frameFormat.width / frameFormat.subSampleX);
    const ULONG imageHeight = (frameFormat.height / frameFormat.subSampleY);
    LUCAM_CONVERSION conversionParams;
    conversionParams.CorrectionMatrix = LUCAM_CM_FLUORESCENT;
    conversionParams.DemosaicMethod = LUCAM_DM_HIGHER_QUALITY;
    //
    // create vectors to hold image data
    //
    std::vector<unsigned char> rawImageData(imageHeight * imageWidth);
    std::vector<unsigned char> rgbImageData(imageHeight * imageWidth);
    //
    // start the video stream, NULL window handle
    //
    if (LucamStreamVideoControl(hCamera, START_STREAMING, NULL) == FALSE)
    {
       ROS_INFO_STREAM("Failed to start streaming");
    }
    //
    // loop while acquiring image frames from the stream
    //
    int count = 0;
    while(ros::ok()) {
        //
        // set one shot auto exposure target
        //
        UCHAR brightnessTarget = 90;
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
        if(LucamTakeVideo(hCamera, singleFrame, (BYTE *)rawImageData.data()) == FALSE)
        {
            ROS_ERROR_STREAM("Failed to capture image");
        }
        //
        // declare an image message object to hold the data
        //
        sensor_msgs::Image image;
        //
        // configure the image message
        //
        image.header.stamp = ros::Time::now();
        image.data = rawImageData;
        image.height = imageHeight;
        image.width = imageWidth;
        image.step = imageWidth;
        image.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
        //
        // publish the image to an image_view data type
        //
        image_pub.publish(image);
        //
        // save the snap shot to file
        //
        cv::Mat frame(cv::Size(imageWidth, imageHeight),
                      CV_8UC1,
                      rawImageData.data(),
                      cv::Mat::AUTO_STEP);
        ROS_INFO_STREAM("number of channels: " << frame.channels());
        ROS_INFO_STREAM("number of cols: " << frame.cols);
        ROS_INFO_STREAM("number of rows: " << frame.rows);
        ROS_INFO_STREAM("type: " << frame.type());
        //
        // build up jpeg image data and write to file
        //
        if(count == 10) {
            ROS_INFO_STREAM("saving file...");
            ROS_INFO_STREAM("creating frame...");
            cv::Mat saveFrame(cv::Size(imageWidth, imageHeight),
                              CV_8UC3);
            cv::cvtColor(frame, saveFrame, cv::COLOR_BayerBG2BGR);
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params.push_back(95);
            ROS_INFO_STREAM("saving frame...");
            cv::imwrite("/home/shawn/Pictures/test_image.jpg",
                        saveFrame,
                        compression_params);
        }
        //
        // process callbacks and check for messages
        //
        ++count;
        ros::spinOnce();
        loop_rate.sleep();
    }

    LucamCameraClose(hCamera);

}
