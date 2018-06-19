#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <lumenera/lucamapi.h>

static uint32_t imageHeight = 0;
static uint32_t imageWidth = 0;

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
    // get the camera sensor properties
    //
    float height;
    float width;
    LONG flags;
    BOOL success = LucamGetProperty(hCamera, LUCAM_PROP_MAX_HEIGHT, &height, &flags);
    if(success) {
        ROS_INFO_STREAM("max height (pixels): " << height);
        imageHeight = (uint32_t)height;
    }
    success = LucamGetProperty(hCamera, LUCAM_PROP_MAX_WIDTH, &width, &flags);
    if(success) {
        ROS_INFO_STREAM("max width (pixels): " << width);
        imageWidth = (uint32_t)width;
    }
    //
    // create a vector to hold image data
    //
    std::vector<unsigned char> imageData(imageHeight * imageWidth);
    //
    // set camera for auto white balance, expsoure, and gain
    //

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
                                 width,
                                 height);
        //
        // set auto gain
        //
        LucamOneShotAutoGain(hCamera,
                             brightnessTarget,
                             startX,
                             startY,
                             width,
                             height);
        //
        // set auto white balance
        //
        LucamOneShotAutoWhiteBalance(hCamera,
                                     startX,
                                     startY,
                                     width,
                                     height);
        //
        // grab a snap shot
        //
        LONG singleFrame = 1;
        if(LucamTakeVideo(hCamera, singleFrame, (BYTE *)imageData.data()) == FALSE)
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
        image.data = imageData;
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
        //
        // process callbacks and check for messages
        //
        ros::spinOnce();
        loop_rate.sleep();
    }

    LucamCameraClose(hCamera);

}
