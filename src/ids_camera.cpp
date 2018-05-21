#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <libgen.h>
#include <string>
#include <exception>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/photo/photo.hpp>
#include <ueye.h>

static cv::Mat image;
static HIDS hCam;
INT height;
INT width;
char *ptrImgMem;
int memId;
INT bitsPerPixel;

int configureCamera(void);

//
// create a callback function to process the messages
// received on the subcribed topic
//
void saveImageCallback(const std_msgs::String::ConstPtr& msg)
{
    //
    // extract the save image file path from the messaage
    //
    ROS_INFO("subscriber received:  [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ids_camera");
    ros::NodeHandle nh;
    //
    // call a function to initialize the camera
    //
    int retValue = configureCamera();
    if (retValue != 0) {
        ROS_ERROR_STREAM("failed to configure camera...");
        return retValue;
    }
    else {
        //
        // declare and initialize the save image subscriber
        // declare and initialize the image publisher
        // enable auto parameters
        // enable auto gain and white balance
        // start while loop
        //  - grab a frame
        //  - publish current frame to image topic
        //
        // declare published topics
        //
        // i.e. raw image, hdr image, etc
        //
        //
        // set the maximum number of messages storable in the queue buffer
        //
        int queue_size = 1000;
        //
        // calling advertise instantiates a Publiser object
        //
        ros::Publisher pub = nh.advertise<sensor_msgs::Image>(
                    "ids_image", queue_size);
        //
        // set the loop rate used by spin to control while loop execution
        // this is an integer that equates to loops/second
        //
        ros::Rate loop_rate = 2;
        uint imageFormat = CV_8UC3;     // 8/24 bit unsigned bayer image
        image = Mat(Size(width, height), imageFormat);

        //
        // grab the next available frame
        //
        INT frameGrabReturn = -1;
        frameGrabReturn = is_FreezeVideo(hCam, IS_WAIT);
        if (frameGrabReturn == IS_SUCCESS) {
            //
            // link camera image buffer to OpenCV matrix
            //
            image.data = (uchar *)ptrImgMem;
        }
        else {
            cout << "frame grab failed..." << endl;
        }
        while (ros::ok()) {
            //
            // create a msg object to hold the image
            //
            cv_bridge::CvImage image_bridge;
            sensor_msgs::Image image_msg;


            //
            // publish the message to the topic
            //
            pub.publish(msg);
            //
            // direct to message strin to stdout
            //
            ROS_INFO("%s", msg.data.c_str());
            //
            // spinOnce() allows access to callback functions if we have any
            //
            ros::spinOnce();
            //
            // sleep or block or go inactive until the next loop iteration
            //
            loop_rate.sleep();
        }
        //
        // declare services? nor sure it is needed
        //
        // i.e. saved image
        //      - arg would be file path
        //      - return value would be url
        //

    }
}

int configureCamera()
{
    //
    // camera configuration variables
    //
    hCam = 1;
    cout << "configuring camera..." << endl;
    //
    // get the camera handle
    //
    if (is_InitCamera(&hCam, NULL) != IS_SUCCESS) {
        cout << "could not get camera handle..." << endl;
        return 1;
    }
    //
    // set the display mode to binary for direct image buffer access
    //
    INT displayMode = IS_SET_DM_DIB;    // binary display mode
    if (is_SetDisplayMode(hCam, displayMode) != IS_SUCCESS) {
        cout << "could not set display mode..." << endl;
    }
    usleep(delay);
    //
    // get the camera sensor information
    //
    SENSORINFO camSensorInfo;
    camSensorInfo.nColorMode = -1;
    if (is_GetSensorInfo(hCam, &camSensorInfo) != IS_SUCCESS) {
        cout << "could not get sensor info..." << endl;
    }
    //
    // set the image properties
    //
    height = (INT)camSensorInfo.nMaxHeight;
    width = (INT)camSensorInfo.nMaxWidth;
    ptrImgMem = NULL;     // pointer to start of active image memory
    memId;                  // numerical identifier for active memory
    bitsPerPixel = 24;      // 8 bit per pixel/color channel
    //
    // intitialize camera memory
    //
    INT allocResult;
    allocResult = is_AllocImageMem(hCam, width, height,
                                   bitsPerPixel, &ptrImgMem, &memId);
    if (allocResult != IS_SUCCESS) {
        cout << "could not allocate image mem..." << endl;
    }
    usleep(delay);
    //
    // set allocated memory as active memory
    //
    INT setActiveResult = -1;
    setActiveResult = is_SetImageMem(hCam, ptrImgMem, memId);
    if (setActiveResult != IS_SUCCESS) {
        cout << "failed to set image mem active..." << endl;
    }

    return 0;
}
