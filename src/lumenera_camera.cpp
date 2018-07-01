#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <lumenera/lucamapi.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <libgen.h>
#include <string.h>

static ULONG imageWidth;
static ULONG imageHeight;
static cv::Mat frame;

//
// subscribe to the save_image topic to be notified when to save an image
// to a file
//
void saveImage(const std_msgs::String path)
{
    //
    // extract the directory and base name from the path
    //
    ROS_INFO_STREAM("saved file path: " << path);
    std::string pathStr = path.data;
    char *pathCopy = strdup(pathStr.c_str());
    char *baseName = basename(pathCopy);
    char *dirName = dirname(pathCopy);
    //
    // check file extension
    //
    std::string fileExt = ".jpg";
    std::string baseStr(baseName);
    size_t pos = baseStr.find_first_of(".");
    std::string baseExt = baseStr.substr(pos);
    if (0 != strcmp(fileExt.c_str(), baseExt.c_str())) {
        ROS_INFO_STREAM("usage:  please use file extension .jpg...");
    }
    //
    // create the directory if it does not exist
    //
    struct stat statBuff;
    int dirFound = stat(dirName, &statBuff);
    if (dirFound == -1) {
        ROS_INFO_STREAM("directory does not exist, creating...");
        int dirCreated = mkdir(dirName, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (dirCreated == -1) {
            ROS_INFO_STREAM("mkdir failed...");
        }
    }
    cv::Mat saveFrame(cv::Size(imageWidth, imageHeight),CV_8UC3);
    cv::cvtColor(frame, saveFrame, cv::COLOR_BayerBG2RGB);
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(95);
    ROS_INFO_STREAM("saving frame...");
    cv::imwrite(path.data,
                saveFrame,
                compression_params);
}

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
    // instantiate the subscriber for rest messages
    //
    ros::Subscriber save_image = nh.subscribe("save_image", 1000, saveImage);
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
    // create vectors to hold image data
    //
    std::vector<unsigned char> rawImageData(imageHeight * imageWidth);
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
        // save the snap shot to OpenCV Mat
        //
        frame = cv::Mat(cv::Size(imageWidth, imageHeight),
                      CV_8UC1,
                      rawImageData.data(),
                      cv::Mat::AUTO_STEP);
        //
        // process callbacks and check for messages
        //
        ++count;
        ros::spinOnce();
        loop_rate.sleep();
    }

    LucamCameraClose(hCamera);

}
