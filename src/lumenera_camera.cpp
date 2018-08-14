#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <camera_calibration_parsers/parse.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
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
static sensor_msgs::CameraInfo cameraInfo;
static const std::string cameraName = "lumenera_camera";
static const std::string calibrationFileName = "calibration_params.yaml";

//
// service callback used to set camera calibration parameters and save
// the values to a file
//
bool setAndSaveCameraCalibrationData(sensor_msgs::SetCameraInfo::Request &req,
         sensor_msgs::SetCameraInfo::Response &resp)
{
    //
    // set the camera info
    //
    cameraInfo.header.stamp = ros::Time::now();
    cameraInfo.height = imageHeight;
    cameraInfo.width = imageWidth;
    cameraInfo.distortion_model = req.camera_info.distortion_model;
    cameraInfo.D = req.camera_info.D;
    cameraInfo.K = req.camera_info.K;
    cameraInfo.R = req.camera_info.R;
    cameraInfo.P = req.camera_info.P;
    cameraInfo.binning_x = req.camera_info.binning_x;
    cameraInfo.binning_y = req.camera_info.binning_y;
    cameraInfo.roi = req.camera_info.roi;
    //
    // write data to yaml file
    //
    camera_calibration_parsers::writeCalibration(calibrationFileName,
                                                 cameraName,
                                                 cameraInfo);
    //
    // fill in the response object
    //
    resp.status_message = "calibration data set";

    return true;
}


//
// initialize the camera calibration parameters if the parameter file is
// no found
//
// default values are all set to zero
//
bool initializeCameraCalibrationData(void)
{
    //
    // read the data
    //
    if(camera_calibration_parsers::readCalibration(
                calibrationFileName,
                (std::string&)cameraName,
                cameraInfo)) {
        ROS_INFO_STREAM("calibration file read...");
        return EXIT_SUCCESS;
    }
    else {
        //
        // set the camera info to default values
        //
        cameraInfo.header.stamp = ros::Time::now();
        cameraInfo.height = imageHeight;
        cameraInfo.width = imageWidth;
        cameraInfo.distortion_model = "plumb_bob";
        cameraInfo.binning_x = 0;
        cameraInfo.binning_y = 0;
        sensor_msgs::RegionOfInterest roi;
        roi.height = imageHeight;
        roi.width = imageWidth;
        roi.x_offset = 0;
        roi.y_offset = 0;
        cameraInfo.roi = roi;
        //
        // write data to yaml file
        //
        bool retValue = camera_calibration_parsers::writeCalibration(
                    calibrationFileName,
                    cameraName,
                    cameraInfo);
        if(!retValue) {
            return EXIT_FAILURE;
        }
        else {
            return EXIT_SUCCESS;
        }
    }
}

//
// subscribe to the save_image topic to be notified when to save an image
// to a file
//
// TODO: convert this to a service
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
    ros::Publisher image_pub =
            nh.advertise<sensor_msgs::Image>("lumenera_camera/image_raw", 10);
    //
    // instantiate a publisher for camera calibration info
    //
    ros::Publisher camera_info_pub =
            nh.advertise<sensor_msgs::CameraInfo>("lumenera_camera/camera_info", 10);
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 10;
    //
    // instantiate the subscriber for save image messages
    //
    ros::Subscriber save_image = nh.subscribe("lumenera_camera/save_image",
                                              1000,
                                              &saveImage);


    //
    // instantiate a service to be called after the camera calibrated
    //
    ros::ServiceServer calibration_service = nh.advertiseService(
                "lumenera_camera/set_camera_info",
                &setAndSaveCameraCalibrationData);
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
    // initialize camera calibration data
    //
    if(EXIT_FAILURE == initializeCameraCalibrationData()) {
        ROS_ERROR_STREAM("calibration data initialization failed...");
        return EXIT_FAILURE;
    }

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
