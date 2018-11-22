#ifndef CAMERA_HEADER_HPP
#define CAMERA_HEADER_HPP

#include <ros/ros.h>
#include <nodelet/loader.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <camera_calibration_parsers/parse.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <ros_visnav_slam/SaveImage.h>
#include <lumenera/lucamapi.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <libgen.h>
#include <string.h>
#include <pwd.h>
#include <mutex>
#include <boost/filesystem.hpp>

#define TARGET_BRIGHTNESS       60
#define LOOP_AND_FRAME_RATE     30.0
#define CHAR_BUFFER_SIZE        100

static ULONG imageWidth;
static ULONG imageHeight;
static cv::Mat frame;
static sensor_msgs::CameraInfo cameraInfo;
static const std::string cameraName = "lumenera_camera";
static const char *calibrationFolderName =
        "/catkin_ws/src/ros_visnav_slam/data/";
static const char *calibrationFileName = "calibration_params.yaml";
static char calibrationFilePath[CHAR_BUFFER_SIZE];

//
// helper function that checks calibration file path, and creates the
// directory if it does not exist
//
bool checkCalibrationFilePath(void)
{
    //
    // get the home directory
    //
    struct passwd *pw = getpwuid(getuid());
    char *homeDir = pw->pw_dir;
    //
    // build up the file path
    //
    // first home home directory
    //
    strcpy(calibrationFilePath, homeDir);
    //
    // next directory path
    //
    strcat(calibrationFilePath, calibrationFolderName);
    ROS_INFO_STREAM("folder dir: " << calibrationFilePath);
    //
    // create the directory if it does not exist
    //
    struct stat statBuff;
    int folderFound = stat(calibrationFilePath, &statBuff);
    if (folderFound == -1) {
        if (boost::filesystem::create_directory(calibrationFilePath)) {
            ROS_INFO_STREAM("data directory created");
        }
        else {
            ROS_INFO_STREAM("could not create data directory");
            return false;
        }
    }
    else {
        ROS_INFO_STREAM("folder found...");
    }
    //
    // and finally the file name
    //
    strcat(calibrationFilePath, calibrationFileName);
    ROS_INFO_STREAM("calibration file path: " << calibrationFilePath);

    return true;
}

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
    camera_calibration_parsers::writeCalibration(calibrationFilePath,
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
                calibrationFilePath,
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
                    calibrationFilePath,
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
// service to the save an image to a file
//
bool saveImageCB(ros_visnav_slam::SaveImageRequest& request,
                 ros_visnav_slam::SaveImageResponse& response)
{
    //
    // extract the directory and base name from the path
    //
    ROS_INFO_STREAM("saved file folder: " << request.saved_file_path);
    std::string folderStr = request.saved_file_path;
    const char *folderPtr = folderStr.c_str();
    //
    // if file path starts with '~', strip off and prepend home path
    //
    struct passwd *pw = getpwuid(getuid());
    char *homeDir = pw->pw_dir;
    ROS_INFO_STREAM("home dir: " << homeDir);
    const char *tilde = "~";
    char *filePath = nullptr;
    char firstChar = folderPtr[0];
    ROS_INFO_STREAM("first char: " << firstChar);
    ROS_INFO_STREAM("strncmp result: " << strncmp(tilde, &firstChar, 1));
    if (0 == strncmp(tilde, &firstChar, 1)) {
        ROS_INFO_STREAM("starts with ~");
        //
        // split string and re-assemble
        //
        folderPtr = &folderPtr[1];
        ROS_INFO_STREAM("~ removed: " << folderPtr);
        filePath = strcat(homeDir, folderPtr);
    }
    else {
        filePath = (char *)folderStr.c_str();
    }
    ROS_INFO_STREAM("final path: " << filePath);
    char *folderCopy = strdup(filePath);
    char *fileName = basename(folderCopy);
    ROS_INFO_STREAM("file name:" << fileName);
    char *folderName = dirname(folderCopy);
    ROS_INFO_STREAM("folder: " << folderName);
    //
    // check file extension
    //
    std::string fileExt = ".jpg";
    std::string nameStr(fileName);
    size_t pos = nameStr.find_first_of(".");
    std::string nameExt = nameStr.substr(pos);
    if (0 != strcmp(fileExt.c_str(), nameExt.c_str())) {
        ROS_INFO_STREAM("usage:  please use file extension .jpg...");
    }
    //
    // create the directory if it does not exist
    //
    struct stat statBuff;
    int folderFound = stat(folderName, &statBuff);
    if (folderFound == -1) {
        ROS_INFO_STREAM("directory does not exist, creating...");
        int dirCreated = mkdir(folderName, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (dirCreated == -1) {
            ROS_INFO_STREAM("mkdir failed...");
        }
    }
    else {
        ROS_INFO_STREAM("folder found...");
    }
    cv::Mat saveFrame(cv::Size(imageWidth, imageHeight),CV_8UC3);
    cv::cvtColor(frame, saveFrame, cv::COLOR_BayerBG2RGB);
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(95);
    ROS_INFO_STREAM("saving frame...");
    cv::imwrite(filePath,
                saveFrame,
                compression_params);
    response.result = true;

    return true;
}

#endif // CAMERA_HEADER_HPP
