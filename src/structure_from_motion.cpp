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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>
#include <stdlib.h>
#include <sstream>
#define CERES_FOUND 1
#include <opencv2/sfm.hpp>
#include <opencv2/core.hpp>
#include <mutex>
#include <pwd.h>

#define CHAR_BUFFER_SIZE        100

static sensor_msgs::CameraInfo cameraInfo;
static const std::string cameraName = "lumenera_camera";
static const std::string relativePath = "/Pictures/SFM/";
static const std::string frame_id = "cloud_frame";
static std::vector<cv::String> imagePaths;
static cv::Matx33d cameraIntrinsics;
static sensor_msgs::PointCloud2 pointCloud2;
static pcl::PointCloud<pcl::PointXYZ> pointCloud;
static std::mutex point_cloud_mtx;
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
        ROS_INFO_STREAM("directory does not exist, creating...");
        int dirCreated = mkdir(calibrationFilePath,
                               S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (dirCreated == -1) {
            ROS_INFO_STREAM("mkdir failed...");
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
    // use the values from the example
    // ./example_sfm_scene_reconstruction image_paths_file.txt 350 240 360
    //
    double fx = 350;
    double fy = 350;
    double cx = 240;
    double cy = 360;
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
    //
    // perform reconstruction
    //
    cv::sfm::reconstruct(imagePaths,
                         Rs,
                         Ts,
                         K,
                         points3d,
                         is_projective);
    //
    // get the number of points found
    //
    int dataWidth = points3d.size();
    ROS_INFO_STREAM("number of points: " << dataWidth);
    pointCloud.points.resize(dataWidth);
    //
    // since this is an unorganized point cloud, set height = 1 and
    // width to size or number of points
    //
    pointCloud.height = 1;
    pointCloud.width = dataWidth;
    for (int index = 0; index < dataWidth; ++index) {
        ROS_INFO_STREAM("loop count: " << index);
        //
        // assign the points3D values to the x, y, z fields
        // in the point cloud
        //
        // get the current point, a Vec3f
        //
        cv::Vec3f vec3fPoint = cv::Vec3f(points3d[index]);
        //
        // assign the 3 values to the PCL point cloud
        //
        // then get each of the three XYZ coordinate values
        //
        pointCloud.points[index].x = vec3fPoint[0];
        pointCloud.points[index].y = vec3fPoint[1];
        pointCloud.points[index].y = vec3fPoint[2];
    }
    //
    // now, convert pcl point cloud to sensor_msgs point cloud
    //
    point_cloud_mtx.lock();
    pcl::toROSMsg(pointCloud, pointCloud2);
    point_cloud_mtx.unlock();
    //
    // set the frame id and time stamp
    //
    point_cloud_mtx.lock();
    pointCloud2.header.frame_id = frame_id;
    pointCloud2.header.stamp = ros::Time::now();
    point_cloud_mtx.unlock();
    ROS_INFO_STREAM("time stamp: " << pointCloud2.header.stamp);
    ROS_INFO_STREAM("frame_id: " << pointCloud2.header.frame_id);
    ROS_INFO_STREAM("height: " << pointCloud2.height);
    ROS_INFO_STREAM("width: " << pointCloud2.width);
    ROS_INFO_STREAM("point step: " << pointCloud2.point_step);
    ROS_INFO_STREAM("row step: " << pointCloud2.row_step);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "structure_from_motion");
    ros::NodeHandle nh;
    //
    // check the calibration file path, return if not valid
    //
    if (!checkCalibrationFilePath()) {
        return EXIT_FAILURE;
    }
    //
    // get the camera Info
    //
    if(camera_calibration_parsers::readCalibration(
                calibrationFilePath,
                (std::string&)cameraName,
                cameraInfo)) {
        ROS_INFO_STREAM("calibration file read...");
    }
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 1;
    //
    // instantiate the subscriber for reconstruct messages
    //
    ros::Subscriber reconstruct_sub = nh.subscribe("structure_from_motion/reconstruct",
                                              1000,
                                              &performReconstruction);
    //
    // instantiate a publisher for the point cloud
    //
    ros::Publisher point_cloud_pub =
            nh.advertise<sensor_msgs::PointCloud2>
            ("structure_from_motion/point_cloud_2", 10);
    point_cloud_mtx.lock();
    pointCloud2.header.frame_id = frame_id;
    point_cloud_mtx.unlock();
    //
    // loop while waiting for request to reconstruct
    //
    while(ros::ok()) {
        //
        // publish the point cloud
        //
        point_cloud_mtx.lock();
        point_cloud_pub.publish(pointCloud2);
        point_cloud_mtx.unlock();
        //
        // process callbacks and check for messages
        //
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}
