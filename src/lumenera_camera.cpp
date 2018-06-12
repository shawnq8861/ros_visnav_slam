#include <ros/ros.h>
#include <lumenera/lucamapi.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lumenera_camera");
    ros::NodeHandle nh;

    HANDLE hCamera = LucamCameraOpen(1);
    if (NULL == hCamera) {
        printf("ERROR %u: Unable to open camera.\n", (unsigned int)LucamGetLastError());
        return EXIT_FAILURE;
    }

    ROS_INFO_STREAM("Hello camera!\n");

    LucamCameraClose(hCamera);

}
