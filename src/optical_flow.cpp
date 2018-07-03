#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

static cv_bridge::CvImagePtr cv_ptr;
static cv::Mat frame;
static const char *window_name = "Optical Flow";

//
// subscribe to the image_raw topic to be notified when a new image
// is available.  Assign the image pointer to the Mat.
//
void getImage(const sensor_msgs::Image raw_image)
{
    //
    // convert raw image to Mat data type using cv bridge
    //
    try {
        cv_ptr = cv_bridge::toCvCopy(raw_image,
                                     sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    frame = cv_ptr->image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optical_flow");
    ros::NodeHandle nh;
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = 10;
    //
    // instantiate the subscriber for rest messages
    //
    ros::Subscriber image_sub = nh.subscribe("image_raw", 1000, getImage);
    //
    // loop while processing images and tracking keypoint motion
    // create overlay to depict motion and show image with overlay
    //
    int count = 0;
    ROS_INFO_STREAM("hit <Esc> to close image window...");
    bool featuresInitialized = false;
    cv::Mat firstImage;
    cv::Mat secondImage;
    cv::Mat firstGrayImg;
    cv::Mat secondGrayImg;
    std::vector<cv::Point2f> firstCorners;
    const int maxCorners = 500;
    const double qualityLevel = .01;
    const double minDistance = 5.0;
    const int blockSize = 3;
    bool useHarris = true;
    double harrisFreeParameter = .04;
    while(ros::ok()) {
        //
        // capture first frame and find the good features to track
        // which will be used with successive images during tracking
        //
        if (!featuresInitialized) {
            if(frame.rows > 0 && frame.cols > 0) {
                frame.copyTo(firstImage);
                //
                // convert to grayscale
                //
                cv::cvtColor(firstImage, firstGrayImg, cv::COLOR_BGR2GRAY);
                //
                // call good features to track to find corners
                //
                cv::goodFeaturesToTrack(firstGrayImg,
                                        firstCorners,
                                        maxCorners,
                                        qualityLevel,
                                        minDistance,
                                        cv::noArray(),
                                        blockSize,
                                        useHarris,
                                        harrisFreeParameter
                                        );
                //
                // refine pixel locations to subpixel accuracy
                //
                // set the half side length of the search window to 10
                // for a 20 x 20 search window
                //
                const int winSize = 10;
                const int maxCount = 20;
                double epsilon = .03;
                cv::TermCriteria termCrit = cv::TermCriteria(
                        cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                            maxCount,
                            epsilon);
                cv::cornerSubPix(firstGrayImg,
                                 firstCorners,
                                 cv::Size(winSize, winSize),
                                 cv::Size(-1, -1),
                                 termCrit
                                 );
                //
                // done initializing, set the flag to true
                //
                ROS_INFO_STREAM("initial features found...");
                featuresInitialized = true;
            }
        }
        else if(count == 100) {
            if(frame.rows > 0 && frame.cols > 0) {
                //
                // capture second frame and estimate the camera motion
                // using pyramid Lucas-Kanade
                //
                frame.copyTo(secondImage);
                //
                // convert to grayscale
                //
                cv::cvtColor(secondImage, secondGrayImg, cv::COLOR_BGR2GRAY);
                //
            }
        }

/*
        Mat gray, prevGray, image, frame;
        vector<Point2f> points[2];


        cap >> frame;
        if( frame.empty() )
            break;

        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);

        if( nightMode )
            image = Scalar::all(0);

        if( needToInit )
        {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
        }
        else if( !points[0].empty() )
        {
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                    3, termcrit, 0, 0.001);
            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                if( addRemovePt )
                {
                    if( norm(point - points[1][i]) <= 5 )
                    {
                        addRemovePt = false;
                        continue;
                    }
                }

                if( !status[i] )
                    continue;

                points[1][k++] = points[1][i];
                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
            }
            points[1].resize(k);
        }

        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
        {
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }

        needToInit = false;
        imshow("LK Demo", image);

        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch( c )
        {
        case 'r':
            needToInit = true;
            break;
        case 'c':
            points[0].clear();
            points[1].clear();
            break;
        case 'n':
            nightMode = !nightMode;
            break;
        }

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
*/

        //
        // display the live image
        //
        cv::namedWindow( window_name, cv::WINDOW_NORMAL || cv::WINDOW_KEEPRATIO);
        if (frame.rows > 0 && frame.cols) {
            cv::imshow(window_name, frame);
        }
        char c = (char)(cv::waitKey(10));
        if (c == 27) {
            break;
        }
        ++count;
        ros::spinOnce();
        loop_rate.sleep();
    }
    cv::destroyAllWindows();
}
