#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <math.h> 

using namespace std;
using namespace cv;

int main()
{
	cv::VideoCapture in_video;
    in_video.open(camera_id);

    while (in_video.grab() && ros::ok()) 
    {
        in_video.retrieve(image);
        image.copyTo(image_copy);

        //Read image
        Mat im = cv2.imread(image_copy);

        //Set up detector wit default parameters
        SimpleBlobDetector detector;

        //Detect blobs
        std::vector<KeyPoint> keypoints;
        detector.detect(im, keypoints);

    }
}