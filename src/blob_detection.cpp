#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <math.h> 

//https://stackoverflow.com/questions/8076889/how-to-use-opencv-simpleblobdetector

using namespace std;
using namespace cv;

namespace {
const char* about = "Pose estimation of aruco markers\n";
const char* keys  =
        "{ci       |       | Id of the camera as it appears in /dev/video* }";
}

int main(int argc, char *argv[])
{
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 2) {
        parser.printMessage();
        return 0;
    }

    ros::init(argc, argv, "blob_detection");
    ros::NodeHandle n;
    static tf::TransformBroadcaster br;

    int wait_time = 10;
    //ROS_DEBUG("hello");

	cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    
    int camera_id = parser.get<int>("ci");
    
    VideoCapture in_video;
    in_video.open(camera_id);
    
    //ROS_DEBUG("ready");

    while (in_video.grab() && ros::ok()) 
    {
        //ROS_DEBUG("ok");

        in_video.retrieve(image);
        // //cap >> image;
        image.copyTo(image_copy);

        // Setup SimpleBlobDetector parameters.
        SimpleBlobDetector::Params params;

        // // Change thresholds
        params.minThreshold =  50;
        params.maxThreshold = 80;

        //Filter by area
        params.filterByArea = true;
        //params.minArea = 250;
        params.minArea = 450;

        //Filter by inertia
        // params.filterByInertia = true;
        // params.minInertiaRatio = 0.1;

        // // Filter by Circularity
        params.filterByCircularity = true;
        //params.minCircularity = 0.3;
        params.minCircularity = 0.2;

        params.blobColor = 255;

        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        // //SimpleBlobDetector detector(params);

        // // Storage for blobs
        std::vector<KeyPoint> keypoints;

        // // Draw detected blobs as red circles
        // // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures
        // // the size of the circle corresponds to the size of blob

        Mat segmented_image_green;
        Mat segmented_image_blue;
        Mat segmented_image_green_filtered;
        Mat segmented_image_blue_filtered;
        Mat segmented_image;
        Mat segmented_image_filtered;
        Mat im_with_keypoints;
        Mat edges;
        
        vector<vector<Point> > contours_green;
        vector<vector<Point> > contours_blue;
        vector<Vec4i> hierarchy;

        int largest_area_green=0;
        int largest_contour_index_green=0;
        int largest_area_blue=0;
        int largest_contour_index_blue=0;

        Scalar color_green = Scalar(0,255,0); // B G R values
        Scalar color_blue = Scalar(255,0,0);

        //GREEN
        inRange(image_copy, Scalar(30,81,40), Scalar(75,181,140), segmented_image_green); //Green cube //B,G,R
        //inRange(image_copy, Scalar(41,109,83), Scalar(75,181,140), segmented_image_green); //Green cube //B,G,R
        erode(segmented_image_green, segmented_image_green_filtered, Mat(), Point(-1, -1), 2, 1 , 1);
        findContours( segmented_image_green_filtered, contours_green, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        Mat drawing(segmented_image_green_filtered.size(), CV_8UC3, Scalar(255,255,255));
        //Find biggest contour

        for( int i = 0; i< contours_green.size(); i++ ) // iterate through each contour. 
        {
            double a=contourArea( contours_green[i],false);  //  Find the area of contour
            if(a>largest_area_green)
            {
                largest_area_green=a;
                largest_contour_index_green=i;      
                //bounding_rect=boundingRect(contours_green[i]); // Find the bounding rectangle for biggest contour
            }
  
        }

        // vector <Moments> mu_green(contours_green.size());
        // for( int i = 0; i< contours_green.size(); i++ ) // iterate through each contour. 
        // {
        //     mu_green[i] = moments( contours_green[i], false ); 
        // }
        
        Moments mu_green;

        if (!contours_green.empty() && largest_area_green > 200)
        {
            //vector<Point> a = contours_green[largest_contour_index_green];
            //cout << a;
           mu_green = moments(contours_green[largest_contour_index_green], false );
           Point2f mc_green;
            mc_green = Point2f( mu_green.m10/mu_green.m00 , mu_green.m01/mu_green.m00 ); 
            drawContours(drawing, contours_green, largest_contour_index_green, color_green, 2, 8, hierarchy, 0, Point());
            circle( drawing, mc_green, 4, color_green, -1, 8, 0 );
            static tf::TransformBroadcaster brGreenBlob;
            tf::Transform transformGreenBlob;
            transformGreenBlob.setOrigin( tf::Vector3(mc_green.x, mc_green.y, -0.34)); //positions recuperees, 0.45 = z marker
            //double alpha = sqrt(pow(rvecs[i][0], 2) + pow(rvecs[i][1], 2) + pow(rvecs[i][2], 2));
            transformGreenBlob.setRotation( tf::Quaternion(0, 0, 0, 1));
            brGreenBlob.sendTransform( tf::StampedTransform(transformGreenBlob, ros::Time::now(), "camera/" + to_string(camera_id), "green" ));
        }
        
         // get the centroid of figures.
        //vector<Point2f> mc_green(contours_green.size());
        
        //Point2f center;

        // for( int i = 0; i< contours_green.size(); i++ ) // iterate through each contour. 
        // {
        //     mc_green[i] = Point2f( mu_green[i].m10/mu_green[i].m00 , mu_green[i].m01/mu_green[i].m00 ); 
        // }        

        

        //BLUE

        inRange(image_copy, Scalar(60,20,6), Scalar(180,100,70), segmented_image_blue); //Blue cube
        erode(segmented_image_blue, segmented_image_blue_filtered, Mat(), Point(-1, -1), 2, 1 , 1);
        findContours( segmented_image_blue_filtered, contours_blue, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        
        for( int i = 0; i< contours_blue.size(); i++ ) // iterate through each contour. 
        {
            double a=contourArea( contours_blue[i],false);  //  Find the area of contour
            if(a>largest_area_blue)
            {
                largest_area_blue=a;
                largest_contour_index_blue=i;                //Store the index of largest contour
                //bounding_rect=boundingRect(contours_blue[i]); // Find the bounding rectangle for biggest contour
            }
  
        }

        // // vector <Moments> mu_blue(contours_blue.size());
        // // for( int i = 0; i< contours_blue.size(); i++ ) // iterate through each contour. 
        // // {
        // //     mu_blue[i] = moments( contours_blue[i], false ); 
        // // }
        if (!contours_blue.empty() && largest_area_blue > 200)
        {
            Moments mu_blue;
            mu_blue = moments(contours_blue[largest_contour_index_blue], false );
            Point2f mc_blue;
            mc_blue = Point2f( mu_blue.m10/mu_blue.m00 , mu_blue.m01/mu_blue.m00 );
            drawContours(drawing, contours_blue, largest_contour_index_blue, color_blue, 2, 8, hierarchy, 0, Point());
            circle( drawing, mc_blue, 4, color_blue, -1, 8, 0 );
            static tf::TransformBroadcaster brBlueBlob;
            tf::Transform transformBlueBlob;
            transformBlueBlob.setOrigin( tf::Vector3(mc_blue.x, mc_blue.y, -0.34) );
            //double alpha = sqrt(pow(rvecs[i][0], 2) + pow(rvecs[i][1], 2) + pow(rvecs[i][2], 2));
            transformBlueBlob.setRotation( tf::Quaternion(0, 0, 0, 1));
            brBlueBlob.sendTransform( tf::StampedTransform(transformBlueBlob, ros::Time::now(), "camera/" + to_string(camera_id), "blue"));
        }
        
        //  // get the centroid of figures.
        // vector<Point2f> mc_blue;
        // //Point2f center;
        
        // for( int i = 0; i< contours_blue.size(); i++ ) // iterate through each contour. 
        // {
        //     mc_blue[i] = Point2f( mu_blue[i].m10/mu_blue[i].m00 , mu_blue[i].m01/mu_blue[i].m00 ); 
        // }

         // B G R values
        cout << "green";
        cout << largest_area_green;
        cout << "blue";
        cout << largest_area_blue;
        namedWindow("Contours", CV_WINDOW_AUTOSIZE);
        imshow("Contours", drawing);

        // // cout << "green";
        // // cout << "mc_green=";
        // // cout << mc_green;
        // // cout << "x=";
        // // cout << mc_green.x;
        // // cout << "blue";
        // // cout << mc_blue;


        char key = (char) cv::waitKey(wait_time);
        if (key == 27)
            break;
    }

    in_video.release();
    return 0;
}
