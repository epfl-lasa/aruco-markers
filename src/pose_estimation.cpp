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

namespace {
const char* about = "Pose estimation of aruco markers\n";
const char* keys  =
        "{l        |       | Real length of the aruco markers (in m) }"
        "{ci       |       | Id of the camera as it appears in /dev/video* }";
}

int main(int argc, char *argv[])
{
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 3) {
        parser.printMessage();
        return 0;
    }

    ros::init(argc, argv, "marker_poses_publisher");
    ros::NodeHandle n;
    static tf::TransformBroadcaster br;

    int wait_time = 10;
    float actual_marker_length = parser.get<float>("l");
    int camera_id = parser.get<int>("ci");

    cv::Mat image, image_copy, image2, image_copy2;
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;
    
    cv::VideoCapture in_video;
    in_video.open(camera_id);
    cv::Ptr<cv::aruco::Dictionary> dictionary = 
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    
    std::string path = ros::package::getPath("aruco_markers") + "/config/calibration_params.yml";
    cv::FileStorage fs(path, cv::FileStorage::READ);

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    while (in_video.grab() && ros::ok()) 
    {

        in_video.retrieve(image);
        image.copyTo(image_copy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        
        // if at least one marker detected
        if (ids.size() > 0)
        {

            cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_length,
                    camera_matrix, dist_coeffs, rvecs, tvecs);
            // draw axis for each marker
            for(int i=0; i < ids.size(); i++)
            {
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], 0.1);

                tf::Transform transform;
                transform.setOrigin( tf::Vector3(tvecs[i][0], tvecs[i][1], tvecs[i][2]) );
                double alpha = sqrt(pow(rvecs[i][0], 2) + pow(rvecs[i][1], 2) + pow(rvecs[i][2], 2));
                transform.setRotation( tf::Quaternion(rvecs[i][0] / alpha * sin(alpha/2), rvecs[i][1] / alpha * sin(alpha/2), rvecs[i][2] / alpha * sin(alpha/2), cos(alpha/2)) );
                br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "camera/" + to_string(camera_id), "marker/" + to_string(ids[i])) );

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4) 
                                 << "x: " << std::setw(8)<<  tvecs[i](0);
                cv::putText(image_copy, vector_to_marker.str(), 
                        cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                        cvScalar(0, 252, 124), 1, CV_AA);
                
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4) 
                                 << "y: " << std::setw(8) << tvecs[i](1); 
                cv::putText(image_copy, vector_to_marker.str(), 
                        cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                        cvScalar(0, 252, 124), 1, CV_AA);
                
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4) 
                                 << "z: " << std::setw(8) << tvecs[i](2);
                cv::putText(image_copy, vector_to_marker.str(), 
                        cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                        cvScalar(0, 252, 124), 1, CV_AA);
                
            }
        }

        cv::imshow("Pose estimation", image_copy);
        
        //BLOB DETECTION
        SimpleBlobDetector::Params params;

        // // Change thresholds
        params.minThreshold =  50;
        params.maxThreshold = 80;

        //Filter by area
        params.filterByArea = true;
        params.minArea = 450;

        // // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.2;

        params.blobColor = 255;

        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        Mat image_copy_bis;
        Mat segmented_image_green;
        Mat segmented_image_blue;
        Mat segmented_image_green_filtered;
        Mat segmented_image_blue_filtered;
        
        vector<vector<Point> > contours_green;
        vector<vector<Point> > contours_blue;
        vector<Vec4i> hierarchy;

        int largest_area_green=0;
        int largest_contour_index_green=0;
        int largest_area_blue=0;
        int largest_contour_index_blue=0;

        Scalar color_green = Scalar(0,255,0); // B G R values
        Scalar color_blue = Scalar(255,0,0);

        image.copyTo(image_copy_bis);
        //Green
        inRange(image_copy_bis, Scalar(30,81,40), Scalar(75,181,140), segmented_image_green); //Green cube //B,G,R
        // inRange(image_copy_bis, Scalar(50,90,70), Scalar(75,120,90), segmented_image_green); 
        findContours( segmented_image_green, contours_green, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        Mat drawing(segmented_image_green.size(), CV_8UC3, Scalar(255,255,255));

        //Find biggest contour
        for( int i = 0; i< contours_green.size(); i++ ) // iterate through each contour. 
        {
            double a=contourArea( contours_green[i],false);  //  Find the area of contour
            if(a>largest_area_green)
            {
                largest_area_green=a;
                largest_contour_index_green=i;      
            }
        }
        
        Moments mu_green;

        if (!contours_green.empty() && largest_area_green >= 1400 && largest_area_green < 3300)
        {
      
            mu_green = moments(contours_green[largest_contour_index_green], false );
            Point2f mc_green;
            mc_green = Point2f( mu_green.m10/mu_green.m00 , mu_green.m01/mu_green.m00 ); 
            drawContours(drawing, contours_green, largest_contour_index_green, color_green, 2, 8, hierarchy, 0, Point());
            circle( drawing, mc_green, 4, color_green, -1, 8, 0 );
            static tf::TransformBroadcaster brGreenBlob;
            tf::Transform transformGreenBlob;
            transformGreenBlob.setOrigin( tf::Vector3(mc_green.x, mc_green.y, 0.44)); //positions recuperees, 0.45 = z marker
            transformGreenBlob.setRotation( tf::Quaternion(0, 0, 0, 1));
            brGreenBlob.sendTransform( tf::StampedTransform(transformGreenBlob, ros::Time::now(), "camera/" + to_string(camera_id), "green" ));
        }

        //Blue
        inRange(image_copy_bis, Scalar(60,20,6), Scalar(180,100,70), segmented_image_blue); //Blue cube
        // inRange(image_copy_bis, Scalar(90,80,50), Scalar(135,105,80), segmented_image_blue); 
        findContours( segmented_image_blue, contours_blue, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
        
        for( int i = 0; i< contours_blue.size(); i++ ) // iterate through each contour. 
        {
            double a=contourArea( contours_blue[i],false);  //  Find the area of contour
            if(a>largest_area_blue)
            {
                largest_area_blue=a;
                largest_contour_index_blue=i;                //Store the index of largest contour
            }
  
        }

        if (!contours_blue.empty() && largest_area_blue >= 1400 && largest_area_blue <= 3300)
        {
            Moments mu_blue;
            mu_blue = moments(contours_blue[largest_contour_index_blue], false );
            Point2f mc_blue;
            mc_blue = Point2f( mu_blue.m10/mu_blue.m00 , mu_blue.m01/mu_blue.m00 );
            drawContours(drawing, contours_blue, largest_contour_index_blue, color_blue, 2, 8, hierarchy, 0, Point());
            circle( drawing, mc_blue, 4, color_blue, -1, 8, 0 );
            static tf::TransformBroadcaster brBlueBlob;
            tf::Transform transformBlueBlob;
            transformBlueBlob.setOrigin( tf::Vector3(mc_blue.x, mc_blue.y, 0.44) );
            transformBlueBlob.setRotation( tf::Quaternion(0, 0, 0, 1));
            brBlueBlob.sendTransform( tf::StampedTransform(transformBlueBlob, ros::Time::now(), "camera/" + to_string(camera_id), "blue"));
        }

        
        namedWindow("Contours", CV_WINDOW_AUTOSIZE);
        imshow("Contours", drawing);

        char key = (char) cv::waitKey(wait_time);
        if (key == 27)
            break;
    }

    in_video.release();
}
