#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace cv;

namespace {
const char* about = "Pose estimation of aruco markers\n";
const char* keys  =
        "{l        |       | Real length of the aruco markers (in m) }"
        "{c        |       | Id of the camera as it appears in /dev/video* }";
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
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/camera/marker_poses", 1000);

    int wait_time = 10;
    float actual_marker_length = parser.get<int>("l");
    int camera_id = parser.get<int>("c");

    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;
    
    cv::VideoCapture in_video;
    in_video.open(camera_id);
    cv::Ptr<cv::aruco::Dictionary> dictionary = 
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    
    std::string path = ros::package::getPath("aruco_markers");
    cv::FileStorage fs(path + "config/calibration_params.yml", cv::FileStorage::READ);

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;

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
                
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4) 
                                 << "x: " << std::setw(8)<<  tvecs[0](0);
                cv::putText(image_copy, vector_to_marker.str(), 
                        cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                        cvScalar(0, 252, 124), 1, CV_AA);
                
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4) 
                                 << "y: " << std::setw(8) << tvecs[0](1); 
                cv::putText(image_copy, vector_to_marker.str(), 
                        cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                        cvScalar(0, 252, 124), 1, CV_AA);
                
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4) 
                                 << "z: " << std::setw(8) << tvecs[0](2);
                cv::putText(image_copy, vector_to_marker.str(), 
                        cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                        cvScalar(0, 252, 124), 1, CV_AA);
                
            }
        }
        
        cv::imshow("Pose estimation", image_copy);
        char key = (char) cv::waitKey(wait_time);
        if (key == 27)
            break;
    }

    in_video.release();
}
