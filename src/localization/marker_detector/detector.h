#ifndef DETECTOR_H
#define DETECTOR_H


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <sensor_msgs/CameraInfo.h>
//#include <opencv2/nonfree/nonfree.hpp>



#include <iostream>
#include <stdio.h>



using namespace std;
using namespace cv;

void callback(const sensor_msgs::ImageConstPtr& msg);
Point2f detect( Mat orig_frame );
bool checkWhiteInEllipse(const RotatedRect ellipse, Mat threshold_output);
void capture_image ();
geometry_msgs::Point32 estimatePose(Point2f frame_target);
void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg);

class PointEstimator {
public:
    tf::TransformListener tf_listener;
    PointEstimator();
};

class DownwardCamera {
public:
    double fx, fy;
    double cx, cy;
    double height, width;
    DownwardCamera(): height(0), width(0), fx(0), fy(0), cx(0), cy(0) {};
    void init(double _height, double _width,
              double _fx, double _fy,
              double _cx, double _cy);
};

#endif
