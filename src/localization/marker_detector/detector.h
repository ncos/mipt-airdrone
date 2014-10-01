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
//#include <opencv2/nonfree/nonfree.hpp>



#include <iostream>
#include <stdio.h>



using namespace std;
using namespace cv;


void callback(const sensor_msgs::ImageConstPtr& msg);
int detect( Mat orig_frame );
bool checkWhiteInEllipse(const RotatedRect ellipse, Mat threshold_output);
void capture_image ();


#endif
