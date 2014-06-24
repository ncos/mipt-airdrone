#ifndef FLOW_H
#define FLOW_H


#include <ros/ros.h>
#include <sstream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"


extern int numLevels;
extern double pyrScale;
extern bool fastPyramids;
extern int winSize;
extern int numIters;
extern int polyN;
extern double polySigma;
extern int flags;




template <typename T>
inline T mapVal(T x, T a, T b, T c, T d)
{
    x = ::cv::max(::cv::min(x, b), a);
    return c + (d-c) * (x-a) / (b-a);
};

class OpticalFlow
{
private:
    cv::Mat prevgray, gray, downsampled;
    cv::VideoCapture cap;

    int64 flow_time_0, flow_time_1, total_time_0, total_time_1;
    std::stringstream s;
    int size_x, size_y;
    int64 numpoints, step;

public:
    std::vector<cv::Point2f> flowp1, flowp2;
    cv::Mat flowxy, frame;
    double dt;

private:
    void renew_vectors ();

public:
    cv::Mat getOptFlowMap (int step, const cv::Scalar& color);
    cv::Mat colorizeFlow ();

    OpticalFlow (int video_source, int sx, int sy);
    void renew ();
};




#endif
