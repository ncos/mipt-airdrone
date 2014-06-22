#ifndef DETECTOR_H
#define DETECTOR_H


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
extern double resize_scale;


class OpticalFlow
{
private:
    cv::Mat prevgray, gray, flowx, flowy;
    cv::VideoCapture cap;

    double t0, t1;
    std::stringstream s;

public:
    cv::Mat flowxy, cflow, frame, downsampled;

private:
    void drawOptFlowMap (const cv::Mat& flowx, const cv::Mat& flowy, cv::Mat& out_flow, int step, const cv::Scalar& color);
    void get_vectors (const cv::Mat& flowx, const cv::Mat& flowy, std::vector<cv::Point2f> &p0, std::vector<cv::Point2f> &p1, int step);

public:
    OpticalFlow (int video_source);
    void renew ();
};




#endif
