
#include "flow.h"




cv::Mat OpticalFlow::colorizeFlow()
{
    cv::Mat cflow(this->size_x, this->size_y, CV_8UC3, cv::Scalar::all(0) );
    cv::Mat flwx (this->size_x, this->size_y, CV_32F,  cv::Scalar::all(0) );
    cv::Mat flwy (this->size_x, this->size_y, CV_32F,  cv::Scalar::all(0) );

    cv::Mat planes[] = {flwx, flwy};
    split(this->flowxy, planes);
    flwx = planes[0]; flwy = planes[1];

    double uMin, uMax;
    minMaxLoc(flwx, &uMin, &uMax, 0, 0);
    double vMin, vMax;
    minMaxLoc(flwy, &vMin, &vMax, 0, 0);
    uMin = ::abs(uMin); uMax = ::abs(uMax);
    vMin = ::abs(vMin); vMax = ::abs(vMax);
    float dMax = static_cast<float>(::cv::max(::cv::max(uMin, uMax), ::cv::max(vMin, vMax)));

    for (int y = 0; y < flwx.rows; ++y) {
        for (int x = 0; x < flwx.cols; ++x) {
            cflow.at<uchar>(y, 3 * x) = 0;
            cflow.at<uchar>(y, 3 * x + 1) = (uchar)mapVal(-flwy.at<float>(y, x), -dMax, dMax, 0.f, 255.f);
            cflow.at<uchar>(y, 3 * x + 2) = (uchar)mapVal( flwx.at<float>(y, x), -dMax, dMax, 0.f, 255.f);
        }
    }

    return cflow;
};



cv::Mat OpticalFlow::getOptFlowMap(int step, const cv::Scalar& color)
{
    cv::Mat cflow;
    cvtColor(this->prevgray, cflow, cv::COLOR_GRAY2BGR);

    for(int y = 0; y < this->flowxy.rows; y += step) {
        for(int x = 0; x < this->flowxy.cols; x += step) {
            const cv::Point2f& fxy = this->flowxy.at<cv::Point2f>(y, x);
            line(cflow, cv::Point(x,y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
            circle(cflow, cv::Point(x, y), 1, color, -1);
        }
    }

    this->s.str("");
    this->s << "opt. flow FPS: " << cvRound((cv::getTickFrequency()/(this->flow_time_1-this->flow_time_0))) << ".    "
            << cvRound((this->flow_time_1-this->flow_time_0)/cv::getTickFrequency() * 1000 ) << " ms.";
    putText(cflow, this->s.str(), cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,255), 1.5);

    this->s.str("");
    this->s << "total FPS: "     << cvRound((cv::getTickFrequency()/(this->total_time_1-this->total_time_0))) << ".    "
            << cvRound((this->total_time_1-this->total_time_0)/cv::getTickFrequency() * 1000 ) << " ms.";
    putText(cflow, this->s.str(), cv::Point(5, 50), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,255), 1.5);

    return cflow;
};



void OpticalFlow::renew_vectors()
{
    int64 i = 0;
    for(int y = 0; y < this->flowxy.rows; y += this->step) {
        for(int x = 0; x < this->flowxy.cols; x += this->step) {
            const cv::Point2f& fxy = this->flowxy.at<cv::Point2f>(y, x);
            this->flowp1.at(i).x = x;
            this->flowp1.at(i).y = y;
            this->flowp2.at(i).x = x + fxy.x;
            this->flowp2.at(i).y = y + fxy.y;
            ++i;
        }
    }
};



OpticalFlow::OpticalFlow (int video_source, int sx, int sy) : cap(video_source),
        flowxy(sx, sy, CV_32F, cv::Scalar::all(0) )
{
    if( !this->cap.isOpened() ) {
        ROS_ERROR("No optical flow camera found!");
        exit(-1);
    }

    this->total_time_0 = 0;
    this->total_time_1 = 1;
    this->flow_time_0  = 0;
    this->flow_time_1  = 0;
    this->dt = 1;
    this->size_x = sx;
    this->size_y = sy;
    this->step   = 1;


    this->numpoints = this->size_x * this->size_y / (this->step * this->step);
    if (this->numpoints < 3) {
        ROS_WARN("The number of translation points is very small: %ld", this->numpoints);
    }

    for (int64 i = 0; i < this->numpoints; ++i) {
        this->flowp1.push_back(cv::Point2f(0.0, 0.0));
        this->flowp2.push_back(cv::Point2f(0.0, 0.0));
    }
};



void OpticalFlow::renew()
{
    int64 t = cv::getTickCount();

    this->cap >> this->frame;
    resize(this->frame, this->downsampled, cv::Size(this->size_x, this->size_y), 0, 0, cv::INTER_LINEAR);
    cvtColor(downsampled, gray, cv::COLOR_BGR2GRAY);


    if (prevgray.data) {
        this->flow_time_0 = cv::getTickCount();
        cv::calcOpticalFlowFarneback(this->prevgray, this->gray, this->flowxy, pyrScale, numLevels, winSize,
                                                                               numIters, polyN, polySigma, flags);
        this->flow_time_1 = cv::getTickCount();


        this->renew_vectors();
        this->dt = (this->total_time_1 - this->total_time_0) / cv::getTickFrequency();
    }


    std::swap(prevgray, gray);

    this->total_time_0 = t;
    this->total_time_1 = cv::getTickCount();
};
