
#include "flow.h"




cv::Mat OpticalFlow::colorizeFlow()
{
    cv::Mat cflow;

    double uMin, uMax;
    minMaxLoc(this->flowx, &uMin, &uMax, 0, 0);
    double vMin, vMax;
    minMaxLoc(this->flowy, &vMin, &vMax, 0, 0);
    uMin = ::abs(uMin); uMax = ::abs(uMax);
    vMin = ::abs(vMin); vMax = ::abs(vMax);
    float dMax = static_cast<float>(::cv::max(::cv::max(uMin, uMax), ::cv::max(vMin, vMax)));

    cflow.create(this->flowx.size(), CV_8UC3);
    for (int y = 0; y < this->flowx.rows; ++y)
    {
        for (int x = 0; x < this->flowx.cols; ++x)
        {
            cflow.at<uchar>(y,3*x) = 0;
            cflow.at<uchar>(y,3*x+1) = (uchar)mapVal(-this->flowy.at<float>(y,x), -dMax, dMax, 0.f, 255.f);
            cflow.at<uchar>(y,3*x+2) = (uchar)mapVal(this->flowx.at<float>(y,x), -dMax, dMax, 0.f, 255.f);
        }
    }

    return cflow;
};



cv::Mat OpticalFlow::getOptFlowMap(int step, const cv::Scalar& color)
{
    cv::Mat cflow;
    cvtColor(this->prevgray, cflow, cv::COLOR_GRAY2BGR);

    ROS_INFO("(%d, %d)", cflow.cols, cflow.rows);

    for (int y = 0; y < cflow.rows; y += step) {
        for(int x = 0; x < cflow.cols; x += step) {
            cv::Point2f fxy;
            fxy.x = flowx.at<float>(y, x);
            fxy.y = flowy.at<float>(y, x);
            cv::line (cflow, cv::Point(x, y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
            cv::circle (cflow, cv::Point(x, y), 2, color, -1);
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



void OpticalFlow::get_vectors(const cv::Mat& flowx, const cv::Mat& flowy, std::vector<cv::Point2f> &p0, std::vector<cv::Point2f> &p1, int step)
{
    if ((flowy.rows != flowx.rows) || (flowy.cols != flowx.cols)) {
        ROS_ERROR("Unequal flowx and flowy sizes!");
        exit(-1);
    }

    for (int y = 0; y < flowx.rows; y += step) {
        for(int x = 0; x < flowx.cols; x += step) {
            cv::Point2f fxy;
            fxy.x = flowx.at<float>(y, x);
            fxy.y = flowy.at<float>(y, x);
            p0.push_back(cv::Point2f(x, y));
            p1.push_back(cv::Point2f(x + fxy.x, y + fxy.y));
        }
    }
};



OpticalFlow::OpticalFlow (int video_source, int sx, int sy) : cap(video_source),
        flowxy(sx, sy, CV_32F, cv::Scalar::all(0) ), flowx(sx, sy, CV_32F, cv::Scalar::all(0) ),
        flowy(sx, sy, CV_32F, cv::Scalar::all(0) )
{
    if( !this->cap.isOpened() ) {
        ROS_ERROR("No optical flow camera found!");
        exit(-1);
    }

    this->total_time_0 = 0;
    this->total_time_1 = 1;
    this->flow_time_0  = 0;
    this->flow_time_1  = 0;
    this->size_x = sx;
    this->size_y = sy;

    cv::gpu::DeviceInfo info = cv::gpu::getDevice();
    ROS_INFO("%s", info.name().c_str());
    ROS_INFO("%s\n", cv::getBuildInformation().c_str());
    ROS_INFO("Using OpenCV %s", CV_VERSION);


    this->d_calc.numLevels = numLevels;
    this->d_calc.pyrScale  = pyrScale;
    this->d_calc.fastPyramids = fastPyramids;
    this->d_calc.winSize   = winSize;
    this->d_calc.numIters  = numIters;
    this->d_calc.polyN     = polyN;
    this->d_calc.polySigma = polySigma;
    this->d_calc.flags     = flags;
};



void OpticalFlow::renew()
{
    int64 t = cv::getTickCount();

    this->cap >> this->frame;
    resize(this->frame, this->downsampled, cv::Size(this->size_x, this->size_y), 0, 0, cv::INTER_LINEAR);
    cvtColor(downsampled, gray, cv::COLOR_BGR2GRAY);

    cv::gpu::GpuMat d_frameL, d_frameR, d_flowx, d_flowy;

    if (prevgray.data) {
        this->flow_time_0 = cv::getTickCount();
        d_frameL.upload(prevgray);
        d_frameR.upload(gray);
        d_calc(d_frameL, d_frameR, d_flowx, d_flowy);
        d_flowx.download(flowx);
        d_flowy.download(flowy);
        this->flow_time_1 = cv::getTickCount();

        cv::Mat planes[] = {flowx, flowy};
        split(flowxy, planes);
        flowx = planes[0]; flowy = planes[1];


        std::vector<cv::Point2f> start_p, end_p;
        get_vectors(flowx, flowy, start_p, end_p, 1);

        cv::Mat R = cv::estimateRigidTransform(start_p, end_p, false);

        // extend rigid transformation to use perspectiveTransform:
        cv::Mat H = cv::Mat(3, 3, R.type());

        H.at<double>(0, 0) = R.at<double>(0, 0);
        H.at<double>(0, 1) = R.at<double>(0, 1);
        H.at<double>(0, 2) = R.at<double>(0, 2);

        H.at<double>(1, 0) = R.at<double>(1, 0);
        H.at<double>(1, 1) = R.at<double>(1, 1);
        H.at<double>(1, 2) = R.at<double>(1, 2);

        H.at<double>(2, 0) = 0.0;
        H.at<double>(2, 1) = 0.0;
        H.at<double>(2, 2) = 1.0;

        // compute perspectiveTransform on p1
        std::vector<cv::Point2f> to_transform;
        to_transform.push_back(cv::Point2f(0, 1));
        std::vector<cv::Point2f> result;
        cv::perspectiveTransform(to_transform, result, H);

        //ROS_INFO("(%f, %f)", result.at(0).x, result.at(0).y);


    }


    std::swap(prevgray, gray);

    this->total_time_0 = t;
    this->total_time_1 = cv::getTickCount();
};
