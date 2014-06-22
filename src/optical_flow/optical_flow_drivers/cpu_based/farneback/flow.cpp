
#include "flow.h"



void OpticalFlow::drawOptFlowMap(const cv::Mat& flowx, const cv::Mat& flowy, cv::Mat& out_flow, int step, const cv::Scalar& color)
{
    for (int y = 0; y < out_flow.rows; y += step) {
        for(int x = 0; x < out_flow.cols; x += step) {
        	cv::Point2f fxy;
            fxy.x = flowx.at<float>(y, x);
            fxy.y = flowy.at<float>(y, x);
            cv::line (out_flow, cv::Point(x, y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
            cv::circle (out_flow, cv::Point(x, y), 2, color, -1);
        }
    }
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






OpticalFlow::OpticalFlow (int video_source) : cap(video_source){
    if( !this->cap.isOpened() ) {
        ROS_ERROR("No optical flow camera found!");
        exit(-1);
    }

    this->t0 = 0;
    this->t1 = 1;
};

void OpticalFlow::renew()
{
    int64 t = cv::getTickCount();

    this->cap >> this->frame;
    resize(this->frame, this->downsampled, cv::Size(), resize_scale, resize_scale, cv::INTER_LINEAR);
    cvtColor(downsampled, gray, cv::COLOR_BGR2GRAY);


    if (prevgray.data) {
        double tc0 = cv::getTickCount();
        cv::calcOpticalFlowFarneback(prevgray, gray, flowxy, pyrScale, numLevels, winSize,
                                                             numIters, polyN, polySigma, flags);
        double tc1 = cv::getTickCount();

        cv::Mat planes[] = {flowx, flowy};
        split(flowxy, planes);
        flowx = planes[0]; flowy = planes[1];



        cvtColor(this->prevgray, this->cflow, cv::COLOR_GRAY2BGR);
        drawOptFlowMap(flowx, flowy, this->cflow, 16, cv::Scalar(0, 255, 255));


        s.str("");
        s << "opt. flow FPS: " << cvRound((cv::getTickFrequency()/(tc1-tc0))) << ".    " << cvRound(    (tc1-tc0)/cv::getTickFrequency() * 1000 ) << " ms.";
        putText(cflow, s.str(), cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,255), 1.5);

        s.str("");
        s << "total FPS: " << cvRound((cv::getTickFrequency()/(t1-t0))) << ".    " << cvRound(    (t1-t0)/cv::getTickFrequency() * 1000 ) << " ms.";
        putText(cflow, s.str(), cv::Point(5, 50), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,255), 1.5);


        std::vector<cv::Point2f> start_p, end_p;
        get_vectors(flowx, flowy, start_p, end_p, 1);

        cv::Mat R = cv::estimateRigidTransform(start_p, end_p, false);

        // extend rigid transformation to use perspectiveTransform:
        cv::Mat H = cv::Mat(3, 3, R.type());

        H.at<double>(0,0) = R.at<double>(0,0);
        H.at<double>(0,1) = R.at<double>(0,1);
        H.at<double>(0,2) = R.at<double>(0,2);

        H.at<double>(1,0) = R.at<double>(1,0);
        H.at<double>(1,1) = R.at<double>(1,1);
        H.at<double>(1,2) = R.at<double>(1,2);

        H.at<double>(2,0) = 0.0;
        H.at<double>(2,1) = 0.0;
        H.at<double>(2,2) = 1.0;

        // compute perspectiveTransform on p1
        std::vector<cv::Point2f> to_transform;
        to_transform.push_back(cv::Point2f(0, 1));
        std::vector<cv::Point2f> result;
        cv::perspectiveTransform(to_transform, result, H);

        //ROS_INFO("(%f, %f)", result.at(0).x, result.at(0).y);
    }


    std::swap(prevgray, gray);

    this->t0 = t;
    this->t1 = cv::getTickCount();
};
