
#include "flow.h"



using namespace cv::gpu;




void drawOptFlowMap(const cv::Mat& flowx, const cv::Mat& flowy, cv::Mat& out_flow, int step, const cv::Scalar& color)
{
	if ((flowy.rows != flowx.rows) || (flowy.cols != flowx.cols)) {
		ROS_ERROR("Unequal flowx and flowy sizes!");
		exit(-1);
	}

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


void get_vectors(const cv::Mat& flowx, const cv::Mat& flowy, std::vector<cv::Point2f> &p0, std::vector<cv::Point2f> &p1, int step)
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


int flow(int argc, char **argv)
{
    DeviceInfo info = getDevice();
    ROS_INFO("%s", info.name().c_str());
    ROS_INFO("%s\n", cv::getBuildInformation().c_str());
    ROS_INFO("Using OpenCV %s", CV_VERSION);

    cv::VideoCapture cap(0);
    if( !cap.isOpened() ) {
    	ROS_ERROR("No optical flow camera found!");
        return -1;
    }

    cv::namedWindow("flow", 1);


    GpuMat d_frameL, d_frameR, d_flowx, d_flowy;
    cv::Mat    prevgray, gray,     flowx,   flowy, flowxy,  frame, cflow, downsampled;
    FarnebackOpticalFlow d_calc;


    double t0 = 0, t1 = 1;
    std::stringstream s;

    for (;;)
    {
        int64 t = cv::getTickCount();

        double tc2 = cv::getTickCount();
        cap >> frame;
        resize(frame, downsampled, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

        cvtColor(downsampled, gray, cv::COLOR_BGR2GRAY);
        double tc3 = cv::getTickCount();



        if (prevgray.data)
        {

        	/*
            double tc0 = cv::getTickCount();
            d_frameL.upload(prevgray);
            d_frameR.upload(gray);
            d_calc(d_frameL, d_frameR, d_flowx, d_flowy);
            d_flowx.download(flowx);
            d_flowy.download(flowy);
            double tc1 = cv::getTickCount();
            */
        	double tc0 = cv::getTickCount();
            cv::calcOpticalFlowFarneback(
            		prevgray, gray, flowxy, d_calc.pyrScale, d_calc.numLevels, d_calc.winSize,
                    d_calc.numIters, d_calc.polyN, d_calc.polySigma, d_calc.flags);
            double tc1 = cv::getTickCount();

            cv::Mat planes[] = {flowx, flowy};
            split(flowxy, planes);
            flowx = planes[0]; flowy = planes[1];



            cvtColor(prevgray, cflow, cv::COLOR_GRAY2BGR);
            drawOptFlowMap(flowx, flowy, cflow, 16, cv::Scalar(0, 255, 255));


            s.str("");
            s << "opt. flow FPS: " << cvRound((cv::getTickFrequency()/(tc1-tc0))) << ".    " << cvRound(    (tc1-tc0)/cv::getTickFrequency() * 1000 ) << " ms.";
            putText(cflow, s.str(), cv::Point(5, 65), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255,0,255), 1);

            s.str("");
            s << "total FPS: " << cvRound((cv::getTickFrequency()/(t1-t0))) << ".    " << cvRound(    (t1-t0)/cv::getTickFrequency() * 1000 ) << " ms.";
            putText(cflow, s.str(), cv::Point(5, 75), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255,0,255), 1);

            s.str("");
            s << "Test flag time: " << cvRound(((tc3-tc2)/cv::getTickFrequency()*1000)) << " ms.";
            putText(cflow, s.str(), cv::Point(5, 85), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255,0,255), 1);

            imshow("flow", cflow);


            std::vector<cv::Point2f> start_p, end_p;
            get_vectors(flowx, flowy, start_p, end_p, 1);
            //ROS_INFO("%lu", start_p.size());


            cv::Mat R = cv::estimateRigidTransform(start_p, end_p, false);
            // extend rigid transformation to use perspectiveTransform:
            cv::Mat H = cv::Mat(3, 3, R.type());
            //ROS_INFO("%d, %d", R.cols, R.rows);


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

            ROS_INFO("(%f, %f)", result.at(0).x, result.at(0).y);
        }


        if (cv::waitKey(30) >= 0) {
            break;
        }


        std::swap(prevgray, gray);

        t0 = t;
        t1 = cv::getTickCount();
    }

    return 0;
};
