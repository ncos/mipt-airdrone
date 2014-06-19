
#include "flow.h"



using namespace std;
using namespace cv;
using namespace cv::gpu;




void drawOptFlowMap(const Mat& flowx, const Mat& flowy, Mat& cflowmap, int step, const Scalar& color)
{
    for (int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            Point2f fxy;
            fxy.x = flowx.at<float>(y,x);
            fxy.y = flowy.at<float>(y,x);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }
}


int flow(int argc, char **argv)
{
    DeviceInfo info = getDevice();
    cout << info.name() << endl << endl;
    ROS_INFO("%s", info.name().c_str());
    ROS_INFO("%s\n", cv::getBuildInformation().c_str());
    ROS_INFO("Using OpenCV %s", CV_VERSION);

    VideoCapture cap(0);
    if( !cap.isOpened() ) {
    	ROS_ERROR("No optical flow camera found!");
        return -1;
    }

    namedWindow("flow", 1);


    GpuMat d_frameL, d_frameR, d_flowx, d_flowy;
    Mat    prevgray, gray,     flowx,   flowy,   frame, cflow, downsampled;
    FarnebackOpticalFlow d_calc;


    double t0 = 0, t1 = 1;
    stringstream s;

    for(;;)
    {
        int64 t = getTickCount();

        double tc2 = getTickCount();
        cap >> frame;
        resize(frame, downsampled, Size(), 0.5, 0.5, INTER_LINEAR);

        cvtColor(downsampled, gray, COLOR_BGR2GRAY);
        double tc3 = getTickCount();



        if( prevgray.data )
        {

            double tc0 = getTickCount();
            d_frameL.upload(prevgray);
            d_frameR.upload(gray);
            d_calc(d_frameL, d_frameR, d_flowx, d_flowy);
            d_flowx.download(flowx);
            d_flowy.download(flowy);
            double tc1 = getTickCount();


            cvtColor(prevgray, cflow, COLOR_GRAY2BGR);
            drawOptFlowMap(flowx, flowy, cflow, 16, Scalar(0, 255, 255));


            s.str("");
            s << "opt. flow FPS: " << cvRound((getTickFrequency()/(tc1-tc0))) << ".    " << cvRound(    (tc1-tc0)/getTickFrequency() * 1000 ) << " ms.";
            putText(cflow, s.str(), Point(5, 65), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255,0,255), 1);

            s.str("");
            s << "total FPS: " << cvRound((getTickFrequency()/(t1-t0))) << ".    " << cvRound(    (t1-t0)/getTickFrequency() * 1000 ) << " ms.";
            putText(cflow, s.str(), Point(5, 75), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255,0,255), 1);

            s.str("");
            s << "Test flag time: " << cvRound(((tc3-tc2)/getTickFrequency()*1000)) << " ms.";
            putText(cflow, s.str(), Point(5, 85), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255,0,255), 1);

            imshow("flow", cflow);

        }

        if(waitKey(30)>=0)
            break;


        std::swap(prevgray, gray);

        t0 = t;
        t1 = getTickCount();
    }

    return 0;
};
