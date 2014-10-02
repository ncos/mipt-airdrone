
#include "detector.h"

using namespace std;
using namespace cv;

/** Global variables */
//String face_cascade_name = "/home/ncos/catkin_ws/src/marker_detector/cascades/haarcascade_frontalface_alt.xml";
String face_cascade_name = "/home/vsevolod/workspace/mipt-airdrone/src/localization/marker_detector/cascades/mycrossdetector__new_20st_20x20_nosim.xml";
//String face_cascade_name = "/home/ncos/catkin_ws/src/marker_detector/cascades/mycrossdetector3.xml";

CascadeClassifier face_cascade;
string window_name = "Capture - Marker detection";
string window_name_t1 = "Capture - Marker detection -t1";
string window_name_t2 = "Capture - Marker detection -t2";

RNG rng(12345);

void callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    detect(cv_ptr->image);
    cv::imshow("view", cv_ptr->image);
    cv::waitKey(3);
    /*
    sensor_msgs::CvBridge bridge;
    try
    {
        imshow("view", bridge.imgMsgToCv(msg, "bgr8"));
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    */
}

// Determines if white pixels count within given shape on given binarized
// image exceeds 80% of total pixels count
// [in] ellipse - dimensions of found ellipse
// [in] threshold_output - image to check ellipse on
// [retval] - true if check passed
bool checkWhiteInEllipse(const RotatedRect ellipse, Mat threshold_output) {
    // total pixel count
    unsigned int count = 0;
    // white pixel count
    unsigned int countWhite = 0;
    // find region dimensions
    int h_min = ( int )( ellipse.center.y - ellipse.size.width/2 ) + 1;
    int h_max = ( int )( ellipse.center.y + ellipse.size.width/2 ) - 1;
    // loop inside given region vertically
    for ( int i = h_min; i < h_max ; ++i ) {
        // get Y coord.
        int y = abs(i - ellipse.center.y);
        // find region dimensions
        int x_min = ( int )( ellipse.center.x - ( ellipse.size.height/2 )*sqrt( 1 - y*y/((ellipse.size.width/2)*(ellipse.size.width/2)))) + 1;
        int x_max = ( int )( ellipse.center.x + ( ellipse.size.height/2 )*sqrt( 1 - y*y/((ellipse.size.width/2)*(ellipse.size.width/2)))) - 1;
        // loop inside given region horizontaly
        for( int j = x_min; j < x_max; ++j) {
            // count total
            count++;
            // count white
            if(0 < i && i < threshold_output.rows &&
               0 < j && j < threshold_output.cols &&
               threshold_output.at<uchar>(i,j) > 150)
                countWhite ++;
        }
    }
    /// Check count of white pixels inside wllipse
    return countWhite > ((int)(0.8*count));
}

void capture_image () {
    /*
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return;

    if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return; };
     */

    Mat orig_frame;
    orig_frame = imread("/home/vsevolod/workspace/mipt-airdrone/src/localization/marker_detector/image1.jpg"
                         , CV_LOAD_IMAGE_COLOR);
    if (!orig_frame.data) {
        printf("--(!)Error getting picture\n"); return;
    }


    for(;;) {
        //cap >> orig_frame;
        detect(orig_frame);
        if (waitKey(30) > 0) exit(-1);
    }
}

int detect( Mat orig_frame )
{
    Mat frame;
    frame = orig_frame.clone();
    resize(orig_frame, orig_frame, Size(480, 480), 0.3, 0.3, INTER_CUBIC);
    Mat grayscale;
    cvtColor(frame, grayscale, CV_BGR2GRAY);

    Mat thresholded;
    adaptiveThreshold(grayscale, thresholded, 255, ADAPTIVE_THRESH_MEAN_C,
                      THRESH_BINARY, (orig_frame.cols + orig_frame.rows) / 32 - 1, -20);
    medianBlur(thresholded, thresholded, 9);

    int erosion_size = 7;
    Mat element = getStructuringElement(cv::MORPH_CROSS,
           cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
           cv::Point(erosion_size, erosion_size) );

    // Apply erosion or dilation on the image
    erode(thresholded, thresholded, element);

    namedWindow( "treshholded", CV_WINDOW_AUTOSIZE );
    imshow( "treshholded", thresholded );


    vector<vector<Point> > contours;
    findContours(thresholded, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    vector<vector<Point> > minContours;

    for( int i = 0; i < contours.size(); i++ ) {
        if( contours[i].size() > 50)
            minContours.push_back(contours[i]);
    }

    /*
    /// Draw contours + rotated rects + ellipses
    Mat drawing = Mat::zeros(frame.size(), CV_8UC3);
    for( int i = 0; i< minContours.size(); i++ ) {
        drawContours( frame, minContours, i, Scalar(0, 255, 255), 2, 8);
    }
    */

    vector<vector<Point> > curves;
    vector<Point> approxCurve;
    for (int i = 0; i < minContours.size(); i++) {
        double eps = minContours[i].size() * 0.0005;
        approxPolyDP(minContours[i], approxCurve, eps, true);
        if (approxCurve.size() < 4)
            continue;
        curves.push_back(approxCurve);
    }


    cv::Mat labels = cv::Mat::zeros(frame.size(), CV_8UC1);
    std::vector<float> cont_avgs(curves.size(), 0.f); // This contains the averages of each contour

    for (size_t i = 0; i < curves.size(); ++i)
    {
        // Labels starts at 1 because 0 means no contour
        cv::drawContours(labels, curves, i, cv::Scalar(i+1), CV_FILLED);
    }

    std::vector<float> counts(curves.size(), 0.f);
    const int width = frame.rows;
    for (int i = 0; i < frame.rows; ++i)
    {
        for (size_t j = 0; j < frame.cols; ++j)
        {
            uchar label = labels.data[i*width + j];

            if (label == 0)
            {
                continue;   // No contour
            }
            else
            {
                label -= 1; // Make labels zero-indexed
            }

            uchar value = frame.data[i*width + j];
            cont_avgs[label] += value;
            ++counts[label];
        }
    }

    for (int i = 0; i < cont_avgs.size(); ++i)
    {
        cont_avgs[i] /= counts[i];
    }

    vector<vector<Point> > target_contours;

    Mat drawing = Mat::zeros(frame.size(), CV_8UC3);
    for( int i = 0; i< curves.size(); i++ ) {
        if (cont_avgs[i] > 0 && contourArea(curves[i]) > 30) {
            drawContours( frame, curves, i, Scalar(0, 0, 255), 2, 8);
            target_contours.push_back(curves[i]);
        }
    }



    /// Get the moments
    vector<Moments> mu(target_contours.size() );
    for( int i = 0; i < target_contours.size(); i++ )
       { mu[i] = moments( target_contours[i], false ); }

    ///  Get the mass centers:
    vector<Point2f> mc( target_contours.size() );
    for( int i = 0; i < target_contours.size(); i++ )
       { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

    Point2f target (0, 0);
    /// Draw contours
    drawing = Mat::zeros( frame.size(), CV_8UC3 );
    if (target_contours.size() > 0) {
        for( int i = 0; i < target_contours.size(); i++ )
        {
            target += mc[i];
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            circle( frame, mc[i], 4, color, 10, 8, 0 );
        }
        target.x /=  target_contours.size();
        target.y /=  target_contours.size();
        circle( frame, target, 4, Scalar(0, 0, 255), 10, 8, 0);
    }

    namedWindow( "frame", CV_WINDOW_AUTOSIZE );
    imshow("frame", frame);

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
