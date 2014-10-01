
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
                                                    THRESH_BINARY_INV, 499, 20);
    medianBlur(thresholded, thresholded, 5);

    namedWindow( "treshholded", CV_WINDOW_AUTOSIZE );
    imshow( "treshholded", thresholded );


    vector<vector<Point> > contours;
    findContours(thresholded, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    vector<vector<Point> > minContours;

    for( int i = 0; i < contours.size(); i++ ) {
        if( contours[i].size() > 50)
            minContours.push_back(contours[i]);
    }

    /// Draw contours + rotated rects + ellipses
    Mat drawing = Mat::zeros(frame.size(), CV_8UC3);
    for( int i = 0; i< minContours.size(); i++ ) {
        drawContours( frame, minContours, i, Scalar(0, 255, 255), 2, 8);
    }

    vector<vector<Point> > curves;
    vector<Point> approxCurve;
    for (int i = 0; i < minContours.size(); i++) {
        double eps = minContours[i].size() * 0.005;
        approxPolyDP(minContours[i], approxCurve, eps, true);
        curves.push_back(approxCurve);
    }

    drawing = Mat::zeros(frame.size(), CV_8UC3);
    for( int i = 0; i< curves.size(); i++ ) {
        drawContours( frame, curves, i, Scalar(0, 0, 255), 2, 8);
    }

    vector<Vec4i> lines;
    Mat canny_out;
    Canny(thresholded, canny_out, 0, 255, 3, true);
    imshow("canny_out", canny_out);
    HoughLinesP(canny_out, lines, 1, CV_PI/180, 80, 30, 10);
    for( size_t i = 0; i < lines.size(); i++ ) {
        Vec4i l = lines[i];
        line( frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 3, CV_AA);
    }

    /*
    vector<RotatedRect> ellipseWithWhite;
    // Check percentage of white pixel inside ellipses
    for(unsigned int i = 0; i < minContours.size() ; i++) {
        if(checkWhiteInEllipse(minContours.at(i), grayscale.clone()))
            ellipseWithWhite.push_back(minContours.at(i));
    }

    /// Draw contours + rotated rects + ellipses
    drawing = Mat::zeros(frame.size(), CV_8UC3);
    for( int i = 0; i< ellipseWithWhite.size(); i++ ) {
        // ellipse
        ellipse(frame, ellipseWithWhite[i], Scalar(0, 0, 255), 2, 8);
    }

    vector<Point2f> target_points;

    for( int i = 0; i< ellipseWithWhite.size(); i++ ) {
        Point2f corners_arr[4];
        ellipseWithWhite[i].points(corners_arr);

        vector<Point2f> corners_vec (corners_arr, corners_arr + sizeof(corners_arr) / sizeof(corners_arr[0]) );

        // Define the destination image
        Mat quad = Mat::zeros(300, 300, CV_8UC3);
        Point2f quad_pts_arr [4];
        quad_pts_arr[0] = Point2f(0, 0);
        quad_pts_arr[1] = Point2f(quad.cols, 0);
        quad_pts_arr[2] = Point2f(quad.cols, quad.rows);
        quad_pts_arr[3] = Point2f(0, quad.rows);
        // Corners of the destination image
        vector<Point2f> quad_pts (quad_pts_arr, quad_pts_arr + sizeof(quad_pts_arr) / sizeof(quad_pts_arr[0]) );
        //quad_pts.push_back(Point2f(0, 0));
        //quad_pts.push_back(Point2f(quad.cols, 0));
        //quad_pts.push_back(Point2f(quad.cols, quad.rows));
        //quad_pts.push_back(Point2f(0, quad.rows));

        // Get transformation matrix
        //Mat transmtx = findHomography(corners_vec, quad_pts);
        Mat transmtx = getPerspectiveTransform(corners_vec, quad_pts);

        // Apply perspective transformation
        warpPerspective(orig_frame, quad, transmtx, quad.size());
        //imshow("quadrilateral", quad);

        Mat edges_quad;
        Canny(quad, edges_quad, 50, 200, 3);
        Mat color_dst;
        cvtColor(edges_quad, color_dst, CV_GRAY2BGR);
        //imshow("canny quad", color_dst);

        vector<Vec4i> lines;
        HoughLinesP( edges_quad, lines, 1, CV_PI/180, 80, 30, 10 );

        int ort_counter = 0;
        double scalar_mul;
        Point2f vec1, vec2;
        double len1, len2;
        if (lines.size() == 0)
            continue;
        for( int j = 0; j < lines.size() - 1; j++ ) {
            vec1.x = lines[j][2] - lines[j][0];
            vec1.y = lines[j][3] - lines[j][1];

            len1 = sqrt (vec1.x * vec1.x + vec1.y * vec1.y);
            if (!(len1 > 0))
                continue;
            vec1.x /= len1;
            vec1.y /= len1;

            for( int k = j; k < lines.size(); k++ ) {
                vec2.x = lines[k][2] - lines[k][0];
                vec2.y = lines[k][3] - lines[k][1];

                len2 = sqrt (vec2.x * vec2.x + vec2.y * vec2.y);
                if (!(len2 > 0))
                    continue;
                vec2.x /= len2;
                vec2.y /= len2;

                scalar_mul = vec1.x * vec2.x + vec1.y * vec2.y;
                if (fabs(scalar_mul) < 0.1)
                    ort_counter++;

            }
        }

        //imshow("line quad", color_dst);

        std::cout << ort_counter << std::endl;
        fflush(stdout);

        if (ort_counter < 3) {
            continue;
        }

       target_points.push_back(ellipseWithWhite[i].center);
       std::cout << "Pushed" << std::endl;
    }


    Point2f final_target (0, 0);
    std::cout << "Size targ " << target_points.size() << std::endl;
    if (target_points.size() > 0) {
        for (int i = 0; i < target_points.size(); i++) {
            final_target += target_points[i];
        }

        final_target.x /= target_points.size();
        final_target.y /= target_points.size();

        std::cout << "Targ " << final_target.x << " " << final_target.y << std::endl;

        circle(frame, final_target, 16, Scalar (255, 255, 0), 10);
    }
    /// Show in a window

    /*
    namedWindow( "grayscale", CV_WINDOW_AUTOSIZE );
    imshow( "grayscale", grayscale );

    namedWindow( "treshholded", CV_WINDOW_AUTOSIZE );
    imshow( "treshholded", treshholded );

    namedWindow( "canny", CV_WINDOW_AUTOSIZE );
    imshow("canny", canny_output);
    */

    namedWindow( "frame", CV_WINDOW_AUTOSIZE );
    imshow("frame", frame);

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
