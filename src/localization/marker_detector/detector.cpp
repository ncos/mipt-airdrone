
#include "detector.h"

using namespace std;
using namespace cv;

/** Global variables */
//String face_cascade_name = "/home/ncos/catkin_ws/src/marker_detector/cascades/haarcascade_frontalface_alt.xml";
String face_cascade_name = "/home/ncos/catkin_ws/src/marker_detector/cascades/mycrossdetector__new_20st_20x20_nosim.xml";
//String face_cascade_name = "/home/ncos/catkin_ws/src/marker_detector/cascades/mycrossdetector3.xml";

CascadeClassifier face_cascade;
string window_name = "Capture - Marker detection";
string window_name_t1 = "Capture - Marker detection -t1";
string window_name_t2 = "Capture - Marker detection -t2";

RNG rng(12345);

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
               threshold_output.at<uchar>(i,j))
                countWhite ++;
        }
    }
    /// Check count of white pixels inside wllipse
    return countWhite > ((int)(0.8*count));
}

void detectAndDisplay( Mat frame )
{
    std::vector<Rect> faces;
    Mat frame_gray;
    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    face_cascade.detectMultiScale( frame_gray, faces, 1.05, 2, 0|CV_HAAR_SCALE_IMAGE, Size(10, 10) );
    for( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
        ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }
    imshow( window_name, frame );
}

int haar( Mat frame )
{

    Mat downsampled;
    Mat tilted_1;
    Mat tilted_2;
    resize(frame, downsampled, Size(), 0.3, 0.3, INTER_CUBIC);
    resize(frame, tilted_1, Size(), 0.3, 0.7, INTER_CUBIC);
    detectAndDisplay( tilted_1 );

    return 0;
}


int detect( int argc, char** argv )
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    for(;;)
    {
        Mat orig_frame;
        cap >> orig_frame; // get a new frame from camera
        Mat frame;
        frame = orig_frame.clone();
        Mat graysacale;
        cvtColor(frame, graysacale, CV_BGR2GRAY);
        edges = graysacale.clone();

        Mat canny_output;
        double thresh = 10;
        blur( edges, edges, Size(3,3) );
        Canny( edges, canny_output, thresh, thresh * 2, 3);

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

         /// Find the rotated rectangles and ellipses for each contour
        vector<RotatedRect> minRect( contours.size() );
        vector<RotatedRect> minEllipse( contours.size() );

        for( int i = 0; i < contours.size(); i++ ) {
            minRect[i] = minAreaRect( Mat(contours[i]) );
            if( contours[i].size() > 500 )
                 minEllipse[i] = fitEllipse( Mat(contours[i]) );
        }

        Mat treshholded;
        adaptiveThreshold(graysacale, treshholded, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
                                                        THRESH_BINARY, 7, 2);

        vector<RotatedRect> ellipseWithWhite;
        // Check percentage of white pixel inside ellipses
        for(unsigned int i = 0; i < minEllipse.size() ; i++) {
            if(checkWhiteInEllipse(minEllipse.at(i), treshholded.clone()))
                ellipseWithWhite.push_back(minEllipse.at(i));
        }

        /// Draw contours + rotated rects + ellipses
        Mat drawing = Mat::zeros(frame.size(), CV_8UC3);
        for( int i = 0; i< ellipseWithWhite.size(); i++ ) {
            // ellipse
            ellipse(frame, ellipseWithWhite[i], Scalar(0, 0, 255), 2, 8);
        }

        for( int i = 0; i< ellipseWithWhite.size(); i++ ) {
            // Corners of the source image
            Point2d center;
            center.x = ellipseWithWhite[i].center.x;
            center.y = ellipseWithWhite[i].center.y;
            double half_width;
            half_width = ellipseWithWhite[i].size.width / 2;
            double half_height;
            half_height = ellipseWithWhite[i].size.height / 2;

            std::vector<Point2f> corners;
            Point2f corners_arr[4];
            ellipseWithWhite[i].points(corners_arr);
            corners.push_back(corners_arr[0]);
            corners.push_back(corners_arr[1]);
            corners.push_back(corners_arr[2]);
            corners.push_back(corners_arr[3]);

            // Define the destination image
            Mat quad = Mat::zeros(300, 300, CV_8UC3);
            // Corners of the destination image
            std::vector<Point2f> quad_pts;
            quad_pts.push_back(Point2f(0, 0));
            quad_pts.push_back(Point2f(quad.cols, 0));
            quad_pts.push_back(Point2f(quad.cols, quad.rows));
            quad_pts.push_back(Point2f(0, quad.rows));

            // Get transformation matrix
            Mat transmtx = getPerspectiveTransform(corners, quad_pts);

            // Apply perspective transformation
            warpPerspective(orig_frame, quad, transmtx, quad.size());
            imshow("quadrilateral", quad);

            Mat quad_grayscale;
            cvtColor(quad, quad_grayscale, CV_BGR2GRAY);
            std::vector<Rect> faces;
            face_cascade.detectMultiScale( quad, faces, 1.001, 0);

            std::cout << faces.size() << std::endl;
            for( size_t i = 0; i < faces.size(); i++ )
            {
                Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
                ellipse( quad_grayscale, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
            }
            imshow( window_name, quad_grayscale );
        }

        /// Show in a window
        namedWindow( "treshholded", CV_WINDOW_AUTOSIZE );
        imshow( "treshholded", treshholded );

        namedWindow( "frame", CV_WINDOW_AUTOSIZE );
        imshow("frame", frame);

        namedWindow( "canny", CV_WINDOW_AUTOSIZE );
        imshow("canny", canny_output);

        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
