
#include "flow.h"





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



int haar( int argc, char** argv )
{
  CvCapture* capture;
  Mat frame;

  if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };

  capture = cvCaptureFromCAM( -1 );
  if( !capture ) {
	  ROS_ERROR("No camera found! Aborting...");
	  return 0;
  }

  while( true )
  {
	  frame = cvQueryFrame( capture );

      if( !frame.empty() ) {

    	  Mat downsampled;
    	  Mat tilted_1;
    	  Mat tilted_2;

    	  resize(frame, downsampled, Size(), 0.3, 0.3, INTER_CUBIC);
    	  resize(frame, tilted_1,    Size(), 0.3, 0.7, INTER_CUBIC);

    	  detectAndDisplay( tilted_1 );

      }
      else {
    	  printf(" --(!) No captured frame -- Break!");
    	  break;
      }

      int c = waitKey(1);
      if( (char)c == 'c' ) { break; }
     }

  return 0;
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
