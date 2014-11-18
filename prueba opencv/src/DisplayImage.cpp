

//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <iostream>
//
////Maybe in OpenCV2.2 the correct include statement would be:
////#include "opencv2/opencv.hpp"
//
//
//int main(int, char**)
//{
//    cv::VideoCapture capLeft(1); // open the Left camera
//    cv::VideoCapture capRight(0); // open the Right camera
//
//    if(!capLeft.isOpened() || !capRight.isOpened())  // check if we succeeded
//    {
//        std::cerr << "ERROR: Could not open cameras." << std::endl;
//        return -1;
//    }
//
//
//    cv::namedWindow("Left",1);
//    cv::namedWindow("Right",1);
//
//    for(;;)
//    {
//        bool isValid = true;
//
//
//        cv::Mat frameLeft;
//        cv::Mat frameRight;
//
//        try
//        {
//          capLeft >> frameLeft; // get a new frame from left camera
//          capRight >> frameRight; //get a new frame from right camera
//        }
//        catch( cv::Exception& e )
//        {
//          std::cout << "An exception occurred. Ignoring frame. " << e.err << std::endl;
//          isValid = false;
//        }
//
//        if (isValid)
//        {
//          try
//          {
//            cv::imshow("Left", frameLeft);
//            cv::imshow("Right", frameRight);
//
//            /************************************************************
//            *    This is the place for all the cool stuff that you      *
//            *    want to do with your stereo images                     *
//            ************************************************************/
//
//            //TODO:...
//
//          }
//          catch( cv::Exception& e )
//          {
//            /************************************************************
//            *    Sometimes an "Unrecognized or unsuported array type"   *
//            *    exception is received so we handle it to avoid dying   *
//            ************************************************************/
//            std::cout << "An exception occurred. Ignoring frame. " << e.err << std::endl;
//          }
//        }
//        if(cv::waitKey(30) >= 0) break;
//    }
//    // the camera will be deinitialized automatically in VideoCapture destructor
//    return 0;
//}


//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <stdio.h>
//
//using namespace cv;
//using namespace std;
//
//int main( int argc, char** argv )
//{
//  Mat image;
//  image = imread( argv[1], 1 );
//
//  if( argc != 2 || !image.data )
//    {
//      printf( "No image data \n" );
//      return -1;
//    }
//
//  namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
//  imshow( "Display Image", image );
//
//  waitKey(0);
//
//  return 0;
//}
