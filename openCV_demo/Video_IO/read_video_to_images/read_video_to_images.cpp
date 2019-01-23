#include <iostream>
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

#define AOV_V 31  // AngleOfView_Veritically
#define AOV_H 67  // AngleOfView_Horizontally
#define HEIGHT 1400 // Measurement in mm, height over ground in wich video was captured
#define PI 3.14159265

#define CSV_OUTPUT 1
#define BASH_OUTPUT 0
#define DEBUG_TIME 0
#define SOLUTION_1 0
#define SOLUTION_2 1

#if DEBUG_TIME
  #include <chrono>
#endif

double degreeToRadiant(double degree)
{
  return (degree * (double) PI / 180);
}

int main (int argc, char ** argv)
{
  if (argc < 2)
  {
    cout << "Wrong usage, please specify a video file!" << endl;
    return -1;
  }

  Mat src;
  const string videoSrc = argv[1];
  VideoCapture captVidSrc(videoSrc);

  if(!captVidSrc.isOpened())
  {
    cout << "Could not open video!" << endl;
    return -1;
  }

  // Windows
  const char * WIN_SRC = "Source Video";
  namedWindow(WIN_SRC, WINDOW_AUTOSIZE);

  // Image data
  double frameHeight = 0;
  double frameWidth = 0;

  frameHeight = captVidSrc.get(CAP_PROP_FRAME_HEIGHT);
  frameWidth = captVidSrc.get(CAP_PROP_FRAME_WIDTH);

  while(1)
  {
    Mat frame;

    if(!captVidSrc.read(frame))
    {
      cout << "All frames read or error reading a frame" << endl;
      break;
    }

    // GPS Data
    // Temporarily
    double latitude = 48.3788083;
    double longitude = 16.8258389;
    double direction = 189;

    // Position variables
    double tilt = 50;

    // Coordinate variables
    double distanceOfBaseline;
    double distanceOfSideline;
    double distanceOfBaselineCenterNorth;
    double distanceOfBaselineCenterEast;
    double distanceOfSidelineNorth;
    double distanceOfSidelineEast;
    double distanceNorth;
    double distanceEast;


/**
    distanceOfBaseline = tan(degreeToRadiant((tilt - (AOV_V / 2)))) * HEIGHT;
    distanceOfSideline = tan(degreeToRadiant(AOV_H / 2)) * distanceOfBaseline;

    distanceOfBaselineNorth = cos(degreeToRadiant(direction)) * distanceOfBaseline;
    distanceOfBaselineEast = sin(degreeToRadiant(direction)) * distanceOfBaseline;

    distanceOfSidelineNorth = sin(degreeToRadiant(direction)) * distanceOfSideline;
    distanceOfSidelineEast = - cos(degreeToRadiant(direction)) * distanceOfSideline;

    distanceNorth = distanceOfBaselineNorth + distanceOfSidelineNorth;
    distanceEast = distanceOfBaselineEast + distanceOfSidelineEast;

    cout << "DoB: " << distanceOfBaseline << endl;
    cout << "DoS: " << distanceOfSideline << endl;
    cout << "DoB_N: " << distanceOfBaselineNorth << endl;
    cout << "DoB_E: " << distanceOfBaselineEast << endl;
    cout << "DoS_N: " << distanceOfSidelineNorth << endl;
    cout << "DoS_E: " << distanceOfSidelineEast << endl << endl;

    cout << frameHeight << "/" << frameWidth << endl;

  **/

    double halfFrameHeight = frameHeight / 2;
    double halfFrameWidth = frameWidth / 2;

    #if CSV_OUTPUT
      cout << "baselinePixelAngle;sidelinePixelAngle;distanceOfBaseline;distanceOfSideline;distanceOfSidelineEast;distanceOfSidelineNorth" << endl;
    #endif

    #if DEBUG_TIME
      auto begin = chrono::high_resolution_clock::now();
    #endif

/***************************************/
/*** Solution 1, purely trigonometric **/
/***************************************/
#if SOLUTION_1
    for (int row = 1; row <= frameHeight; row++)
    {
      double baselinePixelAngle = (tilt + ((double) AOV_V / 2)) - ((frameHeight - (row - 1)) / frameHeight) * (double) AOV_V;
      distanceOfBaseline = tan(degreeToRadiant(baselinePixelAngle)) * (double) HEIGHT;

      for (int column = 1; column <= frameWidth; column++)
      {
        double sidelinePixelAngle;
        if (column < (halfFrameWidth + 1))  // Left image side
        {
          sidelinePixelAngle = ((double) AOV_H / 2) * (halfFrameWidth - (column - 1)) / halfFrameWidth;
          distanceOfSideline = tan(degreeToRadiant(sidelinePixelAngle)) * distanceOfBaseline;
          distanceOfSidelineEast = - cos(degreeToRadiant(direction)) * distanceOfSideline;
          distanceOfSidelineNorth = sin(degreeToRadiant(direction)) * distanceOfSideline;
        }
        else  // Right image side
        {
          sidelinePixelAngle = ((double) AOV_H / 2) * ((column - 1) - halfFrameWidth) / halfFrameWidth;
          distanceOfSideline = tan(degreeToRadiant(sidelinePixelAngle)) * distanceOfBaseline;
          distanceOfSidelineEast = cos(degreeToRadiant(direction)) * distanceOfSideline;
          distanceOfSidelineNorth = - sin(degreeToRadiant(direction)) * distanceOfSideline;
        }

        #if BASH_OUTPUT
          cout << "Angle: " << sidelinePixelAngle << ";distanceOfSideline: " << distanceOfSideline << ";distanceOfSidelineEast: " << distanceOfSidelineEast << endl;
          
          if (column > 100)
            return 0;
        #endif

        #if CSV_OUTPUT
          cout << baselinePixelAngle << ";" << sidelinePixelAngle << ";" << distanceOfBaseline << ";" << distanceOfSideline << ";" << distanceOfSidelineEast << ";" << distanceOfSidelineNorth << ";" << endl;
        #endif


      }
    }
#endif



/*******************************************************/
/****** Solution 2, triginometric and linear equations**/
/*******************************************************/
#if SOLUTION_2
    for (int row = 1; row <= frameHeight; row++)
    {
      double baselinePixelAngle = (tilt + ((double) AOV_V / 2)) - ((frameHeight - (row - 1)) / frameHeight) * (double) AOV_V;
      distanceOfBaseline = tan(degreeToRadiant(baselinePixelAngle)) * (double) HEIGHT;
      distanceOfBaselineCenterEast = sin(degreeToRadiant(direction)) * distanceOfBaseline;
      distanceOfBaselineCenterNorth = cos(degreeToRadiant(direction)) * distanceOfBaseline;
      long double distanceOfBaselineCenterEastGPS = (distanceOfBaselineCenterEast / 1000000) / 111.32;
      long double distanceOfBaselineCenterNorthGPS = (distanceOfBaselineCenterNorth / 1000000) / 111.32;

      distanceOfSideline = tan(degreeToRadiant((double) AOV_H / 2)) * distanceOfBaseline;
      // Left point of view
      distanceOfSidelineEast = - cos(degreeToRadiant(direction)) * distanceOfSideline;
      distanceOfSidelineNorth = sin(degreeToRadiant(direction)) * distanceOfSideline;
      long double distanceOfSidelineEastGPS = (distanceOfSidelineEast / 1000000) / 111.32;
      long double distanceOfSidelineNorthGPS = (distanceOfSidelineNorth / 1000000) / 111.32;

      long double longitudeLeftPoint = longitude + distanceOfBaselineCenterEastGPS + distanceOfSidelineEastGPS;
      long double latitudeLeftPoint = latitude + distanceOfBaselineCenterNorthGPS + distanceOfSidelineNorthGPS;

      // Right point of view
      distanceOfSidelineEast = cos(degreeToRadiant(direction)) * distanceOfSideline;
      distanceOfSidelineNorth = - sin(degreeToRadiant(direction)) * distanceOfSideline;
      distanceOfSidelineEastGPS = (distanceOfSidelineEast / 1000000) / 111.32;
      distanceOfSidelineNorthGPS = (distanceOfSidelineNorth / 1000000) / 111.32;

      long double longitudeRightPoint = longitude + distanceOfBaselineCenterEastGPS + distanceOfSidelineEastGPS;
      long double latitudeRightPoint = latitude + distanceOfBaselineCenterNorthGPS + distanceOfSidelineNorthGPS;

      long double deviationEast = longitudeRightPoint - longitudeLeftPoint;
      long double deviationNorth = latitudeRightPoint - latitudeLeftPoint;

      ////////// implement function to be used by pixels

      for (int column = 1; column <= frameWidth; column++)
      {

      }
    }
#endif

    #if DEBUG_TIME
      auto end = chrono::high_resolution_clock::now();
      auto dur = end - begin;
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
      cout << ms << endl;
    #endif

    #if CSV_OUTPUT
      return 0;
    #endif

    imshow(WIN_SRC, frame);
    if(waitKey(100) >= 0)
    {
      break;
    }

    if(captVidSrc.get(CAP_PROP_POS_FRAMES) == 10)
    {
      imwrite("VideoFrame.png", frame);
    }
  }
  
  return 0;
}