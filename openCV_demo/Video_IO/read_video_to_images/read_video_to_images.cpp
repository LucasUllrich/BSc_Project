#include <iostream>
#include <fstream>
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

#define SOLUTION_1 0
#define SOLUTION_2 0

// DEBUGGING
#define CSV_OUTPUT 0
#define BASH_OUTPUT 0
#define DEBUG_TIME 0
#define DEBUG_PLAN 1
#define IMAGE_PROCESSING 1

#if DEBUG_TIME
  #include <chrono>
#endif

struct GPS_Point {
  double startLatitude;
  double startLongitude;
  double endLatitude;
  double endLongitude;
};

struct Line_Marking_Points {
  double latitude;
  double longitude;
};

long double degreeToRadiant(long double degree)
{
  return (degree * (long double) PI / 180);
}

int main (int argc, char ** argv)
{
  if (argc < 3)
  {
    cout << "Wrong usage, please specify a video and plan file!" << endl;
    return -1;
  }


  /******************************************************/
  /*** Parsing of plan file to optain line paramteres ***/
  /******************************************************/
  const string planFileName = argv[1];
  ifstream planFile;
  planFile.open(planFileName);
  if (!planFile.is_open())
  {
    cout << "Could not open plan file!" << endl;
    return -1;
  }
  string line;
  int dataCounter;
  GPS_Point *gpsPoint = new GPS_Point[1];
  GPS_Point *tempGpsPoint = new GPS_Point[1];
  for(dataCounter = 0; getline(planFile, line); )
  {
    if(line.compare(0, 2, "S:") == 0)
    {
      // Starting point
      size_t pos = line.find("/");
      string latitude = line.substr(2, pos - 2);
      string longitude = line.substr(pos + 1);
      size_t stringSize;
      tempGpsPoint[0].startLatitude = stod(latitude, &stringSize);
      if(stringSize < 9)
      {
        cout << "Error processing starting point of " << dataCounter << ". GPS Line, to few digits" << endl;
        return -1;
      }
      tempGpsPoint[0].startLongitude = stod(longitude, &stringSize);
      if(stringSize < 9)
      {
        cout << "Error processing starting point of " << dataCounter << ". GPS Line, to few digits" << endl;
        return -1;
      }
    }
    else if (line.compare(0, 2, "E:") == 0)
    {
      // Ending point
      size_t pos = line.find("/");
      string latitude = line.substr(2, pos - 2);
      string longitude = line.substr(pos + 1);
      size_t stringSize;
      tempGpsPoint[0].endLatitude = stod(latitude, &stringSize);
      if(stringSize < 9)
      {
        cout << "Error processing ending point of " << dataCounter << ". GPS Line, to few digits" << endl;
        return -1;
      }
      tempGpsPoint[0].endLongitude = stod(longitude, &stringSize);
      if(stringSize < 9)
      {
        cout << "Error processing ending point of " << dataCounter << ". GPS Line, to few digits" << endl;
        return -1;
      }
      dataCounter++;
      GPS_Point *temp = new GPS_Point[dataCounter];
      memcpy(temp, gpsPoint, (dataCounter - 1) * sizeof(GPS_Point));
      memcpy(temp + (dataCounter - 1), tempGpsPoint, sizeof(GPS_Point));
      #if DEBUG_PLAN
        cout.precision(8);
        cout << "**************************" << endl;
        for(int debugCounter = 0; debugCounter < dataCounter; debugCounter++)
        {
          cout << "TEMP: StartLat:  " << temp[debugCounter].startLatitude << endl;
          cout << "TEMP: StartLong: " << temp[debugCounter].startLongitude << endl;
          cout << "TEMP: EndLat:  " << temp[debugCounter].endLatitude << endl;
          cout << "TEMP: EndLong: " << temp[debugCounter].endLongitude << endl << endl;
        }
      #endif
      delete[] gpsPoint;
      gpsPoint = temp;
    }
  }
  planFile.close();
  delete[] tempGpsPoint;

  #if DEBUG_PLAN
    
    for(int debugCounter = 0; debugCounter < dataCounter; debugCounter++)
    {
      cout << "GPS Line Number: " << (debugCounter + 1) << endl;
      cout << "StartLat:  " << gpsPoint[debugCounter].startLatitude << endl;
      cout << "StartLong: " << gpsPoint[debugCounter].startLongitude << endl;
      cout << "EndLat:  " << gpsPoint[debugCounter].endLatitude << endl;
      cout << "EndLong: " << gpsPoint[debugCounter].endLongitude << endl;
    }
  #endif



  /***********************************/
  /*** Calculation of line markers ***/
  /***********************************/
  size_t markingSize = 0;
  Line_Marking_Points *lineMark = new Line_Marking_Points[1];
  for(int lineCounter = 0; lineCounter < dataCounter; lineCounter++)
  {
    double deltaLatitude = gpsPoint[lineCounter].endLatitude - gpsPoint[lineCounter].startLatitude;
    double deltaLongitude = gpsPoint[lineCounter].endLongitude - gpsPoint[lineCounter].startLongitude;
    int steps;
    if (deltaLatitude > deltaLongitude)
      steps = deltaLatitude * 1000000;
    else
      steps = deltaLatitude * 1000000;

    if (steps < 0)
      steps = steps * -1;

    Line_Marking_Points *tempLineMark = new Line_Marking_Points[++steps];
    for(int stepCounter = 0; stepCounter < steps; stepCounter++)
    {
      tempLineMark[stepCounter].latitude = gpsPoint[lineCounter].startLatitude + (double) stepCounter * (deltaLatitude / (double) steps);
      tempLineMark[stepCounter].longitude = gpsPoint[lineCounter].startLongitude + (double) stepCounter * (deltaLongitude / (double) steps);
    }
    
    markingSize += steps;
    Line_Marking_Points *temp = new Line_Marking_Points[markingSize];
    memcpy(temp, lineMark, (markingSize - steps) * sizeof(Line_Marking_Points));
    memcpy(temp + (markingSize - steps), tempLineMark, steps * sizeof(Line_Marking_Points));
    delete[] lineMark;
    lineMark = temp;
    delete[] tempLineMark;
  }

  #if DEBUG_PLAN
    for(int debugCounter = 0; debugCounter < markingSize; debugCounter++)
    {
      cout << "Line Mark Nr.: " << debugCounter << endl;
      cout << "Latitude:  " << lineMark[debugCounter].latitude << endl;
      cout << "Longitude: " << lineMark[debugCounter].longitude << endl << endl;
    }
  #endif



  /************************/
  /*** Image processing ***/
  /************************/
  #if IMAGE_PROCESSING
  const string videoSrc = argv[2];
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
  int frameHeight = 0;
  int frameWidth = 0;

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
    long double latitude = 48.3788083;
    long double longitude = 16.8258389;
    // VID1
    double direction = 189;
    // VID2
    // double direction = 159;
    

    // Position variables
    double tilt = 80;

    // Coordinate variables
    long double distanceOfBaseline;
    long double distanceOfSideline;
    long double distanceOfBaselineCenterNorth;
    long double distanceOfBaselineCenterEast;
    long double distanceOfSidelineNorth;
    long double distanceOfSidelineEast;
    long double pixelPositionEast;
    long double pixelPositionNorth;

    cout.precision(9);

    long double halfFrameHeight = (double) frameHeight / 2;
    long double halfFrameWidth = (double) frameWidth / 2;

    #if CSV_OUTPUT
      #if SOLUTION_1
        cout << "row;column;pixelPositionEast;pixelPositionNorth;" << endl;
      #endif
      #if SOLUTION_2
        cout << "row;column;distanceOfBaseline;pixelPositionEast;pixelPositionNorth" << endl;
      #endif
    #endif

    #if DEBUG_TIME
      auto begin = chrono::high_resolution_clock::now();
    #endif


/****************************************/
/*** Solution 1, purely trigonometric ***/
/****************************************/
#if SOLUTION_1
    for (int row = 1; row <= frameHeight; row++)
    {
      long double baselinePixelAngle = (tilt + ((long double) AOV_V / 2)) - (((long double) frameHeight - (row - 1)) / (long double) frameHeight) * (long double) AOV_V;
      if(baselinePixelAngle > 87)
        break;    // Aborting calulation because the distance is irrelevant
      distanceOfBaseline = tan(degreeToRadiant(baselinePixelAngle)) * (long double) HEIGHT;
      distanceOfBaselineCenterEast = sin(degreeToRadiant(direction)) * distanceOfBaseline;
      distanceOfBaselineCenterNorth = cos(degreeToRadiant(direction)) * distanceOfBaseline;

      for (int column = 1; column <= frameWidth; column++)
      {
        long double sidelinePixelAngle;
        if (column < (halfFrameWidth + 1))  // Left image side
        {
          sidelinePixelAngle = ((long double) AOV_H / 2) * (halfFrameWidth - (column - 1)) / halfFrameWidth;
          distanceOfSideline = tan(degreeToRadiant(sidelinePixelAngle)) * distanceOfBaseline;
          distanceOfSidelineEast = - cos(degreeToRadiant(direction)) * distanceOfSideline;
          distanceOfSidelineNorth = sin(degreeToRadiant(direction)) * distanceOfSideline;
        }
        else  // Right image side
        {
          sidelinePixelAngle = ((long double) AOV_H / 2) * ((column - 1) - halfFrameWidth) / halfFrameWidth;
          distanceOfSideline = tan(degreeToRadiant(sidelinePixelAngle)) * distanceOfBaseline;
          distanceOfSidelineEast = cos(degreeToRadiant(direction)) * distanceOfSideline;
          distanceOfSidelineNorth = - sin(degreeToRadiant(direction)) * distanceOfSideline;
        }

        long double pixelDistanceEast = distanceOfBaselineCenterEast + distanceOfSidelineEast;
        long double pixelDistanceNorth = distanceOfBaselineCenterNorth + distanceOfSidelineNorth;

        pixelPositionEast = longitude + ((pixelDistanceEast / 1000000) / 111.32);
        pixelPositionNorth = latitude + ((pixelDistanceNorth / 1000000) / 111.32);

        #if BASH_OUTPUT
          cout << "Angle: " << sidelinePixelAngle << ";distanceOfSideline: " << distanceOfSideline << ";distanceOfSidelineEast: " << distanceOfSidelineEast << endl;  
          if (column > 100)
            return 0;
        #endif

        #if CSV_OUTPUT
          cout << row << ";" << column<< ";" << pixelPositionEast << ";" << pixelPositionNorth << ";" << endl;
        #endif


      }
    }
#endif /**** SOLUTION 1 ****/



/******************************************************/
/*** Solution 2, triginometric and linear equations ***/
/******************************************************/
#if SOLUTION_2
    for (int row = 1; row <= frameHeight; row++)
    {
      long double baselinePixelAngle = (tilt + ((long double) AOV_V / 2)) - (((long double) frameHeight - (row - 1)) / (long double) frameHeight) * (long double) AOV_V;
      if(baselinePixelAngle > 87)
        break;    // Aborting calulation because the distance is irrelevant
      distanceOfBaseline = tan(degreeToRadiant(baselinePixelAngle)) * (long double) HEIGHT;
      distanceOfBaselineCenterEast = sin(degreeToRadiant(direction)) * distanceOfBaseline;
      distanceOfBaselineCenterNorth = cos(degreeToRadiant(direction)) * distanceOfBaseline;
      long double distanceOfBaselineCenterEastGPS = (distanceOfBaselineCenterEast / 1000000) / 111.32;
      long double distanceOfBaselineCenterNorthGPS = (distanceOfBaselineCenterNorth / 1000000) / 111.32;

      distanceOfSideline = tan(degreeToRadiant((long double) AOV_H / 2)) * distanceOfBaseline;
      // Left point of view
      distanceOfSidelineEast = - cos(degreeToRadiant(direction)) * distanceOfSideline;
      distanceOfSidelineNorth = sin(degreeToRadiant(direction)) * distanceOfSideline;
      long double distanceOfSidelineEastGPS = (distanceOfSidelineEast / 1000000) / 111.32;  // change to latitude position!!!!*********************
      long double distanceOfSidelineNorthGPS = (distanceOfSidelineNorth / 1000000) / 111.32;

      long double longitudeLeftPoint = longitude + distanceOfBaselineCenterEastGPS + distanceOfSidelineEastGPS;
      long double latitudeLeftPoint = latitude + distanceOfBaselineCenterNorthGPS + distanceOfSidelineNorthGPS;

      #if BASH_OUTPUT
        cout << "longitude: " << longitude << endl;
        cout << "latitude:  " << latitude << endl;
        cout << "distanceOfBaselineCenterEast:  " << distanceOfBaselineCenterEast << endl;
        cout << "distanceOfBaselineCenterNorth: " << distanceOfBaselineCenterNorth << endl;
        cout << "****Left point of view****" << endl;
        cout << "distanceOfSidelineEast:  " << distanceOfSidelineEast << endl;
        cout << "distanceOfSidelineNorth: " << distanceOfSidelineNorth << endl;
        cout << "distanceOfSidelineEastGPS:  " << distanceOfSidelineEastGPS << endl;
        cout << "distanceOfSidelineNorthGPS: " << distanceOfSidelineNorthGPS << endl;
      #endif

      // Right point of view
      distanceOfSidelineEast = cos(degreeToRadiant(direction)) * distanceOfSideline;
      distanceOfSidelineNorth = - sin(degreeToRadiant(direction)) * distanceOfSideline;
      distanceOfSidelineEastGPS = (distanceOfSidelineEast / 1000000) / 111.32;
      distanceOfSidelineNorthGPS = (distanceOfSidelineNorth / 1000000) / 111.32;

      long double longitudeRightPoint = longitude + distanceOfBaselineCenterEastGPS + distanceOfSidelineEastGPS;
      long double latitudeRightPoint = latitude + distanceOfBaselineCenterNorthGPS + distanceOfSidelineNorthGPS;

      #if BASH_OUTPUT
        cout << "****Right point of View****" << endl;
        cout << "distanceOfSidelineEast:  " << distanceOfSidelineEast << endl;
        cout << "distanceOfSidelineNorth: " << distanceOfSidelineNorth << endl;
        cout << "distanceOfSidelineEastGPS:  " << distanceOfSidelineEastGPS << endl;
        cout << "distanceOfSidelineNorthGPS: " << distanceOfSidelineNorthGPS << endl;
        cout << "Longitude L/R: " <<  longitudeLeftPoint << " / " << longitudeRightPoint << endl;
        cout << "Latitude  L/R: " << latitudeLeftPoint << " / " << latitudeRightPoint << endl;
        if (row > 10)
          return 0;
      #endif

      long double deviationEast = longitudeRightPoint - longitudeLeftPoint;
      long double deviationNorth = latitudeRightPoint - latitudeLeftPoint;

      long double steppingWidthEast = deviationEast / (long double) frameWidth;
      long double steppingWidthNorth = deviationNorth / (long double) frameWidth;

      #if BASH_OUTPUT
        cout << "*******" << endl;
        cout << "deviationEast:  " << deviationEast << endl;
        cout << "deviationNorth: " << deviationNorth << endl;
        cout << "steppingWidthEast:  " << steppingWidthEast << endl;
        cout << "steppingWidthNorth: " << steppingWidthNorth << endl << endl;
      #endif

      for (int column = 1; column <= frameWidth; column++)
      {
        pixelPositionEast = longitudeLeftPoint + column * steppingWidthEast;   //// ERROR ***********
        pixelPositionNorth = latitudeLeftPoint + column * steppingWidthNorth;
        #if BASH_OUTPUT
          cout << "East: " << pixelPositionEast << "\tNorth: " << pixelPositionNorth <<  endl;
          if (column > 10)
          return 0;
        #endif

        #if CSV_OUTPUT
          cout << row << ";" << column<< ";" << distanceOfBaseline << ";" << pixelPositionEast << ";" << pixelPositionNorth << endl;
        #endif
      }
    }
#endif  /**** SOLUTION 2 ****/



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
  
  #endif
  delete[] lineMark;
  delete[] gpsPoint;
  cout << "Mem cleared" << endl;
  return 0;
}