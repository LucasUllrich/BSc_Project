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
#define SOLUTION_2 1

// DEBUGGING
#define CSV_OUTPUT 0
#define BASH_OUTPUT 0
#define DEBUG_TIME 0
#define DEBUG_PLAN 0
#define DEBUG_LINE_MARKING 0
#define DEBUG_PLAN_CSV 0
#define DEBUG_MARKING_CSV 0
#define DEBUG_CAMERA_PATH 0
#define IMAGE_PROCESSING 1
#define VIDEO1 0
#define VIDEO2 1
#define STORE_FRAMES 0

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
  /*long double*/int latitude;
  /*long double*/int longitude;
};


long double degreeToRadiant(long double degree)
{
  return (degree * (long double) PI / 180);
}


bool lineMarkCompare(Line_Marking_Points lhs, Line_Marking_Points rhs)
{
  return lhs.latitude < rhs.latitude;
}


double longitudeToLatitude(long double latitude)
{
  return (0.000053979563197308 * pow(latitude, 3) - 0.01911988569736 * pow(latitude, 2) + 0.026419572546895 * latitude + 111.32);
}

bool comparePositionToLineMark(int pixelPositionEast, int pixelPositionNorth, Line_Marking_Points *lmp, int markingSize)
{
  int stepSizeIndexing = markingSize / 19;
  #if DEBUG_LINE_MARKING
    cout << "pixelPositionEast, pixelPositionNorth, lmp.lat, lmp.long, markingSize" << endl;
    cout << pixelPositionEast << "; " << pixelPositionNorth << "; " << lmp[0].latitude << "; " << lmp[0].longitude << "; " << markingSize << endl;
  #endif
  // cout << "pixelPositionNorth: " << pixelPositionNorth << endl;
  // cout << "lmp[221].latitude:  " << lmp[221].latitude << endl << endl;
  // cout << "pixelPositionEast:  " << pixelPositionEast << endl;
  // cout << "lmp[195].longitude: " << lmp[195].longitude << endl << endl;

  for(int indexCounterLat = 0; indexCounterLat < markingSize; indexCounterLat += stepSizeIndexing)
  {
    if((pixelPositionNorth > lmp[indexCounterLat].latitude) & (pixelPositionNorth < lmp[(indexCounterLat + stepSizeIndexing)].latitude))
    {
      // approximate match for latitude
      // cout << "appox match lat" << endl;
      for(int lmpCounterLat = indexCounterLat; lmpCounterLat < (indexCounterLat + stepSizeIndexing); lmpCounterLat++)
      {
        if(pixelPositionNorth == lmp[lmpCounterLat].latitude)
        {
          // match for latitude
          // cout << "match lat" << endl;
          if(pixelPositionEast == lmp[lmpCounterLat].longitude)
          {
            // cout << "match lat" << endl;
            return true;
          }
        }
      }
    }
  }
  return false;
}



/********************/
/*** Main routine ***/
/********************/
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
    if (abs(deltaLatitude) > abs(deltaLongitude))
      steps = abs(deltaLatitude) * 1000000;
    else
      steps = abs(deltaLongitude) * 1000000;

    cout.precision(8);
    Line_Marking_Points *tempLineMark = new Line_Marking_Points[++steps];
    for(int stepCounter = 0; stepCounter < steps; stepCounter++)
    {
      tempLineMark[stepCounter].latitude = (gpsPoint[lineCounter].startLatitude + (long double) stepCounter * (deltaLatitude / (long double) (steps - 1))) * 1000000;
      tempLineMark[stepCounter].longitude = (gpsPoint[lineCounter].startLongitude + (double) stepCounter * (deltaLongitude / (double) (steps - 1))) * 1000000;
    }
    markingSize += steps;
    Line_Marking_Points *temp = new Line_Marking_Points[markingSize];
    memcpy(temp, lineMark, (markingSize - steps) * sizeof(Line_Marking_Points));
    memcpy(temp + (markingSize - steps), tempLineMark, steps * sizeof(Line_Marking_Points));
    delete[] lineMark;
    lineMark = temp;
    delete[] tempLineMark;
  }

  sort(lineMark, lineMark + markingSize, lineMarkCompare);


  #if DEBUG_PLAN
    for(int debugCounter = 0; debugCounter < markingSize; debugCounter++)
    {
      cout << "Line Mark Nr.: " << debugCounter << endl;
      cout << "Latitude:  " << lineMark[debugCounter].latitude << endl;
      cout << "Longitude: " << lineMark[debugCounter].longitude << endl << endl;
    }
  #endif

  #if DEBUG_PLAN_CSV
    for(int debugCounter = 0; debugCounter < markingSize; debugCounter++)
    {
      cout << lineMark[debugCounter].latitude << ";" << lineMark[debugCounter].longitude << endl;
    }
    return 0;
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

  #if VIDEO1
    long double latitudeStart = 48.3788083;
    long double longitudeStart = 16.8258389;
    long double latitudeEnd = 48.378706;
    long double longitudeEnd = 16.825814;
    double direction = 189;
  #endif
  #if VIDEO2
    long double latitudeStart = 48.378986;
    long double longitudeStart = 16.825719;
    long double latitudeEnd = 48.378914;
    long double longitudeEnd =  16.825755;
    double direction = 150;
  #endif

  long double latitudePath[(int)captVidSrc.get(CAP_PROP_FRAME_COUNT)];
  long double longitudePath[(int)captVidSrc.get(CAP_PROP_FRAME_COUNT)];

  double tilt = 89;
  Mat frame;

  int frameCounter = 0;

  while(1)
  {
    if(!captVidSrc.read(frame))
    {
      cout << "All frames read or error reading a frame" << endl;
      break;
    }

    latitudePath[frameCounter] = latitudeStart +  ((latitudeEnd - latitudeStart) / captVidSrc.get(CAP_PROP_FRAME_COUNT)) * frameCounter;
    longitudePath[frameCounter] = longitudeStart +  ((longitudeEnd - longitudeStart) / captVidSrc.get(CAP_PROP_FRAME_COUNT)) * frameCounter;

    #if DEBUG_CAMERA_PATH
      cout << "latPath" << frameCounter << ":  " << latitudePath[frameCounter] << endl;
      cout << "longPath" << frameCounter << ": " << longitudePath[frameCounter] << endl << endl;
    #endif

    // GPS Data
    

    // Position variables

    // Coordinate variables
    long double distanceOfBaseline;
    long double distanceOfSideline;
    long double distanceOfBaselineCenterNorth;
    long double distanceOfBaselineCenterEast;
    long double distanceOfSidelineNorth;
    long double distanceOfSidelineEast;
    int pixelPositionEast;
    int pixelPositionNorth;

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

        pixelPositionEast = longitudePath[frameCounter] + ((pixelDistanceEast / 1000000) / 111.32);
        pixelPositionNorth = latitudePath[frameCounter] + ((pixelDistanceNorth / 1000000) / 111.32);

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

      long double longitudeLeftPoint = longitudePath[frameCounter] + distanceOfBaselineCenterEastGPS + distanceOfSidelineEastGPS;
      long double latitudeLeftPoint = latitudePath[frameCounter] + distanceOfBaselineCenterNorthGPS + distanceOfSidelineNorthGPS;

      #if BASH_OUTPUT
        cout << "longitude: " << longitudePath[frameCounter] << endl;
        cout << "latitude:  " << latitudePath[frameCounter] << endl;
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

      long double longitudeRightPoint = longitudePath[frameCounter] + distanceOfBaselineCenterEastGPS + distanceOfSidelineEastGPS;
      long double latitudeRightPoint = latitudePath[frameCounter] + distanceOfBaselineCenterNorthGPS + distanceOfSidelineNorthGPS;

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

      long double steppingWidthEast = (deviationEast / (long double) frameWidth);
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
        pixelPositionEast = (int)((longitudeLeftPoint + column * steppingWidthEast) * 1000000);
        pixelPositionNorth = (int)((latitudeLeftPoint + column * steppingWidthNorth) * 1000000);
        #if BASH_OUTPUT
          cout << "East: " << pixelPositionEast << "\tNorth: " << pixelPositionNorth <<  endl;
          if (column > 10)
            return 0;
        #endif
        #if CSV_OUTPUT
          cout << row << ";" << column<< ";" << distanceOfBaseline << ";" << pixelPositionEast << ";" << pixelPositionNorth << endl;
        #endif

        /*** Compare image position ***/
        bool match = comparePositionToLineMark(pixelPositionEast, pixelPositionNorth, lineMark, markingSize);
        #if DEBUG_MARKING_CSV
          cout << column << ";" << row << ";" << pixelPositionNorth << ";" << pixelPositionEast << endl;
        #endif
        if(match == true)
        {
          int x = frameHeight - row;
          int y = column;
          // cout << x << " / " << y << endl;
          frame.at<Vec3b>(x, y) = Vec3b(0, 0, 255);
          // frame.at<Vec3b>(column, row) = Vec3b(0, 0, 255);
        }
      }
    }
#endif  /**** SOLUTION 2 ****/

    frameCounter++;

    #if DEBUG_TIME
      auto end = chrono::high_resolution_clock::now();
      auto dur = end - begin;
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
      cout << ms << endl;
    #endif

    imshow(WIN_SRC, frame);


    #if CSV_OUTPUT | DEBUG_MARKING_CSV
      waitKey(0);
      return 0;
    #endif


    if(waitKey(1) >= 0)
    {
      break;
    }

    #if STORE_FRAMES
      string frameName = "./Frames/VideoFrame";
      frameName.append(to_string(frameCounter));
      frameName.append(".png");
      imwrite(frameName, frame);
    #endif
  }
  
  #endif
  delete[] lineMark;
  delete[] gpsPoint;
  cout << "Mem cleared" << endl;
  return 0;
}