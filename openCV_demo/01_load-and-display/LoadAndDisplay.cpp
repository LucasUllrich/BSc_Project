#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main (int argc, char ** argv)
{
  String imageName("./../SampleImage.jpg");
  Mat image;

  image = imread(imageName, IMREAD_COLOR);
  if (image.empty() == 1)
  {
    cout << "Could not open image" << std::endl;
    return -1;
  }

  namedWindow("Loaded Image", WINDOW_AUTOSIZE);
  imshow("Loaded Image", image);

  waitKey(0);
  return 0;
}
