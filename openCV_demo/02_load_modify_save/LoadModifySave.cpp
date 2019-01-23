#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main (int argc, char ** argv)
{
    String imageName = "./../SampleImage.jpg";
    Mat image;

    image = imread(imageName, IMREAD_COLOR);

    Mat gray_image;
    cvtColor(image, gray_image, COLOR_BGR2GRAY);
    
    imwrite("./cvt_image.jpg", gray_image);

    namedWindow(imageName, WINDOW_AUTOSIZE);
    namedWindow("Gray Image", WINDOW_AUTOSIZE);

    imshow(imageName, image);
    imshow("Gray Image", gray_image);

    waitKey(0);
    return 0;
}