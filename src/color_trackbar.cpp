/*
  script for identifying HSV threshold values
  for different colors
*/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  if(argc != 2)
  {
    cout << "Usage: color_trackbar ImageToLoad" << endl;
    return -1;
  }
  
  Mat image = imread(argv[1], CV_LOAD_IMAGE_COLOR);

  if(!image.data)
  { 
    cout << "Could not open the Image" << endl;
    return -1;
  }

  Mat hsv;
  cvtColor(image, hsv, CV_BGR2HSV);

  int hh = 0, hl = 0, sh = 0, sl = 0, vh = 0, vl = 0;

  string windowName = "Image";
  namedWindow(windowName);

  createTrackbar("hh", windowName, &hh, 255);
  createTrackbar("hl", windowName, &hl, 255);
  createTrackbar("sh", windowName, &sh, 255);
  createTrackbar("sl", windowName, &sl, 255);
  createTrackbar("vh", windowName, &vh, 255);
  createTrackbar("vl", windowName, &vl, 255);

  Mat mask;
  int key = 0;
  do
  {
      Mat out;
      inRange(hsv, Scalar(hl, sl, vl), Scalar(hh, sh, vh), mask);
      bitwise_and(image, image, out, mask);

      imshow(windowName, out);
      key = waitKey(33);
  } while((char)key != 27);

  waitKey();

  return 0;
}
