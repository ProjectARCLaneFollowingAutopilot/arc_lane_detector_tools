#pragma once
/* DESCRIPTION:
This class offers method to find lines in an image, using different image processing methods.
However, all are based on the hough line transform, but the images are preprocessed differently.
*/
#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <cv.h>

class LINEDETECTOR
{
  public:
  // PUBLIC MEMBER METHODS.
  // Standard constructor.
  LINEDETECTOR();
  // Standard destructor.
  ~LINEDETECTOR();
  // Does the Hough-Transform and draws the lines.
  void houghTransform(Mat &contours, Mat &draw_to, vector<Vec2f> &lines_hT, int threshold);

  
  // PUBLIC MEMBER VARIABLES.


  private:
  // PRIVATE MEMBER METHODS.
  // Line finding methods:
  vector<Vec2f> HoughClassic (Mat &src_HC);
  vector<Vec2f> InRange (Mat src_IR);
  vector<Vec2f> GrayProperty (Mat src_GP);
  vector<Vec2f> CompareGray (Mat src_CG);

  // Helper methods:
  Mat showChannel(Mat RGB, bool B, bool G, bool R);
  Mat RoadThreshold(Mat src_RT);
  Vec3b IntensityOfArea(Mat &src_IOA, int x_gray, int y_gray, int width_gray, int height_gray);

  // PRIVATE MEMBER VARIABLES.
  
};
