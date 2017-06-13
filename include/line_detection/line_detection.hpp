#pragma once
/* DESCRIPTION:
This class offers method to find lines in an image, using different image processing methods.
However, all are based on the hough line transform, but the images are preprocessed differently.
At the end, the algorithm should return only two lines.
*/
#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <vector>

using namespace cv;

class LineDetector
{
  public:
  // PUBLIC MEMBER METHODS.
  // Standard constructor.
  LineDetector();
  // Standard destructor.
  ~LineDetector();
  // Set a new original image.
  // Define all parameters: ROI-Cropping corners, two default lines (original coordinates), 
  // Does the Hough-Transform and draws the lines.
  void houghTransform(Mat &contours, Mat &draw_to, vector<Vec2f> &lines_hT, int threshold);
  // Function which returns ALL lines in the original coordinate system.
  // Function which returns ALL lines in the cropped coordinate system.
  // Method which deletes the lines vectors.
  
  // PUBLIC MEMBER VARIABLES.


  private:
  // PRIVATE MEMBER METHODS.
  // Line finding methods:
  // ???
  vector<Vec2f> HoughClassic (Mat &src_HC);
  // ???
  vector<Vec2f> InRange (Mat src_IR);
  // ???
  vector<Vec2f> GrayProperty (Mat src_GP);
  // ???
  vector<Vec2f> CompareGray (Mat src_CG);

  // Helper methods:
  // ???
  Mat showChannel(Mat RGB, bool B, bool G, bool R);
  // ???
  Mat RoadThreshold(Mat src_RT);
  // ???
  Vec3b IntensityOfArea(Mat &src_IOA, int x_gray, int y_gray, int width_gray, int height_gray);
  // ???
  Mat FindGray(Mat src);
  // Method which transforms rho and theta from cropped image (ROI) to original image.
  // Method which transforms rho and theta from original image to cropped image (ROI).
  // Function which does all kind of line detections for a given image and returns the lines.
  // Draws lines to an image (both have to have the same coordinate system!) by getting a vector with (rho, theta).

  // PRIVATE MEMBER VARIABLES.
  // Store parameters of two default lines (original coordinates) --> Needed for resetting.
  // Store coordinates of cropped image (top left and bottom right corner) in original coordinates.
  vector<Point2f> cropping_corners[2];
  // Store original color image.
  cv::Mat original_; 
  // Store cropped color image (from original).
  cv::Mat cropped_;

  // Store all of the detected lines in the original coordinate system.
};
