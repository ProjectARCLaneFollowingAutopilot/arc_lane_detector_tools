#pragma once

#include <cmath>
#include <cv.h>
#include <Eigen/Dense>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include "../../include/constants.hpp"

using namespace cv;

// GENERAL FUNCTIONS.
// Callback function for setMouseCallback and returns the point clicked on.
void getClickedPixel(int event, int x, int y, int flags, void *ptr);

class LineDetector
{
  public:
  // PUBLIC MEMBER METHODS.
  // Standard constructor.
  LineDetector();
  // Standard destructor.
  ~LineDetector();
  // Define some parameters: ROI-Cropping corners.
  void setParams(Point2f roi_left_top, Point2f roi_right_bottom);
  // Set a new original and cropped image and also set default and initial values.
  void setImage(Mat &new_image);
  // Method which forces to detect lines, does filtering and saves two lines-->master function.
  void doLineDetection();
  // Get the coordinates (LB, LT, RB, RT) of the two lines in the original image.
  vector<Point2f> getLineCoordinates();
  // Method which clears variables for a next image.
  void clearUp();

  // Helper methods:
  // Does the Hough-Transform and draws the lines.
  void houghTransform(Mat &contours, Mat &draw_to, vector<Vec2f> &lines_ht, int threshold);
  
  private:
  // PRIVATE MEMBER METHODS.
  // Method which finds all lines in an image, using a combination of different line finding methods.
  vector<Vec2f> findAllLines(Mat &lines_to_find);
  // Line finding methods:
  // Method for generating Lines - First blurs the image, Canny edge detector and Hough transform.
  vector<Vec2f> houghClassic(Mat src_hc); 
  // Method for generating Lines - Compares the three RGB channels for emphasising the road, and Hough of Canny image.
  vector<Vec2f> grayProperty(Mat src_gp);
  // Method for generating Lines - Compares colour to the averaged colour of the road, and Hough of Canny image.
  vector<Vec2f> compareGray (Mat src_cg);
  // Heuristic filters.
  // Method which decides if values shall be resetted.
  void resetToDefault();
  // Method which filters all lines to find and set only two lines (old approach).
  void filterLines1();
  // Method which filters all lines to find and set only two lines (new approach).
  void filterLines2();

  // Helper methods:
  // Method to prompt the user to set four input control points, which are on the two lines and saves them.
  void setDefaultLines(Mat &new_image);
  // Functions emphasises the road by comparing the road colour to every pixel.
  Mat roadThreshold(Mat src_rt);
  // Function averages the intensity of a area for each colour channel.
  Vec3b intensityOfArea(Mat &src_ioa, int x_gray, int y_gray, int width_gray, int height_gray);
  // Filters an image binarily by comparing the different colour channels.
  Mat findGray(Mat src_fg);
  // Method which returns rho and theta of a line defined in the cartesian space.
  Eigen::Vector2f cartesian2PolarLines(float x_a, float x_b, float y_a, float y_b);
  // Method which transforms a point coordinate from original to cropped image.
  Point2f coordinateOrig2Crop(Point2f coord_orig);
  // Method which transforms a point coordinate from cropped to original image.
  Point2f coordinateCrop2Orig(Point2f coord_crop);
  // Show filtered hough lines in original image.
  void drawTwoLinesOriginal(Mat image_to_draw);
  // Displays the image in a window.
  void showImage(Mat show, string name);

  // PRIVATE MEMBER VARIABLES.
  // Counter to know if default lines have been set already.
  int init_counter_;  
  // Store parameters of two default lines left (cropped coordinates) --> Needed for resetting.
  Vec2f default_left_;
  // Store parameters of two default lines right (cropped coordinates) --> Needed for resetting.
  Vec2f default_right_;
  // The default value for the left line gradient angle.
  float alpha_default_;
  // The value for the right line gradient angle.
  float beta_default_;
  // How much is alpha allowed to jump?
  float del_alpha_limit_;
  // How much is beta allowed to jump?
  float del_beta_limit_;
  // Store coordinates of cropped image (top left and bottom right corner) in original coordinates.
  vector<Point2f> cropping_corners_;
  // Store original color image.
  cv::Mat original_; 
  // Store cropped color image (from original).
  cv::Mat cropped_;
  // Threshold after how many loops of not updating, reset shall be performed.
  int reset_trigger_;
  // Keep track for how many times, no left line update was performed --> Decide then if RESET needed.
  int update_counter_left_;
  // Keep track for how many times, no left right update was performed --> Decide then if RESET needed.
  int update_counter_right_;
  // Decide if left line shall be drawn in.
  bool draw_left_;
  // Decide if the right line shall be drawn in.
  bool draw_right_;
  
  // Store all of the detected lines in the cropped coordinate system.
  vector<Vec2f> all_lines_cropped_;
  // Store the left line in the original coordinate system.
  Vec2f left_line_orig_;
  // Store the right line in the original coordinate system.
  Vec2f right_line_orig_;
  // Store the left line in the cropped coordinate system.
  Vec2f left_line_cropped_;
  // Store the right line in the cropped coordinate system.
  Vec2f right_line_cropped_;
  // Save the gradient angle of the left line.
  float alpha_deg_;
  // Save the gradient angle of the right line.
  float beta_deg_;
};
