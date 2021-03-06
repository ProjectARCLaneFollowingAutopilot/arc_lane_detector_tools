#pragma once

#include <cmath>
#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "../../include/constants.hpp"

// GENERAL FUNCTIONS.
// Callback function for setMouseCallback and returns the point clicked on.
void getClickedPixel(int event, int x, int y, int flags, void *ptr);

class IPM
{
  public:
  // PUBLIC MEMBER METHODS.
  // Default constructor.
  IPM();
  // Default destructor.
  ~IPM();
  // Method to set parameters, such as extrinsic and intrinsic camera parameters and then finds the homography matrix.
  void setParam(float camera_height_m, float pitch_angle_deg, float focal_length_px, int input_width_px, int input_height_px);
  // Method to set a new input image.
  void getImage(cv::Mat src);
  // Method which does IPM and returns undistorted, projected image.
  cv::Mat invPerspectiveMapping();
  // Method to prompt the user to set input control points.
  void setCtrlPts();
  // Method which transforms a single point and returns the coordinates in the vehicle frame.
  cv::Point2f image2Local(cv::Point2f image_coordinate);

  private:
  // PRIVATE MEMBER METHODS.
  // Method which prompts the user to calibrate the transformation matrix by assigning four pixel each in two images image.
  void setTransformationMatrix();
  // Method which uses four predefined points on input image, uses equation (6) to project and then gets and sets the transformation matrix.
  // The input argument of type boolean is only needed for overloading the function.
  void setTransformationMatrix(bool some_variable);

  // PRIVATE MEMBER VARIABLES.
  // Input image, which is perspectively distorted.
  cv::Mat input_img_;
  // Output image, which is undistorted and projected on the ground plane.
  cv::Mat output_img_;
  // Height of camera above World-Origin/Ground-Plane (in meter).
  float camera_height_m_;
  // Pitch angle of camera w.r.t vertical link Camera<->World (in degree).
  float pitch_angle_deg_;
  // Focal length of the camera (in pixels), assume that fx=fy.
  float focal_length_px_;
  // Width of input image (in pixels).
  int input_width_px_;
  // Height of input image (in pixels).
  int input_height_px_;
  // Vector to store the points in the input image for finding the homography.
  cv::Point2f src_points_[4];
  // Vector to store the points in the output image for finding the homography.
  cv::Point2f dst_points_[4];
  // Matrix to store the homography matrix.
  cv::Mat perspective_transform_;
};