#pragma once
/* DESCRIPTION:
Use this class to fit a polynomial or spline to a set of points.
Input shall be: Set of points, type of function to fit, number of iterations, thresholds
Output shall be: parameters of the fitted curve/function.

At the beginning: Only implement one curve type: 3rd order polynomial: f(x) = a*x³ + b*x² + c*x + d
*/
#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"

class RANSAC
{
  public:
  // PUBLIC MEMBER METHODS.
  RANSAC();
  ~RANSAC();
  // Method to set the parameters of the RANSAC Algorithm (threshholds, number of iterations,...).
  // Some method to assign new data points.
  // Method which returns the fitting points and the parameters of the 3rd order polynomial.
  // PUBLIC MEMBER VARIABLES.


  private:
  // PRIVATE MEMBER METHODS.
  // Method which choses three random points, fits the 3rd order polynomial and returns the parameters and the error.
  // Method which does a loop for finding the ideal fitting points and parameters and saves them.
  // PRIVATE MEMBER VARIABLES.
  // Number of maximal iterations to find fitting points.
  // Vector which stores 2D Vectors corresponding to the x,y points.
  // Threshold for the cross error.
};
