#pragma once
/* DESCRIPTION:
Use this class to fit a polynomial or spline to a set of points.
Input shall be: Set of points, (type of function to fit), number of iterations, thresholds
Output shall be: parameters of the fitted curve/function.

At the beginning: Only implement one curve type: 2nd order polynomial: f(x) = a*x² + b*x + c
*/
#include <cv.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"


// A new struct would be helpful: Stores parameters (Actually not needed) and the consensus set and size of the consensus set.

class RANSAC
{
  public:
  // PUBLIC MEMBER METHODS.
  RANSAC();
  ~RANSAC();
  // Some method to assign new data points.
  setRansacDataSet();
  // Method to set the parameters of the RANSAC Algorithm (threshholds, number of iterations,...).
  setRansacParams(float inlier_distance, int num_of_iterations, int min_size_consensus);
  // Method which does the RANSAC algorithm and returns the found polynomial parameters.
  // PUBLIC MEMBER VARIABLES.


  private:
  // PRIVATE MEMBER METHODS.
  // Method which choses three random points from the data set.

  // Method which solves system of linear equation to find the temporary parameters of the ongoing iteration and saves them.

  // Method which creates a new struct (in the vector defined below), and iterates through whole dataset to create consensus set.
  
  // Method which returns the distance from the temporary curve to a point.

  // Method, which takes a subset of the dataset (largest consensus set), fits the polynom and returns the final parameters.
  // PRIVATE MEMBER VARIABLES.
  // Vector which stores the data set of 2D pixel points.

  // Vector which stores, temporarily, the polynom parameters of the ongoing iteration.
  // Vector of structs, storing the consensus set, parameters and size of consensus set.
  // The maximum allowed distance (define distance!) from the curve, such that point counted as inlier.
  float inlier_distance_; 
  // Number of maximal iterations to find fitting points.
  int num_of_iterations_;
  // Size of the consensus set, to be counted as a valable consensus set.
  int min_size_consensus_;
};
