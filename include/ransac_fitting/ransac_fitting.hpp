#pragma once
/* DESCRIPTION:
Use this class to fit a 2nd order polynomial to a set of points.
Input shall be: Set of points, number of iterations, thresholds.
Output shall be: Parameters of the fitted function.
*/

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cv.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
//cv::Point2f init_points[4];


// This struct stores all required informations of a specific consensus set.
struct Consensus
{
  // Random points, used for calculating this consensus set.
  vector<Point2f> random_points; 
  // Coefficients (a, b, c) of the polynom passing through the three random points.
  vector<float> polynom_coeff;
  // Consensus set.
  vector<Point2f> cons_set;
  // Size of the consensus set.
  int size_cons_set;
};

class Ransac
{
  public:
  // PUBLIC MEMBER METHODS.
  // DONE & TESTED: Standard constructor.
  Ransac();
  // DONE & TESTED: Standard destructor.
  ~Ransac();
  // DONE & TESTED: Method to assign new data points and also sets x_min_dataset_ and x_max_dataset_;
  void setRansacDataSet(vector<cv::Point2f> &data_set);
  // DONE & TESTED: Method to set the parameters of the RANSAC Algorithm (threshholds, number of iterations,...).
  void setRansacParams(float max_inlier_distance, int max_num_of_iterations, int min_size_consensus);
  // DONE: Method which does the RANSAC algorithm and returns the found polynomial parameters -->Master function.
  vector<float> getRansacCoeff();

  // PUBLIC MEMBER VARIABLES.

  private:
  // PRIVATE MEMBER METHODS.
  // DONE & TESTED: Method which choses a random points from the data set.
  Point2f getRandomPoint();
  // DONE & TESTED: Method which takes exactly the required number of points fits the polynom (-->fully determined) and returns the coefficients.
  vector<float> getCoeffDet(vector<Point2f> det_points);  
  // DONE & TESTED: Method which returns the absolute distance from a given polynomial.
  float getDistancePointToPolynom(Point2f point, vector<float> polynom_coeff);
  // DONE & TESTED: Method, which takes more than the required number of points, fits the polynom using LSQ (-->over determined) and returns the parameters.
  vector<float> getCoeffLSQ(vector<Point2f> over_det_points); 
  // PRIVATE MEMBER VARIABLES.
  // The maximum allowed distance (define distance!) from the curve, such that point counted as inlier.
  float max_inlier_distance_; 
  // Number of maximal iterations to find fitting points.
  int max_num_of_iterations_;
  // Size of the consensus set, to be counted as a valable consensus set.
  int min_size_consensus_;
  // Minimal x-coordinate which is apparent in the data set (required for discrete shortest distance finder).
  float x_min_dataset_;
  // Maximal x-coordinate which is apparent in the data set (required for discrete shortest distance finder).
  float x_max_dataset_;

  // Vector which stores the data set of 2D pixel points.
  vector<Point2f> data_set_;
  // Vector of structs, storing all of the consensus set structs.
  vector<Consensus> all_cons_sets_;
  Consensus largest_consensus_set_;
};
