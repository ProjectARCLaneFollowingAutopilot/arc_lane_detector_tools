#pragma once
/* DESCRIPTION:
Use this class to fit a 2nd order polynomial to a set of points.
Input shall be: Set of points, number of iterations, thresholds.
Output shall be: Parameters of the fitted function.
*/

#include <cv.h>
#include <iostream>
#include <vector>
#include "opencv2/highgui/highgui.hpp"


// This struct stores all required informations of a specific consensus set.
struct Consensus
{
  // Random points, used for calculating this consensus set.
  vector<Vec2f> random_points(3);
  // Coefficients (a, b, c) of the polynom passing through the three random points.
  std::vector<float> polynom_coeff(3); 
  // Consensus set.
  vector<Vec2f> cons_set;
  // Size of the consensus set.
  int size_cons_set;
};

class Ransac
{
  public:
  // PUBLIC MEMBER METHODS.
  Ransac();
  ~Ransac();
  // Some method to assign new data points.
  setRansacDataSet(vector<Vec2f> &data_set);
  // Method to set the parameters of the RANSAC Algorithm (threshholds, number of iterations,...).
  setRansacParams(float inlier_distance, int num_of_iterations, int min_size_consensus);
  // Method which does the RANSAC algorithm and returns the found polynomial parameters -->Master function.
  vector<float> getRansacCoeff();
  // PUBLIC MEMBER VARIABLES.

  private:
  // PRIVATE MEMBER METHODS.
  // Method which choses a random points from the data set.
  cv::Point2f getRandomPoint();
  // Method which takes exactly the required number of points fits the polynom (-->fully determined) and returns the coefficients.
  std::vector<float> getCoeffDet(vector<Vec2f> det_points);  
  // Method which returns the absolute distance from a given polynomial.
  float getDistancePointToPolynom(cv::Point2f point, vector<Vec2f> polynom_coeff);
  // Method, which takes more than the required number of points, fits the polynom using LSQ (-->over determined) and returns the parameters.
  std::vector<float> getCoeffLSQ(vector<Vec2f> over_det_points); 
  // PRIVATE MEMBER VARIABLES.
  // The maximum allowed distance (define distance!) from the curve, such that point counted as inlier.
  float max_inlier_distance_; 
  // Number of maximal iterations to find fitting points.
  int max_num_of_iterations_;
  // Size of the consensus set, to be counted as a valable consensus set.
  int min_size_consensus_;


  // Vector which stores the data set of 2D pixel points.
  vector<Vec2f> data_set_;
  // Vector of structs, storing all of the consensus set structs.
  vector<Consensus> all_cons_sets_;
};
