#pragma once

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cv.h>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

// This struct stores all required informations of a specific consensus set.
struct Consensus
{
  // Random points, used for calculating this consensus set.
  vector<Point2f> random_points; 
  // Coefficients (a, b, c, d) of the polynom passing through the four random points.
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
  // Standard constructor.
  Ransac();
  // Standard destructor.
  ~Ransac();
  // Method to assign new data points and also sets x_min_dataset_ and x_max_dataset_;
  void setRansacDataSet(vector<cv::Point2f> &data_set);
  // Method to set the parameters of the RANSAC Algorithm (threshholds, number of iterations,...).
  void setRansacParams(float max_inlier_distance, int max_num_of_iterations, int min_size_consensus);
  // Method which does the RANSAC algorithm and returns the found polynomial parameters -->Master function.
  vector<float> getRansacCoeff();
  // Method to clear up for the next fitting.
  void clearUp();

  private:
  // PRIVATE MEMBER METHODS.
  // Method which choses a random points from the data set.
  Point2f getRandomPoint();
  // Method which takes exactly the required number of points fits the polynom (-->fully determined) and returns the coefficients.
  vector<float> getCoeffDet(vector<Point2f> det_points);  
  // Method which returns the absolute distance from a given polynomial.
  float getDistancePointToPolynom(Point2f point, vector<float> polynom_coeff);
  // Method, which takes more than the required number of points, fits the polynom using LSQ (-->over determined) and returns the parameters.
  vector<float> getCoeffLSQ(vector<Point2f> over_det_points); 
  // Write the used consensus set to a text file.
  void writeLargestConsSetToFile();


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
  // The consensus set with the most points.
  Consensus largest_consensus_set_;
};
