#include "../../include/ransac_fitting/ransac_fitting.hpp"

// PUBLIC MEMBER METHODS.
// Standard constructor.
Ransac::Ransac()
{
	std::cout<<"RANSAC Object created!"<<std::endl;
}
// Standard destructor.
Ransac::~Ransac()
{
	std::cout<<"RANSAC Object destroyed!"<<std::endl;

}
// Method to assign new data points.
void Ransac::setRansacDataSet(vector<Vec2f> &data_set)
{
	this->data_set_ = data_set;
}

// Method to set the parameters of the RANSAC Algorithm (threshholds, number of iterations,...).
void Ransac::setRansacParams(float max_inlier_distance, int max_num_of_iterations, int min_size_consensus)
{
	this->max_inlier_distance_ = max_inlier_distance;
	this->max_num_of_iterations_ = max_num_of_iterations;
	this->min_size_consensus_ = min_size_consensus;
}

// Method which does the RANSAC algorithm and returns the found polynomial parameters -->Master function.
vector<float> Ransac::getRansacCoeff()
{
	// Loop as many times as previously defined to generate the consensus sets.
	for(int i = 0; i < this->max_num_of_iterations_; i++)
	{
		// Do a random sample to find three points from the data set.

		// Fit a first guess using the randomly chosen points.

		// Iterate through the whole data set and find the inliers and create a new Consensus variable to store.

		// Append the consensus set vector with the just found consensus set.

	}

	// Find the largest consensus set.

	// Do least square to find the final coefficients.

	// Return the found coefficients.

}

// PRIVATE MEMBER METHODS.