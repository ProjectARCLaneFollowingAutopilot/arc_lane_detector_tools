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
void Ransac::setRansacDataSet(vector<cv::Point2f> &data_set)
{
	// Copy the dataset.
	this->data_set_ = data_set;
	int x_min_dataset = 10000;
	int x_max_dataset = -1000;
	// Find the minimal and maximal x-coordinates in the data set.
	for(int i = 0; i<data_set.size(); i++)
	{
		if((this->data_set_)[i].x < x_min_dataset)
		{
			x_min_dataset = (this->data_set_)[i].x;
		}
		else if((this->data_set_)[i].x > x_max_dataset)
		{
			x_max_dataset = (this->data_set_)[i].x;

		}
	}
	// Set the found values.
	this->x_min_dataset_ = x_min_dataset;
	this->x_max_dataset_ = x_max_dataset;
}

// Method to set the parameters of the RANSAC Algorithm (threshholds, number of iterations,...).
void Ransac::setRansacParams(float max_inlier_distance, int max_num_of_iterations, int min_size_consensus)
{
	this->max_inlier_distance_ = max_inlier_distance;
	this->max_num_of_iterations_ = max_num_of_iterations;
	this->min_size_consensus_ = min_size_consensus;
}

// NOCH NICHT FERTIG: Method which does the RANSAC algorithm and returns the found polynomial parameters -->Master function.
vector<float> Ransac::getRansacCoeff()
{
	// Loop as many times as previously defined to generate the consensus sets.
	for(int i = 0; i < this->max_num_of_iterations_; i++)
	{
		// Create a Consensus struct to store the data of the current iteration.
		Consensus consensus_set_temp;

		// FEHLT NOCH: Do a random sample to find three points from the data set. 
		// Fit a first guess using the randomly chosen points.
		consensus_set_temp.polynom_coeff = Ransac::getCoeffDet(consensus_set_temp.random_points);  

		// Iterate through the whole data set and find the inliers and create a new Consensus variable to store.
		for(int j = 0; j<this->data_set_.size(); j++)
		{
			float distance = Ransac::getDistancePointToPolynom(this->data_set_[j], temp_coeff);
			if(distance < this->max_inlier_distance_)
			{
				consensus_set_temp.cons_set.pushback(this->data_set_[j]);
			}
			consensus_set_temp.size_cons_set = consensus_set_temp.cons_set.size();
		}
		// Append the consensus set vector with the just found consensus set struct.
		if(consensus_set_temp.size_cons_set > this->min_size_consensus_)
		{
			this->all_cons_sets_.pushback(consensus_set_temp);
		}
	}

	// Find the largest consensus set.
	int size_of_largest_cons = 0;
	for(int j = 0; i<this->all_cons_sets_.size();j++)
	{
		if(this->all_cons_sets_.size_cons_set > size_of_largest_cons)
		{
			this->largest_consensus_set_ = all_cons_sets_[j];
			size_of_largest_cons = this->all_cons_sets_.size_cons_set;
		}
	}

	// Do least square to find the final coefficients.

	// Return the found coefficients.

}

// PRIVATE MEMBER METHODS.

 // FEHLT NOCH: Method which takes exactly the required number of points fits the polynom (-->fully determined) and returns the coefficients.
 vector<float> Ransac::getCoeffDet(vector<cv::Point2f> det_points)
 {

 }



// Method which returns the absolute distance from a given polynomial. 
// !!! Effizienter wäre es für einen random set an Punkten das Polynom nur einmal zu diskretisieren.
float Ransac::getDistancePointToPolynom(Point2f point, vector<float> polynom_coeff)
{
	// Iterate discretely through the polynomial to find the nearest point to the curve.
	float resolution = 0.1;
	vector<cv::Point2f> discrete_polynomial;
	float minimal_distance = 1000.0;
	for(float x = this->x_min_dataset_; x<this->x_max_dataset_; x = x + resolution)
	{
		float y = polynom_coeff[0]*pow(x,2) + polynom_coeff[1]*x + polynom_coeff[2];
		// Evaluate polynomial at a discrete point.
		cv::Point2f temp_coord(x, y);
		// Calculate absolute distance to given point.
		float distance = sqrt(pow(temp_coord.x - point.x,2) + pow(temp_coord.y - point.y,2));
		// Decide if smallest distance have been found.
		if(distance<minimal_distance)
		{
			minimal_distance = distance;
		}
	}
	// Return the distance to the nearest point.
	return minimal_distance;
}
