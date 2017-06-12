#include "../../include/ransac_fitting/ransac_fitting.hpp"

// PUBLIC MEMBER METHODS.
// DONE: Standard constructor.
Ransac::Ransac()
{
	std::srand(std::time(0)); // use current time as seed for random generator
	std::cout<<"RANSAC Object created!"<<std::endl;
}
// DONE: Standard destructor.
Ransac::~Ransac()
{
	std::cout<<"RANSAC Object destroyed!"<<std::endl;

}
// DONE & TESTED: Method to assign new data points.
void Ransac::setRansacDataSet(vector<cv::Point2f> &data_set)
{
	// Copy the dataset.
	this->data_set_ = data_set;
	float x_min_dataset = 10000;
	float x_max_dataset = -1000;
	// Find the minimal and maximal x-coordinates in the data set.
	for(int i = 0; i < this->data_set_.size(); i++)
	{
		if(this->data_set_[i].x < x_min_dataset)
		{
			x_min_dataset = this->data_set_[i].x;
		}
		else if(this->data_set_[i].x > x_max_dataset)
		{
			x_max_dataset = (this->data_set_)[i].x;
		}
	}
	// Set the found values.
	this->x_min_dataset_ = x_min_dataset;
	this->x_max_dataset_ = x_max_dataset;
}

// DONE & TESTED: Method to set the parameters of the RANSAC Algorithm (threshholds, number of iterations,...).
void Ransac::setRansacParams(float max_inlier_distance, int max_num_of_iterations, int min_size_consensus)
{
	this->max_inlier_distance_ = max_inlier_distance;
	this->max_num_of_iterations_ = max_num_of_iterations;
	this->min_size_consensus_ = min_size_consensus;
}

// DONE: Method which does the RANSAC algorithm and returns the found polynomial parameters -->Master function.
vector<float> Ransac::getRansacCoeff()
{
	// Loop as many times as previously defined to generate the consensus sets.
	for(int i = 0; i < this->max_num_of_iterations_; i++)
	{
		// Create a Consensus struct to store the data of the current iteration.
		Consensus consensus_set_temp;

		// Do a random sample to find three points from the data set. 
		for(int k = 0; k<3; k++)
		{
			Point2f random_point = this->Ransac::getRandomPoint();
			consensus_set_temp.random_points.push_back(random_point);
		}
		// Fit a first guess using the randomly chosen points.
		consensus_set_temp.polynom_coeff = this->Ransac::getCoeffDet(consensus_set_temp.random_points);  

		// Iterate through the whole data set and find the inliers and create a new Consensus variable to store.
		for(int j = 0; j < this->data_set_.size(); j++)
		{
			float distance = Ransac::getDistancePointToPolynom(this->data_set_[j], consensus_set_temp.polynom_coeff);
			if(distance < this->max_inlier_distance_)
			{
				consensus_set_temp.cons_set.push_back(this->data_set_[j]);
			}
			consensus_set_temp.size_cons_set = consensus_set_temp.cons_set.size();
		}
		// Append the consensus set vector with the just found consensus set struct.
		if(consensus_set_temp.size_cons_set > this->min_size_consensus_)
		{
			this->all_cons_sets_.push_back(consensus_set_temp);
		}
	}

	// Find the largest consensus set.
	int size_of_largest_cons = 0;
	for(int j = 0; j<this->all_cons_sets_.size();j++)
	{
		if(this->all_cons_sets_[j].size_cons_set > size_of_largest_cons)
		{
			this->largest_consensus_set_ = all_cons_sets_[j];
			size_of_largest_cons = this->all_cons_sets_[j].size_cons_set;
		}
	}

	// Return the found coefficients.
	return this->Ransac::getCoeffLSQ(this->largest_consensus_set_.cons_set); 
}

 // ONLY FOR TESTING.
 void Ransac::testFunction()
 {
	vector<Point2f> test_det;
 	for(int i = 0; i<15; i++)
 	{
 		Point2f test_punkt = this->getRandomPoint();
 		test_det.push_back(test_punkt);
 	}
	/*
 	vector<float> coeffs = this->Ransac::getCoeffDet(test_det);
 	std::cout<<"a: " <<coeffs[0]<<std::endl;
 	std::cout<<"b: " <<coeffs[1]<<std::endl;
 	std::cout<<"c: " <<coeffs[2]<<std::endl;
 	*/
 	vector<float> coeffs = this->Ransac::getCoeffLSQ(test_det); 
 	std::cout<<"a: " <<coeffs[0]<<std::endl;
 	std::cout<<"b: " <<coeffs[1]<<std::endl;
 	std::cout<<"c: " <<coeffs[2]<<std::endl;
 }


// PRIVATE MEMBER METHODS.

// DONE & TESTED: Method which choses a random points from the data set.
Point2f Ransac::getRandomPoint()
{
	int random_index = std::rand() % this->data_set_.size();  
	return this->data_set_[random_index];
}


 // DONE & TESTED: Method which takes exactly the required number of points fits the polynom (-->fully determined) and returns the coefficients.
 // !!!Eigentlich könnte diese Funktion auch mit getCoeffLSQ gemacht werden!!!
 vector<float> Ransac::getCoeffDet(vector<Point2f> det_points)
 {
 	Eigen::Matrix3f A;
  	Eigen::Vector3f b;
  	A << pow(det_points[0].x,2),det_points[0].x,1,  pow(det_points[1].x,2),det_points[1].x,1,  pow(det_points[2].x,2),det_points[2].x,1;
  	b << det_points[0].y, det_points[1].y, det_points[2].y;
  	std::cout << "Here is the matrix A:\n" << A << std::endl;
	std::cout << "Here is the vector b:\n" << b << std::endl;
  	Eigen::Vector3f coeff = A.colPivHouseholderQr().solve(b);
  	vector<float> coeff_copy;
  	coeff_copy.push_back(coeff[0]);
  	coeff_copy.push_back(coeff[1]);
  	coeff_copy.push_back(coeff[2]);
  	return coeff_copy;
 }

// DONE & TESTED: Method which returns the absolute distance from a given polynomial. 
// !!! Effizienter wäre es für einen random set an Punkten das Polynom nur einmal zu diskretisieren.
float Ransac::getDistancePointToPolynom(Point2f point, vector<float> polynom_coeff)
{
	// Iterate discretely through the polynomial to find the nearest point to the curve.
	float resolution = 0.1;
	float minimal_distance = 1000.0;
	for(float x = this->x_min_dataset_; x < this->x_max_dataset_; x = x + resolution)
	{
		// Evaluate polynomial at a discrete point --> y(x).
		float y = polynom_coeff[0]*pow(x,2) + polynom_coeff[1]*x + polynom_coeff[2];
		// Save point on polynomial.
		Point2f temp_coord(x, y);
		// Calculate absolute distance to given point.
		float distance = sqrt(pow(temp_coord.x - point.x,2) + pow(temp_coord.y - point.y,2));
		// Decide if a smaller distance has been found.
		if(distance < minimal_distance)
		{
			minimal_distance = distance;
		} 
	}
	// Return the distance to the nearest point.
	return minimal_distance;
}

 // DONE & TESTED: Method, which takes more than the required number of points, fits the polynom using LSQ (-->over determined) and returns the parameters.
 vector<float> Ransac::getCoeffLSQ(vector<Point2f> over_det_points)
 {
 	// Create matrix A.
 	Eigen::MatrixXf A(over_det_points.size(),3);
 	// Create matrix A'.
 	Eigen::MatrixXf A_trans(3,over_det_points.size());
 	// Create matrix L.
 	Eigen::MatrixXf L(3,3);
 	// Create matrix R.
 	Eigen::MatrixXf R(3,over_det_points.size());
 	// Create matrix b.
 	Eigen::VectorXf b(over_det_points.size());
 	// Fill Matrix A and Vector b.
 	for(int i = 0; i<over_det_points.size(); i++)
 	{
 		float x = over_det_points[i].x;
 		float y = over_det_points[i].y;
 		// Assign.
 		A(i, 0) = pow(x,2);
 		A(i, 1) = x;
 		A(i, 2) = 1;
 		b[i] = y;
 	}

 	// Transpose A.
 	A_trans =  A.transpose();
 	// Calculate L.
 	L = A_trans*A;
 	// Calculate R.
 	R = A_trans*b;

   	Eigen::Vector3f coeff = L.colPivHouseholderQr().solve(R);

   	vector<float> coeff_copy;

   	for(int j = 0; j < 3; j++)
   	{
   		coeff_copy.push_back(coeff[j]);
   	}

   	return coeff_copy;
 } 
