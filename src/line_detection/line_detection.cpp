#include "../../include/line_detection/line_detection.hpp"

// GENERAL FUNCTIONS.
// DONE & TESTED: Callback function for setMouseCallback and returns the point clicked on.
void getClickedPixel(int event, int x, int y, int flags, void *ptr)
{
  if(event == cv::EVENT_LBUTTONDOWN)
  {
    std::cout<<"CLICK"<<std::endl;
    cv::Point2f *p = (cv::Point2f*)ptr;
    p->x = x;
    p->y = y;
  }
}

// PUBLIC MEMBER METHODS.
// DONE & TESTED: Standard constructor.
LineDetector::LineDetector()
{
	std::cout<<"Line Detector created!"<<std::endl;
}
// DONE & TESTED: Standard destructor.
LineDetector::~LineDetector()
{
	std::cout<<"Line Detector destroyed!"<<std::endl;
}
// DONE: Define all parameters: ROI-Cropping corners...
void LineDetector::setParams(Point2f roi_left_top, Point2f roi_right_bottom)
{
  this->update_counter_left_ = 0;
	this->update_counter_right_ = 0;
	this->draw_left_ = 1;
	this->draw_right_ = 1;

	this->cropping_corners_.push_back(roi_left_top);
	this->cropping_corners_.push_back(roi_right_bottom);

	this->reset_trigger_ = 20;

  this->del_alpha_limit_ = 5;
  this->del_beta_limit_ = 5;

	this->init_counter_ = 0;
}
// DONE & TESTED: Set a new original and cropped image.
void LineDetector::setImage(Mat &new_image)
{
	// Reset all data from previous iteration.
	this->LineDetector::clearUp();
	// Set original image.
	this->original_ = new_image.clone();
	// Set cropped image.
  this->cropped_ = new_image(Rect(this->cropping_corners_[0].x, this->cropping_corners_[0].y, this->cropping_corners_[1].x - this->cropping_corners_[0].x, this->cropping_corners_[1].y - this->cropping_corners_[0].y));
  // If function gets called for the first time: Set the default lines.
	if(this->init_counter_ == 0)
	{   
		this->LineDetector::setDefaultLines(this->cropped_);
		this->init_counter_ += 1;
	}
}

 // WORK IN PROGRESS: Method which forces to detect lines, does filtering and saves two lines-->master function.
void LineDetector::doLineDetection()
{
	// Find lines in the cropped image and save them.
	this->all_lines_cropped_ =  this->LineDetector::findAllLines(this->cropped_);
	// Reset to default values if necessary.
	this->LineDetector::resetToDefault();
	// Filter the cropped lines out to only find two lines and save.
	this->LineDetector::filterLines2();
}

// DONE: Get the coordinates (LB, LT, RB, RT) of the two lines in the original image.
vector<Point2f> LineDetector::getLineCoordinates()
{
	// Calculate point coordinates (cartesian) where the lines cross the upper and lower horizontal limitation of the cropped image.
  	float y_top = 40.0;
  	float y_bottom = this->cropped_.rows;
  	float x_top_left = this->left_line_cropped_[0]*cos(this->left_line_cropped_[1]) - sin(this->left_line_cropped_[1])*((y_top - this->left_line_cropped_[0]*sin(this->left_line_cropped_[1]))/(cos(this->left_line_cropped_[1])));
  	float x_bottom_left = this->left_line_cropped_[0]*cos(this->left_line_cropped_[1]) - sin(this->left_line_cropped_[1])*((y_bottom - this->left_line_cropped_[0]*sin(this->left_line_cropped_[1]))/(cos(this->left_line_cropped_[1])));
  	float x_top_right = this->right_line_cropped_[0]*cos(this->right_line_cropped_[1]) - sin(this->right_line_cropped_[1])*((y_top - this->right_line_cropped_[0]*sin(this->right_line_cropped_[1]))/(cos(this->right_line_cropped_[1])));
  	float x_bottom_right = this->right_line_cropped_[0]*cos(this->right_line_cropped_[1]) - sin(this->right_line_cropped_[1])*((y_bottom - this->right_line_cropped_[0]*sin(this->right_line_cropped_[1]))/(cos(this->right_line_cropped_[1])));

  	Point2f left_top_dst;
  	Point2f left_bottom_dst;
  	Point2f right_top_dst;
  	Point2f right_bottom_dst;

  	left_top_dst.x = x_top_left;
  	left_top_dst.y = 290; 
  	left_bottom_dst.x = x_bottom_left;
  	left_bottom_dst.y = 420.0;
  	right_top_dst.x = x_top_right;
  	right_top_dst.y = 290;
  	right_bottom_dst.x = x_bottom_right;
  	right_bottom_dst.y = 420.0;

  	vector<Point2f> result;
  	result.push_back(left_bottom_dst);
  	result.push_back(left_top_dst);
  	result.push_back(right_bottom_dst);
  	result.push_back(right_top_dst);

  	return result;
}

// WORK IN PROGRESS: Method which clears variables for a next image.
void LineDetector::clearUp()
{
  	this->all_lines_cropped_.clear();
}

// Helper methods:
// DONE: Does the Hough-Transform, gives lines and draws the lines.
void LineDetector::houghTransform(Mat &contours, Mat &draw_to, vector<Vec2f> &lines_ht, int threshold)
{
  //Hough transform. Parameter to be determined.
  HoughLines(contours, lines_ht, 1, CV_PI/180, threshold, 0, 0); 
  //Iterate through all the lines and draw the line. 
  for(int i = 0; i < lines_ht.size(); i++)
  {
    float rho = lines_ht[i][0];
    float theta = lines_ht[i][1];
    Point pt1;
    Point pt2;
    double a = cos(theta);
    double b = sin(theta);
    double x0 = a*rho;
    double y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));
    line(draw_to, pt1, pt2, Scalar(0, 255, 0), 3, CV_AA);
  }
}
  
// PRIVATE MEMBER METHODS.
// WORK IN PROGRESS: Method which finds all lines in an image, using a combination of different line finding methods.
vector<Vec2f> LineDetector::findAllLines(Mat &lines_to_find)
{
	// Do several Hough transforms.
  	vector<Vec2f> h_c = houghClassic (lines_to_find);
  	vector<Vec2f> g_p = grayProperty(lines_to_find);
  	vector<Vec2f> c_g = compareGray (lines_to_find);

    imshow("lines to find", lines_to_find);
    waitKey(1);

  	// Append all vectors.
  	vector<Vec2f> all_detected_lines;
  	all_detected_lines.insert(all_detected_lines.end(), h_c.begin(), h_c.end());
  	all_detected_lines.insert(all_detected_lines.end(), g_p.begin(), g_p.end());
  	all_detected_lines.insert(all_detected_lines.end(), c_g.begin(), c_g.end());

	//  Return all lines.
	return all_detected_lines;
}

// Line finding methods:
// DONE: ???
vector<Vec2f> LineDetector::houghClassic(Mat src_hc)
{
  Mat src_hc_roi_filtered = src_hc.clone();
  medianBlur(src_hc, src_hc_roi_filtered, 5);
  Mat contours = src_hc_roi_filtered.clone();
  Canny(src_hc_roi_filtered, contours, 30, 50);
  Mat draw_detected_hough = src_hc.clone();
  vector<Vec2f> lines_hc;
  //Do HoughTransform.
  houghTransform(contours, draw_detected_hough, lines_hc, 80);
  return lines_hc;
}

// DONE: ???
vector<Vec2f> LineDetector::grayProperty(Mat src_gp)
{
  Mat src_gp_copy = src_gp.clone();
  vector<Vec2f> lines_gp;
  Mat contours(src_gp.rows, src_gp.cols, CV_8UC1);
  //Find gray areas using the function FindGray.
  Mat gray = this->LineDetector::findGray(src_gp_copy);
  Canny(gray, contours, 180, 180);
  //Sobel(gray, contours, CV_16S, 1, 0, 3);
  houghTransform(contours, src_gp_copy, lines_gp, 30);
  return lines_gp;
}

// DONE: ???
vector<Vec2f> LineDetector::compareGray(Mat src_cg)
{
  vector<Vec2f> lines_cg;
  Mat src_cg_copy = src_cg.clone();
  Mat color_contour = src_cg_copy.clone();
  //Calculate Canny-image by using the function roadThreshold;
  Canny(this->LineDetector::roadThreshold(src_cg_copy),  color_contour, 30, 50);
  Mat show_color_hough = src_cg_copy.clone();
  houghTransform(color_contour, show_color_hough, lines_cg, 30);
  return lines_cg;
}

// Heuristic filters:
// Method which decides if values shall be resetted.
void LineDetector::resetToDefault()
{
	// Check if any of the lines haven't been updated for a long time (always kept constant from previous frame).
  	// If any line wasn't updated for long time: Set that line back to default and a bool, such that this line doesn't get drawn in.
  	if((this->update_counter_left_ > this->reset_trigger_) && (this->update_counter_right_ > this->reset_trigger_))
  	{
    	// Reset both.
    	this->draw_left_ = 0;
    	this->draw_right_ = 0;
    	this->left_line_cropped_ = this->default_left_;
    	this->right_line_cropped_ = this->default_right_;
    	this->alpha_deg_ = this->alpha_default_;
    	this->beta_deg_ = this->beta_default_;
  	}
  	else if((this->update_counter_left_ > this->reset_trigger_) && (this->update_counter_right_ < this->reset_trigger_))
  	{
    	// Reset left.
    	this->draw_left_ = 0;
    	this->left_line_cropped_ = this->default_left_;
    	this->alpha_deg_ = this->alpha_default_;
	}
  	else if((this->update_counter_left_ < this->reset_trigger_) && (this->update_counter_right_ > this->reset_trigger_))
  	{
    	// Reset right.
    	this->draw_right_ = 0;
    	this->right_line_cropped_ = this->default_right_;
    	this->beta_deg_ = this->beta_default_;
  	}
}

// Method which filters all lines to find and set only two lines (new approach).
void LineDetector::filterLines2()
{
	// LAYER 1: Remove all lines passing through the VI sensor.
 	vector<Vec2f> lines = this->all_lines_cropped_;
 	vector<Vec2f> lines_temporary;
  	for(int i = 0; i < lines.size(); i++)
  	{ 
    	float x_middle = this->cropped_.cols/2.0;
    	float y_middle = (-cos(lines[i][1])/sin(lines[i][1]))*x_middle + (lines[i][0])/sin(lines[i][1]);
    	if(y_middle < 85)
    	{
      		lines_temporary.push_back(lines[i]);
    	}
  	}
  	lines = lines_temporary;
  	// LAYER 2: Split the remaining lines into left and right and remove jumping lines at the same time.
  	vector<Vec2f> lines_left;
  	vector<Vec2f> lines_right;
  	// Split the lines vector to a left and a right vector. Only assign those where the gradient angle is below a specific threshold.
  	if(lines.size() > 0)
  	{
    	for(int i = 0; i < lines.size(); i++)
    	{      
      		float top_crossing_x = lines[i][0]*cos(lines[i][1]) - sin(lines[i][1])*((0 - lines[i][0]*sin(lines[i][1]))/(cos(lines[i][1])));
      		float bottom_crossing_x = lines[i][0]*cos(lines[i][1]) - sin(lines[i][1])*((this->cropped_.rows - lines[i][0]*sin(lines[i][1]))/(cos(lines[i][1])));
      		// Assign left lines.
      		if((bottom_crossing_x < this->cropped_.cols/2.0) && (bottom_crossing_x > - 320))
      		{
        		float del_x = std::abs(top_crossing_x - bottom_crossing_x);
        		float del_y = std::abs(this->cropped_.rows);
        		float m = del_y/del_x;
        		float alpha = std::atan(m)*180.0/PI;
            std::cout<<"Alpha"<<alpha<<std::endl;
        		if((std::abs(alpha - this->alpha_deg_) < this->del_alpha_limit_))
        		{
          			lines_left.push_back(lines[i]);
        		}
      		}			
      		// Assign right lines.
     		else if((bottom_crossing_x > this->cropped_.cols/2.0) && (bottom_crossing_x < 960))
      	{
        		float del_x = std::abs(top_crossing_x - bottom_crossing_x);
        		float del_y = std::abs(this->cropped_.rows);
        		float m = del_y/del_x;
        		float beta = std::atan(m)*180.0/PI;
                        std::cout<<"Beta"<<beta<<std::endl;

        		if((std::abs(beta - this->beta_deg_) < this->del_beta_limit_))
        		{
          			lines_right.push_back(lines[i]);
        		}
      		}
    	}	 
	}

 	// LAYER 3: From the split up remaining lines find the best match for the left and right line.
  	// Case: Only left lines vector has elements.
  	if((lines_left.size() > 0) && (lines_right.size() == 0))
  	{
  		float cost_left = 100.0;
    	int index_minimal_cost_left = 0;
    	for(int i = 0; i < lines_left.size(); i++)
    	{	
    		// For each line calculate the relative error in rho.
      		float rel_error_rho_left_loop = (lines_left[i][0] - this->left_line_cropped_[0])/this->left_line_cropped_[0];
      		// For each line calculate the relative error in theta.
      		float rel_error_theta_left_loop = (lines_left[i][1] - this->left_line_cropped_[1])/this->left_line_cropped_[1];
      		// For each line calculate a cost function.
      		float cost_left_loop = sqrt(pow(rel_error_rho_left_loop, 2) + pow(rel_error_theta_left_loop, 2));
      		// If the cost is lower than the previous, save the line as the correct one.
      		if(cost_left_loop < cost_left)
      		{
        		index_minimal_cost_left = i;
        		cost_left = cost_left_loop;
      		}
    	}
    	// Update.
    	this->left_line_cropped_ = lines_left[index_minimal_cost_left];
    	this->update_counter_left_ = 0;
    	this->update_counter_right_ += 1;
    	this->draw_left_ = 1;
  	}
  	// Only right lines vector has elements.
	else if((lines_left.size() == 0) && (lines_right.size() > 0))
  	{
		float cost_right = 100.0;
    	int index_minimal_cost_right = 0;
    	for(int i = 0; i < lines_right.size(); i++)
    	{	
     		// For each line calculate the relative error in rho.
      		float rel_error_rho_right_loop = (lines_right[i][0] - this->right_line_cropped_[0])/this->right_line_cropped_[0];
      		// For each line calculate the relative error in theta.
      		float rel_error_theta_right_loop = (lines_right[i][1] - this->right_line_cropped_[1])/this->right_line_cropped_[1];
      		// For each line calculate a cost function.
      		float cost_right_loop = sqrt(pow(rel_error_rho_right_loop, 2) + pow(rel_error_theta_right_loop, 2));
      		// If the cost is lower than the previous, save the line as the correct one.
      		if(cost_right_loop < cost_right)
      		{
        		index_minimal_cost_right = i;
        		cost_right = cost_right_loop;
      		}
    	}
   		// Update.
    	this->right_line_cropped_ = lines_right[index_minimal_cost_right];
    	this->update_counter_right_ = 0;
    	this->update_counter_left_ += 1;
    	this->draw_right_ = 1;
	}
  	else if((lines_left.size()> 0) && (lines_right.size() > 0))
  	{
    	float cost_left = 100.0;
    	int index_minimal_cost_left = 0;
    	for(int i = 0; i < lines_left.size(); i++)
    	{
      		// For each line calculate the relative error in rho.
      		float rel_error_rho_left_loop = (lines_left[i][0] - this->left_line_cropped_[0])/this->left_line_cropped_[0];
      		// For each line calculate the relative error in theta.
      		float rel_error_theta_left_loop = (lines_left[i][1] - this->left_line_cropped_[1])/this->left_line_cropped_[1];
      		// For each line calculate a cost function.
      		float cost_left_loop = sqrt(pow(rel_error_rho_left_loop, 2) + pow(rel_error_theta_left_loop, 2));
      		// If the cost is lower than the previous, save the line as the correct one.
      		if(cost_left_loop < cost_left)
      		{
        		index_minimal_cost_left = i;
        		cost_left = cost_left_loop;
      		}
     	}
    	float cost_right = 100.0;
    	int index_minimal_cost_right = 0;
    	for(int i = 0; i < lines_right.size(); i++)
    	{
      		// For each line calculate the relative error in rho.
      		float rel_error_rho_right_loop = (lines_right[i][0] - this->right_line_cropped_[0])/this->right_line_cropped_[0];
      		// For each line calculate the relative error in theta.
      		float rel_error_theta_right_loop = (lines_right[i][1] - this->right_line_cropped_[1])/this->right_line_cropped_[1];
      		// For each line calculate a cost function.
      		float cost_right_loop = sqrt(pow(rel_error_rho_right_loop, 2) + pow(rel_error_theta_right_loop, 2));
      		// If the cost is lower than the previous, save the line as the correct one.
      		if(cost_right_loop < cost_right)
      		{
        		index_minimal_cost_right = i;
        		cost_right = cost_right_loop;
      		}
     	}
     	// Update.
     	this->left_line_cropped_ = lines_left[index_minimal_cost_left];
     	this->right_line_cropped_ = lines_right[index_minimal_cost_right];
     	this->update_counter_left_ = 0;
     	this->update_counter_right_ = 0;
     	this->draw_left_ = 1;
     	this->draw_right_ = 1;
  	}
  	else if((lines_left.size() == 0) && (lines_right.size() == 0))
  	{
    	this->update_counter_left_ += 1;
    	this->update_counter_right_ += 1;
  	}

  	// FINAL STEP: Assign new values.
    float top_crossing_x_left = this->left_line_cropped_[0]*cos(this->left_line_cropped_[1]) - sin(this->left_line_cropped_[1])*((0 - this->left_line_cropped_[0]*sin(this->left_line_cropped_[1]))/(cos(this->left_line_cropped_[1])));
    float bottom_crossing_x_left = this->left_line_cropped_[0]*cos(this->left_line_cropped_[1]) - sin(this->left_line_cropped_[1])*((this->cropped_.rows - this->left_line_cropped_[0]*sin(this->left_line_cropped_[1]))/(cos(this->left_line_cropped_[1])));

    float top_crossing_x_right = this->right_line_cropped_[0]*cos(this->right_line_cropped_[1]) - sin(this->right_line_cropped_[1])*((0 - this->right_line_cropped_[0]*sin(this->right_line_cropped_[1]))/(cos(this->right_line_cropped_[1])));
    float bottom_crossing_x_right = this->right_line_cropped_[0]*cos(this->right_line_cropped_[1]) - sin(this->right_line_cropped_[1])*((this->cropped_.rows - this->right_line_cropped_[0]*sin(this->right_line_cropped_[1]))/(cos(this->right_line_cropped_[1])));

    float del_x = std::abs(top_crossing_x_left - bottom_crossing_x_left);
    float del_y = std::abs(this->cropped_.rows);
    float m = del_y/del_x;
    this->alpha_deg_ = std::atan(m)*180.0/PI;

    del_x = std::abs(top_crossing_x_right - bottom_crossing_x_right);
    del_y = std::abs(this->cropped_.rows);
    m = del_y/del_x;
    this->beta_deg_ = std::atan(m)*180.0/PI;
}

// Helper methods:
// DONE: Method to prompt the user to set four input control points, which are on the two lines and saves them.
void LineDetector::setDefaultLines(Mat &new_image)
{
	// The user shall select four points, which will then be used for setting the default line.
 	Point2f p;
	namedWindow("Set Control Points", CV_WINDOW_AUTOSIZE);
  	Point2f init_points[4];
  	std::cout<<"Select two lines to Initialize."<<std::endl;
  	for(int i = 0; i < 4; i++)
  	{
    	std::cout<<"Point: "<<i + 1<<" out of "<<4<<std::endl;
    	std::cout<<"You have now 10 sec to click on your point"<<std::endl;
    	imshow("Set Control Points", new_image);
    	setMouseCallback("Set Control Points", getClickedPixel, &p);
    	waitKey(10000);
    	std::cout<<"Saved pixels: "<<std::endl;
    	std::cout<<p<<std::endl;
    	init_points[i] = p;
    	std::cout<<"Input Points saved!"<<std::endl;
  	}
  	setMouseCallback("Set Control Points", NULL);
  	destroyWindow("Set Control Points");
  	
  	// Transform to rho, theta and save to default lines and to current line.
  	Eigen::Vector2f polar_parameters;
    // Left line.
    polar_parameters = this->LineDetector::cartesian2PolarLines(init_points[0].x, init_points[1].x, init_points[0].y, init_points[1].y);
    this->left_line_cropped_[1] = polar_parameters[0];
    this->left_line_cropped_[0] = polar_parameters[1];
    // Right line;
    polar_parameters = this->LineDetector::cartesian2PolarLines(init_points[2].x, init_points[3].x, init_points[2].y, init_points[3].y);
    this->right_line_cropped_[1] = polar_parameters[0];
    this->right_line_cropped_[0] = polar_parameters[1];

    // Set the initial values as default value.
	this->default_left_ = this->left_line_cropped_;
	this->default_right_ = this->right_line_cropped_;

  	// Calculate alpha and beta (gradient of the lines) and save them.
  	float rho_left = this->default_left_[0];
  	float rho_right = this->default_right_[0];
  	float theta_left_rad = this->default_left_[1];
  	float theta_right_rad = this->default_right_[1];
  	float top_crossing_x_left = rho_left*cos(theta_left_rad) - sin(theta_left_rad)*((0 - rho_left*sin(theta_left_rad))/(cos(theta_left_rad)));
    float bottom_crossing_x_left = rho_left*cos(theta_left_rad) - sin(theta_left_rad)*((this->cropped_.rows - rho_left*sin(theta_left_rad))/(cos(theta_left_rad)));
    float top_crossing_x_right = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((0 - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));
    float bottom_crossing_x_right = rho_right*cos(theta_right_rad) - sin(theta_right_rad)*((this->cropped_.rows - rho_right*sin(theta_right_rad))/(cos(theta_right_rad)));
    float del_x = std::abs(top_crossing_x_left - bottom_crossing_x_left);
    float del_y = std::abs(this->cropped_.rows);
    float m = del_y/del_x;
    this->alpha_deg_ = std::atan(m)*180.0/PI;
    del_x = std::abs(top_crossing_x_right - bottom_crossing_x_right);
    del_y = std::abs(this->cropped_.rows);
    m = del_y/del_x;
    this->beta_deg_ = std::atan(m)*180.0/PI;
    this->alpha_default_ = this->alpha_deg_;
    this->beta_default_ = this->beta_deg_;
}

// DONE: ???
Mat LineDetector::roadThreshold(Mat src_rt)
{
	// Calculate the intensity of areas by using the function IntensityOfArea.
	Vec3b intensity_1 = intensityOfArea(src_rt, 360, 350, 100, 75);
	Vec3b intensity_2 = intensityOfArea(src_rt, 180, 350, 100, 75);
	float b_average = (intensity_1[0] + intensity_2[0])/2;
	float g_average = (intensity_1[1] + intensity_2[1])/2;
	float r_average = (intensity_1[2] + intensity_2[2])/2;
	// Define black vector.
	Vec3b black;
	black[0] = 0;
	black[1] = 0;
	black[2] = 0;
	// Define white vector.
	Vec3b white;
	black[0] = 255;
	black[1] = 255;
	black[2] = 255;
	// Iterate through the whole image.
	for(int y = 0; y < src_rt.rows; y++)
	{
 		for(int x = 0; x < src_rt.cols; x++)
  		{
			// Calculate the deviation of the intensity. 
    		Vec3b color = src_rt.at<Vec3b>(Point(x,y));
			float b_delta_sq = ((color[0] - b_average)/b_average)*((color[0] - b_average)/b_average);
			float g_delta_sq = ((color[1] - g_average)/g_average)*((color[1] - g_average)/g_average);
			float r_delta_sq = ((color[2] - r_average)/r_average)*((color[2] - r_average)/r_average);
			// Threshold the image.
			if ( (b_delta_sq < 0.05) && (g_delta_sq < 0.05) && (r_delta_sq < 0.05))
			{
				src_rt.at<Vec3b>(Point(x,y)) = black;
			}
			else
			{
				src_rt.at<Vec3b>(Point(x,y)) = white;
			}
		}
	}
	return src_rt;
}

// DONE: ???
Vec3b LineDetector::intensityOfArea(Mat &src_ioa, int x_gray, int y_gray, int width_gray, int height_gray)
{
	Mat src = this->original_.clone(); // Added by Nikku.
	// This function calculate the averaged intensity of alle color-channels of a fixed region of a image.
	Rect region_gray = Rect(x_gray, y_gray, width_gray, height_gray);
	rectangle(src_ioa, region_gray, Scalar(0, 255, 0));
  Mat src_gray = src(region_gray);
	float b_average = 0;
	float g_average = 0;
	float r_average = 0;
	for(int y = 0; y < src_gray.rows; y++)
	{
 		for(int x = 0; x < src_gray.cols; x++)
  		{
    		Vec3b color = src_gray.at<Vec3b>(Point(x,y));
		b_average = b_average + color[0];
		g_average = g_average + color[1];
		r_average = r_average + color[2];
		}
	}
	b_average = b_average/(src_gray.rows*src_gray.cols);
	g_average = g_average/(src_gray.rows*src_gray.cols);
	r_average = r_average/(src_gray.rows*src_gray.cols);
	Vec3b intensity;
	intensity[0] = b_average;
	intensity[1] = g_average;
	intensity[2] = r_average;
	return intensity;
}

// DONE: ???
Mat LineDetector::findGray(Mat src_fg)
{
	Mat blur = src_fg.clone();
	// medianBlur(src, blur, 7);
	Mat result(src_fg.rows, src_fg.cols, CV_8UC1);
	int sum;
	for(int y = 0; y < src_fg.rows; y++)
	{
 		for(int x = 0; x < src_fg.cols; x++)
  	{
    		Vec3b color = src_fg.at<Vec3b>(Point(x,y));
		      if( (color[0] > 220) || (color[1] > 220) || (color[2] > 220))
		        {
			        sum = 255;
		        }
		else
		{
			int bg = color[0] - color[1];
			int br = color[0] - color[2];
			int gr = color[1] - color[2];
			sum = (abs(bg) + abs(br) + abs(gr));
		}
		result.at<uchar>(Point(x,y)) = sum;
		}
	}
	medianBlur(result, blur, 7);
	return blur;
}

 // Method which returns rho and theta of a line defined in the cartesian space.
 Eigen::Vector2f LineDetector::cartesian2PolarLines(float x_a, float x_b, float y_a, float y_b)
 {
 	// Calculate theta.
  	float theta = atan2(x_a - x_b, y_b - y_a);

  	// Method to calculate rho.
  	float rho_nom = x_a + y_a*((x_a - x_b)/(y_b - y_a));
  	float rho_denom = cos(theta) - sin(theta)*(x_b - x_a)/(y_b - y_a);
  	float rho = rho_nom/rho_denom;

  	Eigen::Vector2f theta_and_rho;
  	theta_and_rho << theta,rho;

  	return theta_and_rho;
 }

// DONE: Method which transforms a point coordinate from original to cropped image.
Point2f LineDetector::coordinateOrig2Crop(Point2f coord_orig)
{
	Point2f coord_crop(coord_orig.x - this->cropping_corners_[0].x, coord_orig.y - this->cropping_corners_[0].y);
	return coord_crop;
}

// DONE: Method which transforms a point coordinate from cropped to original image.
Point2f LineDetector::coordinateCrop2Orig(Point2f coord_crop)
{
	Point2f coord_orig(coord_crop.x + this->cropping_corners_[0].x, coord_crop.y + this->cropping_corners_[0].y);
	return coord_orig;
}

// DONE: Draws lines to an image (both have to have the same coordinate system!).
void LineDetector::drawLines2Image(Mat &draw_to, vector<Vec2f> &lines_to_draw)
{
	for(int i = 0; i < lines_to_draw.size(); i++)
	{
		float rho = lines_to_draw[i][0];
		float theta = lines_to_draw[i][1];
		Point pt1;
		Point pt2;
		double a = cos(theta);
		double b = sin(theta);
		double x0 = a*rho;
		double y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));
		line(draw_to, pt1, pt2, Scalar(255, 0, 0), 3, CV_AA);
	}
}

 // DONE: Displays the image in a window.
void LineDetector::showImage(Mat show, string name)
{
	namedWindow(name, CV_WINDOW_AUTOSIZE);
	imshow(name, show);
  	waitKey(1);
}