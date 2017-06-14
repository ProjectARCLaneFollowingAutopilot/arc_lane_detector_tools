#include "../../include/line_detection/line_detection.hpp"

// PUBLIC MEMBER METHODS.
// DONE: Standard constructor.
LineDetector::LineDetector()
{
	std::cout<<"Linedetector created!"<<std::endl;
}
// DONE: Standard destructor.
LineDetector::~LineDetector()
{
	std::cout<<"Linedetector destroyed!"<<std::endl;
}
// DONE: Set a new original and cropped image.
void LineDetector::setImage(Mat &new_image)
{
	// Set original image.
	this->original_ = new_image.clone();
	// Set cropped image.
	this->cropped_ = new_image(Rect(this->cropping_corners_[0].x, this->cropping_corners_[0].y, this->cropping_corners_[1].x - this->cropping_corners_[0].x, this->cropping_corners_[1].y - this->cropping_corners_[0].y));
}
// DONE: Define all parameters: ROI-Cropping corners, two default lines (original coordinates),...
void LineDetector::setParams(Point2f roi_left_top, Point2f roi_right_bottom, Vec2f default_left, Vec2f default_right)
{
	this->cropping_corners_[0] = roi_left_top;
	this->cropping_corners_[1] = roi_right_bottom;
	this->default_lines_[0] = default_left;
	this->default_lines_[1] = default_right;
	// Set the two lines to the default lines.
}

// Method which forces to detect lines in the cropped image, does filtering and saves important data-->master function.
void LineDetector::doLineDetection()
{
	// Find lines in the cropped image and save them.
	this->all_lines_cropped_ =  this->LineDetector::findAllLines(this->cropped_);
	// Transform all found lines to original coordinates and save them.
	// Filter the cropped lines out to only find two lines and save.
	vector<Vec2f> two_lines_crop =  this->LineDetector::getTwoLinesCrop();
	this->left_line_cropped_ = two_lines_crop[0];
	this->right_line_cropped_ = two_lines_crop[1];
	// Transform filtered lines and save them.
}

// Method which returns the left pointcloud around the two lines in the original coordinate system.
vector<Point2f> LineDetector::pointsAroundLeftLineOrig()
{

}
// Method which returns the right pointcloud around the two lines in the original coordinate system.
vector<Point2f> LineDetector::pointsAroundRightLineOrig()
{

}
// DONE: Method which deletes the lines vectors.
void LineDetector::clearLines()
{
  this->all_lines_orig_.clear();
  this->all_lines_cropped_.clear();
}

// Helper methods:
// DONE: Does the Hough-Transform, gives lines and draws the lines.
void LineDetector::houghTransform(Mat &contours, Mat &draw_to, vector<Vec2f> &lines_hT, int threshold)
{
	//Hough transform. Parameter to be determined.
 	HoughLines(contours, lines_hT, 1, CV_PI/180, threshold, 0, 0);  // war 90.
	//Iterate through all the lines and draw the line. 
	for(int i = 0; i < lines_hT.size(); i++)
	{
		float rho = lines_hT[i][0];
		float theta = lines_hT[i][1];
		// cout << i << "   " <<theta <<endl;
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
// Method which returns ALL lines in the original coordinate system.
vector<Vec2f> LineDetector::getAllLinesOrig()
{

}
// Method which returns ALL lines in the cropped coordinate system.
vector<Vec2f> LineDetector::getAllLinesCrop()
{

}
// Method which returns only two lines in the original coordinate system.
vector<Vec2f> LineDetector::getTwoLinesOrig()
{

}
// Method which returns only two lines in the cropped coordinate system.
vector<Vec2f> LineDetector::getTwoLinesCrop()
{

}
  
// PRIVATE MEMBER METHODS.
// DONE: Method which finds all lines in an image, using a combination of different line finding methods.
vector<Vec2f> LineDetector::findAllLines(Mat &lines_to_find)
{
	// Do several Hough transforms.
  	vector<Vec2f> h_c = HoughClassic (lines_to_find);
  	vector<Vec2f> g_p = GrayProperty(lines_to_find);
  	vector<Vec2f> i_r = InRange(lines_to_find);
  	vector<Vec2f> c_g = CompareGray (lines_to_find);
  	
  	// Append all vectors.
  	vector<Vec2f> all_detected_lines;
  	all_detected_lines.insert(all_detected_lines.end(), h_c.begin(), h_c.end());
  	all_detected_lines.insert(all_detected_lines.end(), g_p.begin(), g_p.end());
  	all_detected_lines.insert(all_detected_lines.end(), i_r.begin(), i_r.end());
  	all_detected_lines.insert(all_detected_lines.end(), c_g.begin(), c_g.end());

	//  Return all lines.
	return all_detected_lines;
}

// Line finding methods:
// DONE: ???
vector<Vec2f> LineDetector::HoughClassic(Mat &src_HC)
{
	// Filter the image.
 	Mat src_HC_roi_filtered = src_HC.clone();
 	medianBlur(src_HC, src_HC_roi_filtered, 15);

 	Mat contours = src_HC_roi_filtered.clone();
 	// Run Canny.
 	Canny(src_HC_roi_filtered, contours, 30, 50);
	Mat draw_detected_hough = src_HC.clone();
  	vector<Vec2f> lines_HC;
	//Do HoughTransform.
  	houghTransform(contours, draw_detected_hough, lines_HC, 90);
  	return lines_HC;
}
// DONE: ???
vector<Vec2f> LineDetector::InRange(Mat src_IR)
{
	//Calculate a mask by using the openCV-function inRange.
  	Mat mask(src_IR.rows, src_IR.cols, CV_8UC1);
	inRange(src_IR, Scalar(40, 40, 40),Scalar(150, 150, 150), mask);
	Mat gray(src_IR.rows, src_IR.cols, CV_8UC1);
	Mat contours(src_IR.rows, src_IR.cols, CV_8UC1);
	Mat conv(src_IR.rows, src_IR.cols, CV_8UC1, 255);
	//Apply the mask to the source image. 
	bitwise_and(src_IR, src_IR, conv, mask);
	Canny(mask, contours, 120, 150);
	vector<Vec2f> lines_IR;
	houghTransform(contours, src_IR ,lines_IR, 80);
	return lines_IR;
}
// DONE: ???
vector<Vec2f> LineDetector::GrayProperty(Mat src_GP)
{
	vector<Vec2f> lines_GP;
	Mat contours(src_GP.rows, src_GP.cols, CV_8UC1);
	//Find gray areas using the function FindGray.
  	Mat gray = FindGray(src_GP);
	Canny(gray, contours, 50, 150);
	houghTransform(contours, src_GP, lines_GP, 40);
	return lines_GP;
}
// DONE: ???
vector<Vec2f> LineDetector::CompareGray(Mat src_CG)
{
	vector<Vec2f> lines_CG;
	// showImage(showChannel(src_CG, true, true, true), "RGB");
	// showImage(RoadThreshold(src_CG), "RoadTreshold");
	Mat color_contour;
	//Calculate Canny-image by using the function RoadThreshold;
	Canny(RoadThreshold(src_CG),  color_contour, 30, 50);
	// showImage(color_contour, "FarbCanny");
	Mat show_color_Hough=src_CG;
	houghTransform(color_contour, show_color_Hough, lines_CG, 70);
	// showImage(show_color_Hough, "ColorHoug");
	return lines_CG;
}

// Helper methods:
// DONE: ???
Mat LineDetector::showChannel(Mat RGB, bool B, bool G, bool R)
{
	// This function is able to spit the color-channels and merge them again, if they are needed.
	Mat channel[3];
	Mat result=Mat::zeros(RGB.rows, RGB.cols, CV_8UC3);
	split(RGB, channel);
	if (B==false)
		{
		channel[0]=Mat::zeros(RGB.rows, RGB.cols, CV_8UC1);
		}
	if (G==false)
		{
		channel[1]=Mat::zeros(RGB.rows, RGB.cols, CV_8UC1);
		}
	if (R==false)
		{
		channel[2]=Mat::zeros(RGB.rows, RGB.cols, CV_8UC1);
		}
	merge(channel, 3, result);
	return result;
}
// DONE: ???
Mat LineDetector::RoadThreshold(Mat src_RT)
{
	//Calculate the intensity of areas by using the function IntensityOfArea.
	Vec3b Intensity1 = IntensityOfArea(src_RT, 360, 350, 100, 75);
	Vec3b Intensity2 = IntensityOfArea(src_RT, 180, 350, 100, 75);
	float B_average=(Intensity1[0]+Intensity2[0])/2;
	float G_average=(Intensity1[1]+Intensity2[1])/2;
	float R_average=(Intensity1[2]+Intensity2[2])/2;
	//Define black vector.
	Vec3b black;
	black[0]=0;
	black[1]=0;
	black[2]=0;
	//Define white vector.
	Vec3b white;
	black[0]=255;
	black[1]=255;
	black[2]=255;
	//Iterate through the whole image.
	for(int y=0;y<src_RT.rows;y++)
	{
 		for(int x=0;x<src_RT.cols;x++)
  		{
			//Calculate the deviation of the intensity. 
    			Vec3b color = src_RT.at<Vec3b>(Point(x,y));
			float B_delta_sq = ((color[0]-B_average)/B_average)*((color[0]-B_average)/B_average);
			float G_delta_sq = ((color[1]-G_average)/G_average)*((color[1]-G_average)/G_average);
			float R_delta_sq = ((color[2]-R_average)/R_average)*((color[2]-R_average)/R_average);
			//Threshold the image.
			if ( (B_delta_sq<0.05) && (G_delta_sq<0.05) && (R_delta_sq<0.05))
			{
				src_RT.at<Vec3b>(Point(x,y))=black;
			}
			else
			{
				src_RT.at<Vec3b>(Point(x,y))=white;
			}
		}
	}
	return src_RT;
}
// DONE: ???
Vec3b LineDetector::IntensityOfArea(Mat &src_IOA, int x_gray, int y_gray, int width_gray, int height_gray)
{
	Mat src = this->original_.clone(); // Added by Nikku.
	// This function calculate the averaged intensity of alle color-channels of a fixed region of a image.
	Rect region_gray = Rect(x_gray, y_gray, width_gray, height_gray);
	rectangle(src_IOA, region_gray, Scalar(0, 255, 0));
  	Mat src_gray = src(region_gray);
	float B_average=0;
	float G_average=0;
	float R_average=0;
	for(int y=0;y<src_gray.rows;y++)
	{
 		for(int x=0;x<src_gray.cols;x++)
  		{
    		Vec3b color = src_gray.at<Vec3b>(Point(x,y));
		B_average=B_average+color[0];
		G_average=G_average+color[1];
		R_average=R_average+color[2];
		}
	}
	B_average=B_average/(src_gray.rows*src_gray.cols);
	G_average=G_average/(src_gray.rows*src_gray.cols);
	R_average=R_average/(src_gray.rows*src_gray.cols);
	Vec3b intensity;
	intensity[0]=B_average;
	intensity[1]=G_average;
	intensity[2]=R_average;
	return intensity;
}
// DONE: ???
Mat LineDetector::FindGray(Mat src_FG)
{
	Mat blur = src_FG.clone();
	// medianBlur(src, blur, 7);
	Mat result(src_FG.rows, src_FG.cols, CV_8UC1);
	int sum;
	//
	for(int y = 0;y < src_FG.rows; y++)
	{
 		for(int x = 0; x<src_FG.cols; x++)
  	{
    		Vec3b color = src_FG.at<Vec3b>(Point(x,y));
		      if( (color[0]>220) || (color[1]>220) || (color[2]>220))
		        {
			        sum = 255;
		        }
		else
		{
			int bg=color[0]-color[1];
			int br=color[0]-color[2];
			int gr=color[1]-color[2];
			sum=(abs(bg)+abs(br)+abs(gr));
		}
		result.at<uchar>(Point(x,y))=sum;
		}
	}
	medianBlur(result, blur, 7);
	return blur;
}
// Method which transforms rho and theta from cropped image (ROI) to original image.
//Vec2f LineDetector::polarParamCrop2Orig(Vec2f polar_param_crop){}
// Method which transforms rho and theta from original image to cropped image (ROI).
//Vec2f LineDetector::polarParamOrig2Crop(Vec2f polar_param_orig){}

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

// Draws lines to an image (both have to have the same coordinate system!) by getting a vector with (rho, theta).
void LineDetector::drawLines2Image(Mat &draw_to, vector<Vec2f> lines_to_draw)
{

}
 // DONE: Displays the image in a window.
void LineDetector::showImage(Mat show, string name)
{
	namedWindow(name, CV_WINDOW_AUTOSIZE);
	imshow(name, show);
  	waitKey(1);
}