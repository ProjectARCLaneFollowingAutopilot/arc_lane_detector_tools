#include "../../include/line_detection/line_detection.hpp"

 // PUBLIC MEMBER METHODS.
 // Standard constructor.
 LineDetector::LineDetector()
 { 
 	std::cout<<"Linedetector created!"<<std::endl;
 }
 // Standard destructor.
 LineDetector::~LineDetector()
 {
 	std::cout<<"Linedetector destroyed!"<<std::endl;
 }
 // Does the Hough-Transform and draws the lines.
 void LineDetector::houghTransform(Mat &contours, Mat &draw_to, vector<Vec2f> &lines_hT, int threshold)
 {

 }

 // PUBLIC MEMBER VARIABLES.

 // PRIVATE MEMBER METHODS.

 // Line finding methods.
 vector<Vec2f> LineDetector::HoughClassic (Mat &src_HC)
 {

 }
 vector<Vec2f> LineDetector::InRange (Mat src_IR)
 {

 }
 vector<Vec2f> LineDetector::GrayProperty (Mat src_GP)
 {

 }
 vector<Vec2f> LineDetector::CompareGray (Mat src_CG)
 {

 }

 // Helper methods:
 Mat LineDetector::showChannel(Mat RGB, bool B, bool G, bool R)
 {

 }
 Mat LineDetector::RoadThreshold(Mat src_RT)
 {

 }
 Vec3b LineDetector::IntensityOfArea(Mat &src_IOA, int x_gray, int y_gray, int width_gray, int height_gray)
 {

 }
 Mat LineDetector::FindGray(Mat src)
 {

 }