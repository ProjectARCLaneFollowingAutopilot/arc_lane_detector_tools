#include "../../include/line_detection/line_detection.hpp"

 // PUBLIC MEMBER METHODS.
 // Standard constructor.
 LINEDETECTOR::LINEDETECTOR()
 { 
 	std::cout<<"Linedetector created!"
 }
 // Standard destructor.
 LINEDETECTOR::~LINEDETECTOR()
 {
 	std::cout<<"Linedetector destroyed!"<<std::endl;
 }
 // Does the Hough-Transform and draws the lines.
 void LINEDETECTOR::houghTransform(Mat &contours, Mat &draw_to, vector<Vec2f> &lines_hT, int threshold)
 {
 	//Hough transform.
 	HoughLines(contours, lines_hT, 1, CV_PI/180, threshold, 0, 0);
	//Iterate through all the lines and draw the line. 
	for(int i = 0; i < lines_hT.size(); i++)
	{
		float rho = lines_hT[i][0];
		float theta = lines_hT[i][1];
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

 // PUBLIC MEMBER VARIABLES.

 // PRIVATE MEMBER METHODS.

 // Line finding methods
 vector<Vec2f> LINEDETECTOR::HoughClassic (Mat src_HC)
 {
 	// Filter the image.
  	Mat src_HC_roi_filtered = src_HC.clone();
  	medianBlur(src_HC, src_HC_roi_filtered, 15);

  	Mat contours = src_HC_roi_filtered.clone();
  	//Run Canny. Parameter to be determined.
  	Canny(src_HC_roi_filtered, contours, 30, 50);
	Mat draw_detected_hough = src_HC.clone();
  	vector<Vec2f> lines_HC;
	//Do HoughTransform.
  	houghTransform(contours, draw_detected_hough, lines_HC, 90);
  	return lines_HC;
 }
 vector<Vec2f> LINEDETECTOR::InRange (Mat src_IR)
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
 vector<Vec2f> LINEDETECTOR::GrayProperty (Mat src_GP)
 {
 	vector<Vec2f> lines_GP;
	Mat contours(src_GP.rows, src_GP.cols, CV_8UC1);
	//Find gray areas using the function FindGray.
	Mat gray = FindGray(src_GP);
	Canny(gray, contours, 50, 150);
	houghTransform(contours, src_GP, lines_GP, 40);
	return lines_GP;
 }
 vector<Vec2f> LINEDETECTOR::CompareGray (Mat src_CG);

 // Helper methods:
 Mat LINEDETECTOR::showChannel(Mat RGB, bool B, bool G, bool R);
 Mat LINEDETECTOR::RoadThreshold(Mat src_RT);
 Vec3b LINEDETECTOR::IntensityOfArea(Mat &src_IOA, int x_gray, int y_gray, int width_gray, int height_gray);

 // PRIVATE MEMBER VARIABLES.