///***************************************************
//Casimir Sowinski
//Midterm, question 1
//11/10/2016
//***************************************************/
//#include <opencv2/opencv.hpp>
//
//using namespace std;
//using namespace cv;
//
//int main(int argc, char** argv) {
//		/// Read image, check for errors, convert to gray
//		Mat src, src_gray;
//		src = imread("circle.png", 1);
//		if (src.empty())
//		{
//				cout << "Could not load image.\n";
//				return -1;
//		}
//		cvtColor(src, src_gray, CV_BGR2GRAY);
//
//		/// Holds data about found circles' center location and radius
//		vector<Vec3f> circles;
//
//		/// Use Circle version of Hough Transform
//		HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, 1, 50, 15, 0, 0);
//
//		/// Draw the circle(s)
//		for (size_t i = 0; i < circles.size(); i++)
//		{
//				Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
//				int radius = cvRound(circles[i][2]);
//				circle(src, center, radius, Scalar(0, 0, 255), 1, 4, 0);
//		}
//
//		/// Print some data
//		if (circles.size()) {
//				float center_x = circles[0][0]; // a
//				float center_y = circles[0][1]; // b
//				float radius = circles[0][2];
//				float rho = sqrt(pow(center_x, 2) + pow(center_y, 2));
//				float theta_rad = atan(center_y / center_x);
//				float theta_deg = theta_rad * 360 / 6.282;
//
//				cout << "center_x: " << center_x << endl;
//				cout << "center_y: " << center_y << endl;
//				cout << "radius: " << radius << endl;
//				cout << "rho: " << rho << endl;
//				cout << "theta: " << theta_deg << "deg" << endl; 
//		}
//		
//		/// Show detected circle(s)
//		namedWindow("Midterm, Question 1", CV_WINDOW_NORMAL);
//		imshow("Midterm, Question 1", src);
//		imwrite("detected circles.png", src);
//
//		/// Wait, exit
//		waitKey(0);
//		destroyAllWindows();
//		return 0;
//}
//
