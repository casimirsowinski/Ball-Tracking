///*
//Casimir Sowinski
//ECE-510 Embedded Vision 1
//Project
//Tennis ball tracker using color and depth cameras
//*/
//#include <windows.h>
//#include <wchar.h>
//#include "pxcsensemanager.h"
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//using namespace std;
//using namespace cv;
//
///// Globals
//PXCSenseManager *g_smR = PXCSenseManager::CreateInstance();
//////PXCSenseManager *g_smL = PXCSenseManager::CreateInstance();
//// flags
//extern bool g_firstFunc = true;
//bool g_firstLoop = true;
//bool g_beginCanny = false;
//bool g_enableColor = true;
//bool g_enableDepth = false;
//bool g_enableColorSegmentation = true;
//bool g_enableCBs = true;
//// CB 
//const char track1[] = "H Center";
//const char track2[] = "H Range";
//const char track3[] = "S Min";
//const char track4[] = "S Max";
//const char track5[] = "V Min";
//const char track6[] = "V Max";
//const char track7[] = "Openings";
//const char track8[] = "Blurs";
//const char track9[] = "Canny";
//int tb1_value = 60;
//int tb2_value = 10;
//int tb3_value = 0;
//int tb4_value = 255;
//int tb5_value = 0;
//int tb6_value = 255;
//int tb7_numMorph = 8;
//int tb8_numBlur = 2;
//int tb9_lowThresh = 1;
//const int MAXLOWTHRESH = 200;
//Mat g_hueMask;
//Mat g_morph;
//Mat g_blur;
//Mat g_canny;
//const char hue_win[] = "hue window";
//const char morph_win[] = "morph window";
//const char blur_win[] = "blur window";
//const char canny_win[] = "canny window";
//const int MINSIZE = 2;
//const int TRAILSIZE = 31;
//
///// Function declarations
//void initTBs();																									// initialize trackbars
//int getRSParameters();																					// get general properties/modules of RS
//void initRS(PXCSenseManager*, cv::Size size, int rate);					// set up session(s)
//void printMatProperties(cv::Mat*, char*);												// print Mat information, size, etc.
//																																// RS to OCV conversion functions
//cv::Mat PXCImage2CVMat(PXCImage*, PXCImage::PixelFormat);				// original-ish version
//cv::Mat PXCImage2CVMatAlt(PXCImage*, PXCImage::PixelFormat);		// mix of PXCI... and ConvertP...
//void ConvertPXCImageToOpenCVMat(PXCImage*, Mat*);								// original-ish version
//void RSColorPXCImage2OCVMat(PXCImage*, Mat*);										// Only for Color camera
//void ConvertDepthPXCImageToOpenCVMat(PXCImage*, Mat*);					// specifically for depth, not complete
//																																// Mat color conversion funtions
//cv::Mat depth2DepthColor(Mat*, int, int);												// to make rainbow version, not working
//cv::Mat depth2DepthColor(Mat*);																	// to make rainbow version, not working
//																																// CB functions
//void tb1CB_minH(int value, void *);
//void tb2CB_maxH(int value, void *);
//void tb3CB_minS(int value, void *);
//void tb4CB_maxS(int value, void *);
//void tb5CB_minV(int value, void *);
//void tb6CB_maxV(int value, void *);
//void tb7CB_numMorph(int value, void *);
//void tb8CB_numBlur(int value, void *);
//void tb9CB_canny(int value, void *);
//
//int wmain(int argc, WCHAR* argv[]) {
//
//		initTBs();
//
//		/// Desired OpenCV parameters
//		cv::Size frameSize = cv::Size(640, 480);
//		int frameRate = 60;
//		cv::Mat frameColorR = cv::Mat::zeros(frameSize, CV_8UC3);
//		cv::Mat frameDepthR = cv::Mat::zeros(frameSize, CV_8UC1);
//		cv::Mat frameDepthRAlt = cv::Mat::zeros(frameSize, CV_8UC1);
//		////cv::Mat frameColorL = cv::Mat::zeros(frameSize, CV_8UC3);
//		////cv::Mat frameDepthL = cv::Mat::zeros(frameSize, CV_8UC1);
//		////cv::Mat frameDepthLAlt = cv::Mat::zeros(frameSize, CV_8UC1);
//		////cv::Mat frameDepthRColor = cv::Mat::zeros(frameSize, CV_16UC3);
//
//		/// RS stuff
//		//getRSParameters();
//		initRS(g_smR, frameSize, frameRate);
//		////initRS(g_smL, frameSize, frameRate);
//
//		/// Run CBs
//		tb1CB_minH(tb1_value, 0);
//		tb2CB_maxH(tb2_value, 0);
//		tb3CB_minS(tb3_value, 0);
//		tb4CB_maxS(tb4_value, 0);
//		tb5CB_minV(tb5_value, 0);
//		tb6CB_maxV(tb6_value, 0);
//		tb7CB_numMorph(tb7_numMorph, 0);
//		tb8CB_numBlur(tb8_numBlur, 0);
//		tb8CB_numBlur(tb8_numBlur, 0);
//
//		/// For contours
//		int area_max = 0;
//		int max_index = 0;
//		Rect bounding_rect;
//
//		Mat src = imread("shapes.png"); //Load source image
//		Mat thr(src.rows, src.cols, CV_8UC1);
//		Mat dst(src.rows, src.cols, CV_8UC1, Scalar::all(0));
//		cvtColor(src, thr, CV_BGR2GRAY); //Convert to gray
//		threshold(thr, thr, 25, 255, THRESH_BINARY); //Threshold the gray
//
//		vector<vector<Point>> contours; // Vector for storing contour
//		vector<Vec4i> hierarchy;
//		Scalar color(0, 0, 255);
//		Point trail[TRAILSIZE] = { Point(0, 0) };
//
//		/// Main loop, stream data, process data
//		// Loop vars
//		bool connectedToRS = false;							// Track whether main loop entered/connected
//																						//cv::Mat aveArray[3] = { cv::Mat::zeros(frameSize, CV_32FC3) }; // array to keep track of last 3 frames
//		cv::Mat acc = cv::Mat::zeros(frameSize, CV_32FC3); // accumulator for averaging frames
//		cv::Mat acc8U = cv::Mat::zeros(frameSize, CV_8UC3); // 
//		cv::Mat runningAve8U = cv::Mat::zeros(frameSize, CV_8UC3); // rounded average of last three frames
//
//		while (g_smR->AcquireFrame(true) >= PXC_STATUS_NO_ERROR) {
//				connectedToRS = true;								// Flip flag
//
//																						/// Aquire data
//				PXCCapture::Sample *sampleR;
//				////PXCCapture::Sample *sampleL;
//				sampleR = g_smR->QuerySample();
//				////sampleL = g_smL->QuerySample();
//				if (sampleR->color && g_enableColor) {
//						//frameColorR = PXCImage2CVMat(sampleR->color, PXCImage::PIXEL_FORMAT_RGB24);
//						//ConvertPXCImageToOpenCVMat(sampleR->color, &frameColorR); //<- last used
//						RSColorPXCImage2OCVMat(sampleR->color, &frameColorR); //only for colorimage
//																																	////ConvertPXCImageToOpenCVMat(sampleL->color, &frameColorL);
//				}
//				if (sampleR->depth && g_enableDepth) {
//						////PXCImage2CVMatAlt(sampleR->depth, PXCImage::PIXEL_FORMAT_DEPTH_F32).convertTo(frameDepthRAlt, CV_8UC1);
//						ConvertDepthPXCImageToOpenCVMat(sampleR->depth, &frameDepthR);
//						//ConvertDepthPXCImageToOpenCVMat(sampleL->depth, &frameDepthL);
//				}
//
//				/// Manipulate data
//				/// Find tennis ball by its Hue
//				if (g_enableColorSegmentation && g_enableColor) {
//						/// Strip the alpha channel first
//						cvtColor(frameColorR, frameColorR, CV_BGRA2BGR);
//						//cvtColor(frameHSV, frameHSV, CV_RGB2HSV);
//						//cvtColor(frameColorR, frameC3, CV_BGRA2BGR);
//
//						//accumulateWeighted(frameHSV, acc, 0.5);
//						//imshow("acc", acc / 255);
//						//acc.convertTo(acc8U, CV_8UC3);
//
//						/// Blur
//						for (int i = 1; i < 2; i++) {
//								blur(frameColorR, frameColorR, Size(3, 3));
//						}
//
//						/// Convert to HSV
//						Mat frameHSV;
//						cvtColor(frameColorR, frameHSV, CV_RGB2HSV);
//
//						/// Two sided threshold to find a particular Hue range
//						/// Convert taskbar 1 and 2 values to a center and range, ensure ranges
//						int HBoundLo = tb1_value - tb2_value;
//						int HBoundHi = tb1_value + tb2_value;
//						if (HBoundLo < 0) HBoundLo = 0;
//						if (HBoundHi > 255) HBoundHi = 255;
//						Scalar lower(HBoundLo, tb3_value, tb5_value);
//						Scalar upper(HBoundHi, tb4_value, tb6_value);
//						inRange(frameHSV, lower, upper, g_hueMask);
//
//						/// Open image tb7_numMorph times to get rid of some noise
//						Mat kernel = Mat::ones(3, 3, CV_8U);
//						morphologyEx(g_hueMask, g_morph, MORPH_OPEN, kernel, Point(-1, -1), tb7_numMorph, BORDER_CONSTANT);
//
//						//// Run Bluring as many times as input by user 
//						//Mat blurGray;// , edges;
//						//g_morph.convertTo(blurGray, CV_8UC1);
//						//for (int i = 1; i < tb8_numBlur; i++) {
//						//		//GaussianBlur(blurGray, g_blur, Size(3, 3), BORDER_CONSTANT);
//						//		blur(blurGray, g_blur, Size(3, 3));
//						//}										
//
//						/// Find edges with Canny
//						Canny(g_morph, g_canny, tb9_lowThresh, tb9_lowThresh * 3, 5);
//
//						/// Find contours, find the biggest contour for ROI
//						//int bigContourIdx = 0;
//						//int bigContourArea = 0;
//						//Rect bigContourBound;
//						//vector<vector<Point>> contours;
//						//vector<Vec4i> hierarchy;
//						//Mat test = imread("shapes.png", CV_LOAD_IMAGE_COLOR);
//						//findContours(test, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//						//for (int i = 0; i < contours.size(); i++) {
//						//		double area = contourArea(contours[i], false);
//						//		if (area > bigContourArea) {
//						//				bigContourArea = area;
//						//				bigContourIdx = i;
//						//				bigContourBound = boundingRect(contours[i]);
//						//		}
//						//}
//						//Scalar color(0, 0, 255);
//						//drawContours(test, contours, bigContourIdx, color, CV_FILLED, 8, hierarchy);
//						//rectangle(frameColorR, bigContourBound, Scalar(255, 0, 0), 1, 8, 0);
//
//						findContours(g_canny, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image
//						int contSize = contours.size();
//						area_max = 0;
//						max_index = 0;
//						for (int i = 0; i < contSize; i++) // iterate through each contour. 
//						{
//								double area = contourArea(contours[i], false);  //  Find the area of contour
//								if (area > area_max) {
//										area_max = area;
//										max_index = i;                //Store the index of largest contour
//										bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
//								}
//
//						}
//						/// Check if any contours exist, draw the max contour if they do
//						if (contSize && bounding_rect.width > MINSIZE) {
//								rectangle(frameColorR, bounding_rect, Scalar(0, 255, 0), 1, 8, 0);
//								int radius = bounding_rect.width / 2;
//								Point center(bounding_rect.x + radius, bounding_rect.y + radius);
//								circle(frameColorR, center, radius, Scalar(255, 0, 0), 3, 8, 0);
//								// Draw the largest contour using previously stored index.
//								drawContours(frameColorR, contours, max_index, color, 5, 8, hierarchy);
//
//								trail[0] = center;
//								/// Draw trail
//								for (int i = TRAILSIZE; i > 0; i--) {
//										trail[i] = trail[i - 1];
//										//line(frameColorR, trail[i - 1], trail[i], Scalar(39, 127, 255), 8, 8, 0);
//								}
//
//
//						}
//						for (int i = TRAILSIZE; i > 0; i--) {
//								//trail[i] = trail[i - 1];
//								int trailWidth = cvRound(16 - i * 15 / TRAILSIZE);
//								Scalar trailColor(252, 255 - i * 226 / TRAILSIZE, 28 + i * 119 / TRAILSIZE);
//								line(frameColorR, trail[i - 1], trail[i], trailColor, trailWidth, 8, 0);
//						}
//
//
//						/// show results
//						cv::imshow(hue_win, g_hueMask);
//						cv::imshow(morph_win, g_morph);
//						//cv::imshow(blur_win, g_blur);	
//						cv::imshow(canny_win, g_canny);
//						//cv:imshow("gray blur", blur);
//						//cv::imshow("denoised hue", deNoised);
//				}
//
//				/// Display data
//				if (g_enableColor) {
//						cv::imshow("Color Right", frameColorR);
//						////cv::imshow("Color Left", frameColorL);
//						//cv::imshow("Color", )
//				}
//				if (g_enableDepth) {
//						cv::imshow("DepthAlt", frameDepthRAlt);
//						cv::imshow("Depth", frameDepthR);
//						// Convert to Rainbow coloring
//						//frameDepthRColor = depth2DepthColor(&frameDepthR);
//						//cv::imshow("Depth Color", frameDepthRColor);
//				}
//
//				/// Print info
//				if (g_firstLoop) {
//						std::cout << "\n-------------------------------------\n";
//						if (g_enableColor) {
//								printMatProperties(&frameColorR, "frameColorR");
//								printMatProperties(&src, "src");
//								printMatProperties(&dst, "dst");
//								printMatProperties(&thr, "thr");
//								//cout << "\nframeDepthRAlt:\n";
//								//cout << "Size: " << frameColorR.cols << "x" << frameColorR.rows << endl;
//								//cout << "Depth: " << frameColorR.depth() << endl;
//								//cout << "Type: " << frameColorR.type() << endl;
//								//cout << "Channels: " << frameColorR.channels() << endl;
//						}
//						if (g_enableDepth) {
//								printMatProperties(&frameDepthR, "frameDepthR");
//								//cout << "\nframeDepthR:\n";
//								//cout << "Size: " << frameDepthR.cols << "x" << frameDepthR.rows << endl;
//								//cout << "Depth: " << frameDepthR.depth() << endl;
//								//cout << "Type: " << frameDepthR.type() << endl;
//								//cout << "Channels: " << frameDepthR.channels() << endl;
//						}
//						printMatProperties(&g_hueMask, "hueMask");
//						printMatProperties(&g_morph, "morph");
//						printMatProperties(&g_blur, "blur");
//						//printMatProperties(&);
//						//printMatProperties(&);
//						std::cout << "\n-------------------------------------\n";
//						g_firstLoop = false;
//				}
//
//				// Check for exit request
//				int key = cv::waitKey(1);
//				if (key == 27)
//						break;
//
//				// Resume next frame processing
//				g_smR->ReleaseFrame();
//		}
//
//		/// Exit
//		if (connectedToRS) {
//				std::cout << "\n-------------------------------------\n";
//				std::cout << "Closing.";
//				std::cout << "\n-------------------------------------\n";
//		}
//		else {
//				std::cout << "\n-------------------------------------\n";
//				std::cout << "Could not connect to RealSense.\nClosing.";
//				std::cout << "\n-------------------------------------\n";
//		}
//		cv::waitKey(0);
//		g_smR->Release();
//		cv::destroyAllWindows();
//		return 0;
//}
//
//// with speckles, might want to get rid of the range/min/max code as it doesn't really do anything
//// because there is always max range due to white and black noise thats always there
//// maybe get rid of white noise with morphological operations
//cv::Mat depth2DepthColor(cv::Mat* depth) {
//		/// Create return Mat
//		cv::Mat depthColorBGR(Size(depth->cols, depth->rows), CV_8UC3);
//		cv::Mat depthColorHSV(Size(depth->cols, depth->rows), CV_8UC3);
//		cv::Mat GR = Mat::ones(Size(depth->cols, depth->rows), CV_8UC1) * 120;
//
//		Mat test;
//		depth->convertTo(test, CV_8UC3);
//
//		Mat channels[3];
//		//channels[0] = *depth;
//		channels[0] = test;
//		channels[1] = GR;
//		channels[2] = GR;
//
//		cout << "depth of test: " << test.depth() << endl;
//		cout << "depth of GR: " << GR.depth() << endl;
//
//		/// some problem here
//		merge(channels, 3, depthColorBGR);
//
//		if (g_firstFunc && false) {
//				cout << "depthColor:\n";
//				cout << "Size: " << depthColorBGR.cols << "x" << channels[0].rows << endl;
//				cout << "Depth: " << depthColorBGR.depth() << endl;
//				cout << "Type: " << depthColorBGR.type() << endl;
//				g_firstFunc = false;
//		}
//
//		//Mat eightB;
//		//depthColorBGR.convertTo(eightB, CV_32FC3);
//
//		cvtColor(depthColorBGR, depthColorHSV, CV_RGB2HSV);
//
//		///// Ptr to data in input mat
//		//unsigned char* dataLoc = (unsigned char*)(depth->data);
//		///// number of channels for ptr calculations
//		//int numChanngels = 3;
//		//for (int row = 0; row < depth->rows; row++) {
//		//		for (int col = 0; col < depth->cols; col++) {
//		//				int ptr = row * depth->cols * numChannels + col * numChannels;
//		//				dataLoc[ptr] = 
//		//		}
//		//}
//
//
//		return depthColorHSV;
//}
//
//cv::Mat depth2DepthColor(cv::Mat* depth, int min, int max) {
//		/// Create return Mat
//		cv::Mat depthColor(Size(depth->cols, depth->rows), CV_8SC3);
//
//		int range = max - min;
//
//		cout << "range: " << range << endl;
//
//
//		return depthColor;
//}
//
//cv::Mat PXCImage2CVMat(PXCImage *pxcImage, PXCImage::PixelFormat format)
//{
//		PXCImage::ImageData data;
//		pxcImage->AcquireAccess(PXCImage::ACCESS_READ, format, &data);
//
//		int width = pxcImage->QueryInfo().width;
//		int height = pxcImage->QueryInfo().height;
//
//		if (!format)
//				format = pxcImage->QueryInfo().format;
//
//		int type = 0;
//		if (format == PXCImage::PIXEL_FORMAT_Y8)
//				type = CV_8UC1;
//		else if (format == PXCImage::PIXEL_FORMAT_RGB24)
//				type = CV_8UC3;
//		else if (format == PXCImage::PIXEL_FORMAT_DEPTH_F32)
//				type = CV_32FC1;
//
//		cv::Mat ocvImage = cv::Mat(cv::Size(width, height), type, data.planes[0]);
//
//		pxcImage->ReleaseAccess(&data);
//		return ocvImage;
//}
//
//cv::Mat PXCImage2CVMatAlt(PXCImage *pxcImage, PXCImage::PixelFormat format)
//{
//		PXCImage::ImageData data;
//		pxcImage->AcquireAccess(PXCImage::ACCESS_READ, format, &data);
//
//		int width = pxcImage->QueryInfo().width;
//		int height = pxcImage->QueryInfo().height;
//		int dataWidth;
//
//		if (!format)
//				format = pxcImage->QueryInfo().format;
//
//		int type = 0;
//		if (format == PXCImage::PIXEL_FORMAT_Y8) {
//				type = CV_8UC1;
//				dataWidth = 1;
//		}
//		else if (format == PXCImage::PIXEL_FORMAT_RGB24) {
//				type = CV_8UC3;
//				dataWidth = 3;
//		}
//		else if (format == PXCImage::PIXEL_FORMAT_DEPTH_F32) {
//				type = CV_32FC1;
//				dataWidth = 4;
//		}
//
//		cv::Mat ocvImage;
//
//		ocvImage.create(height, data.pitches[0] / dataWidth, type);
//
//		memcpy(ocvImage.data, data.planes[0], height*width*dataWidth * sizeof(pxcBYTE));
//
//		pxcImage->ReleaseAccess(&data);
//		return ocvImage;
//}
//
/////http://stackoverflow.com/questions/32609341/convert-a-pxcimage-into-an-opencv-mat/32609342#32609342
//void ConvertPXCImageToOpenCVMat(PXCImage *inImg, Mat *outImg) {
//		int cvDataType;
//		int cvDataWidth;
//
//		PXCImage::ImageData data;
//		inImg->AcquireAccess(PXCImage::ACCESS_READ, &data);
//		//inImg->AcquireAccess(PXCImage::ACCESS_READ, 
//		//										PXCImage::PIXEL_FORMAT_RGB32,
//		//										PXCImage::ROTATION_90_DEGREE,
//		//										PXCImage::OPTION_ANY,
//		//										&data);
//		//inImg->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH_F32, &data);
//		PXCImage::ImageInfo imgInfo = inImg->QueryInfo();
//
//		switch (data.format) {
//				/* STREAM_TYPE_COLOR */
//		case PXCImage::PIXEL_FORMAT_YUY2: /* YUY2 image  */
//		case PXCImage::PIXEL_FORMAT_NV12: /* NV12 image */
//				throw(0); // Not implemented
//		case PXCImage::PIXEL_FORMAT_RGB32: /* BGRA layout on a little-endian machine */
//				cvDataType = CV_8UC4;
//				cvDataWidth = 4;
//				break;
//		case PXCImage::PIXEL_FORMAT_RGB24: /* BGR layout on a little-endian machine */
//				cvDataType = CV_8UC3;
//				cvDataWidth = 3;
//				break;
//		case PXCImage::PIXEL_FORMAT_Y8:  /* 8-Bit Gray Image, or IR 8-bit */
//				cvDataType = CV_8U;
//				cvDataWidth = 1;
//				break;
//
//				/* STREAM_TYPE_DEPTH */
//		case PXCImage::PIXEL_FORMAT_DEPTH: /* 16-bit unsigned integer with precision mm. */
//		case PXCImage::PIXEL_FORMAT_DEPTH_RAW: /* 16-bit unsigned integer with device specific precision (call device->QueryDepthUnit()) */
//				cvDataType = CV_16U;
//				cvDataWidth = 2;
//				break;
//		case PXCImage::PIXEL_FORMAT_DEPTH_F32: /* 32-bit float-point with precision mm. */
//				cvDataType = CV_32F;
//				cvDataWidth = 4;
//				break;
//
//				/* STREAM_TYPE_IR */
//		case PXCImage::PIXEL_FORMAT_Y16:          /* 16-Bit Gray Image */
//				cvDataType = CV_16U;
//				cvDataWidth = 2;
//				break;
//		case PXCImage::PIXEL_FORMAT_Y8_IR_RELATIVE:    /* Relative IR Image */
//				cvDataType = CV_8U;
//				cvDataWidth = 1;
//				break;
//		}
//
//		// suppose that no other planes
//		if (data.planes[1] != NULL) throw(0); // not implemented
//																					// suppose that no sub pixel padding needed
//		if (data.pitches[0] % cvDataWidth != 0) throw(0); // not implemented
//
//		outImg->create(imgInfo.height, data.pitches[0] / cvDataWidth, cvDataType);
//
//		//memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));
//		memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));
//
//		inImg->ReleaseAccess(&data);
//}
//
//void RSColorPXCImage2OCVMat(PXCImage *inImg, Mat *outImg) {
//		int cvDataType = CV_8UC4;
//		int cvDataWidth = 4;
//
//		PXCImage::ImageData data;
//		//inImg->AcquireAccess(PXCImage::ACCESS_READ, &data);
//		inImg->AcquireAccess(PXCImage::ACCESS_READ,
//				PXCImage::PIXEL_FORMAT_RGB32,
//				PXCImage::ROTATION_0_DEGREE,
//				PXCImage::OPTION_ANY,
//				&data);
//		//inImg->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH_F32, &data);
//		PXCImage::ImageInfo imgInfo = inImg->QueryInfo();
//
//		// suppose that no other planes
//		if (data.planes[1] != NULL) throw(0); // not implemented
//																					// suppose that no sub pixel padding needed
//		if (data.pitches[0] % cvDataWidth != 0) throw(0); // not implemented
//
//		outImg->create(imgInfo.height, data.pitches[0] / cvDataWidth, cvDataType);
//
//		//memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));
//		memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));
//
//		inImg->ReleaseAccess(&data);
//}
//
//void ConvertDepthPXCImageToOpenCVMat(PXCImage *inImg, Mat *outImg) {
//		int cvDataType;
//		int cvDataWidth;
//
//		PXCImage::ImageData data;
//		//inImg->AcquireAccess(PXCImage::ACCESS_READ, &data);
//		inImg->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH_F32, &data);
//		PXCImage::ImageInfo imgInfo = inImg->QueryInfo();
//
//		//cout << "data.format: " << data.format << endl;
//
//		switch (data.format) {
//				/* STREAM_TYPE_COLOR */
//		case PXCImage::PIXEL_FORMAT_YUY2: /* YUY2 image  */
//		case PXCImage::PIXEL_FORMAT_NV12: /* NV12 image */
//				throw(0); // Not implemented
//		case PXCImage::PIXEL_FORMAT_RGB32: /* BGRA layout on a little-endian machine */
//				cvDataType = CV_8UC4;
//				cvDataWidth = 4;
//				break;
//		case PXCImage::PIXEL_FORMAT_RGB24: /* BGR layout on a little-endian machine */
//				cvDataType = CV_8UC3;
//				cvDataWidth = 3;
//				break;
//		case PXCImage::PIXEL_FORMAT_Y8:  /* 8-Bit Gray Image, or IR 8-bit */
//				cvDataType = CV_8U;
//				cvDataWidth = 1;
//				break;
//
//				/* STREAM_TYPE_DEPTH */
//		case PXCImage::PIXEL_FORMAT_DEPTH: /* 16-bit unsigned integer with precision mm. */
//		case PXCImage::PIXEL_FORMAT_DEPTH_RAW: /* 16-bit unsigned integer with device specific precision (call device->QueryDepthUnit()) */
//				cvDataType = CV_16U;
//				cvDataWidth = 2;
//				break;
//		case PXCImage::PIXEL_FORMAT_DEPTH_F32: /* 32-bit float-point with precision mm. */
//				cvDataType = CV_32F;
//				cvDataWidth = 4;
//				break;
//
//				/* STREAM_TYPE_IR */
//		case PXCImage::PIXEL_FORMAT_Y16:          /* 16-Bit Gray Image */
//				cvDataType = CV_16U;
//				cvDataWidth = 2;
//				break;
//		case PXCImage::PIXEL_FORMAT_Y8_IR_RELATIVE:    /* Relative IR Image */
//				cvDataType = CV_8U;
//				cvDataWidth = 1;
//				break;
//		}
//
//		// suppose that no other planes
//		if (data.planes[1] != NULL) throw(0); // not implemented
//																					// suppose that no sub pixel padding needed
//		if (data.pitches[0] % cvDataWidth != 0) throw(0); // not implemented
//
//		outImg->create(imgInfo.height, data.pitches[0] / cvDataWidth, cvDataType);
//
//		memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));
//
//		inImg->ReleaseAccess(&data);
//}
//
//int getRSParameters() {
//		/////////////////////GET PARAMETERS FROM RS
//		// create the PXCSenseManager
//		PXCSenseManager *psm = 0;
//		psm = PXCSenseManager::CreateInstance();
//		if (!psm) {
//				wprintf_s(L"Unable to create the PXCSenseManager\n");
//				return 1;
//		}
//		// Retrieve the underlying session created by the PXCSenseManager.
//		// The returned instance is an PXCSenseManager internally managed object.
//		// Note: Do not release the session!
//		PXCSession *session_1;
//		session_1 = psm->QuerySession();
//		if (session_1 == NULL) {
//				wprintf_s(L"Session not created by PXCSenseManager\n");
//				return 2;
//		}
//		// query the session_1 version
//		PXCSession::ImplVersion ver;
//		ver = session_1->QueryVersion();
//		// print version to console
//		wprintf_s(L" Hello Intel RSSDK Version %d.%d \n", ver.major, ver.minor);
//		// enumerate all available modules that are automatically loaded with the RSSDK
//		for (int i = 0;; i++) {
//				PXCSession::ImplDesc desc;
//				if (session_1->QueryImpl(0, i, &desc) < PXC_STATUS_NO_ERROR) break;
//				// Print the module friendly name and iuid (interface unique ID)
//				wprintf_s(L"Module[%d]: %s\n", i, desc.friendlyName);
//				wprintf_s(L" iuid=%x\n", desc.iuid);
//		}
//		// close the streams (if any) and release any session_1 and processing module instances
//		psm->Release();
//}
//
//void initRS(PXCSenseManager* senseMan, cv::Size size, int rate) {
//		/// Enable modules and streams
//		// Enable blob tracking
//		//sm->EnableBlob();
//		if (g_enableColor) {
//				senseMan->EnableStream(PXCCapture::STREAM_TYPE_COLOR, size.width, size.height, rate);
//				//g_smR->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 0, 0, 0);
//		}
//		if (g_enableDepth) {
//				//g_smR->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, frameSize.width, frameSize.height, frameRate);
//				senseMan->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 0, 0, 0); 		// c: 628, r: 468?
//		}
//		// Initialization
//		senseMan->Init();
//		std::cout << "pxc status: " << senseMan->AcquireFrame(true) << std::endl;
//}
//
//void printMatProperties(cv::Mat* input, char* name) {
//		cout << "___" << name << "___\n";
//		cout << "Size: " << input->cols << "x" << input->rows << endl;
//		cout << "Depth: " << input->depth() << endl;
//		cout << "Type: " << input->type() << endl;
//		cout << "Channels: " << input->channels() << endl;
//}
//
//void initTBs() {
//		namedWindow(hue_win);
//		namedWindow(morph_win);
//		namedWindow(blur_win);
//		namedWindow(canny_win);
//		createTrackbar(track1, hue_win, &tb1_value, 255, NULL);
//		createTrackbar(track2, hue_win, &tb2_value, 255, NULL);
//		createTrackbar(track3, hue_win, &tb3_value, 255, NULL);
//		createTrackbar(track4, hue_win, &tb4_value, 255, NULL);
//		createTrackbar(track5, hue_win, &tb5_value, 255, NULL);
//		createTrackbar(track6, hue_win, &tb6_value, 255, NULL);
//		createTrackbar(track7, morph_win, &tb7_numMorph, 20, NULL);
//		createTrackbar(track8, blur_win, &tb8_numBlur, 20, NULL);
//		createTrackbar(track9, canny_win, &tb9_lowThresh, MAXLOWTHRESH, NULL);
//		tb9CB_canny(0, 0);
//}
//// CB functions - I think I can get rid of these since they're not called
//void tb1CB_minH(int value, void *) {
//		//cv::imshow(hue_win, g_hueMask);
//		return;
//}
//void tb2CB_maxH(int value, void *) {
//		return;
//}
//void tb3CB_minS(int value, void *) {
//		return;
//}
//void tb4CB_maxS(int value, void *) {
//		return;
//}
//void tb5CB_minV(int value, void *) {
//		return;
//}
//void tb6CB_maxV(int value, void *) {
//		return;
//}
//void tb7CB_numMorph(int value, void *) {
//		return;
//}
//void tb8CB_numBlur(int value, void *) {
//		return;
//}
//void tb9CB_canny(int value, void *) {
//		//Mat blurGray;// , edges;
//		//g_morph.convertTo(blurGray, CV_8UC1);
//		//blur(blurGray, g_canny, Size(3, 3));
//		//Canny(g_canny, g_canny, tb9_lowThresh, tb9_lowThresh*3, 3);
//		return;
//}