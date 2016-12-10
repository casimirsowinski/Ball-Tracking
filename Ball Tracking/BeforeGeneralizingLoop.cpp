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
//bool g_enableDepth = true;
//bool g_enableColorSegmentation = true;
//bool g_enableCBs = true;
//bool g_disableLastingTrails = true;
//// CB 
//const char track1[] = "H Center";
//const char track2[] = "H Range";
//const char track3[] = "S Min";
//const char track4[] = "S Max";
//const char track5[] = "V Min";
//const char track6[] = "V Max";
//const char track7[] = "Openings";
//const char track8[] = "Blurs";
//const char track9[] = "Canny Th";
//const char track10[] = "Canny Rat";
//const char track11[] = "Depth";
//int tb1_HCenter = 57;							// empirically found hue center
//int tb2_HRange = 16;							// empirically found hue range
//int tb3_SMin = 0;							// empirically found S min
//int tb4_SMax = 255;						// empirically found S max
//int tb5_VMin = 0;							// empirically found V min
//int tb6_VMax = 255;						// empirically found V max
//int tb7_numMorph = 6;					// empirically found number of times to open image for ROI location
//int tb8_numBlur = 1;						// empirically found number of times to blur...something
//int tb9_lowThresh = 1;					// empirically found canny lowrer threshold
//int tb10_threshRatio = 2;
//int tb11_depthMult = 10;
//const int MAXLOWTHRESH = 200;
//const int MAXTHRESHRATIO = 10;
//const int MAXDEPTHMULT = 100;
//
//Mat g_hueMask;
//Mat g_morphROI;
//Mat g_cannyROI;
//Mat g_morphFine;
//Mat g_cannyFine;
//Mat g_blur;
//Mat g_smoothedMorphROI;
//Mat g_trailOverlay;
//Mat g_depth8U;
//cv::Mat g_depthCrop;
//
//const char hue_win[] = "hue mask";
//const char morphROI_win[] = "morph ROI";
//const char cannyROI_win[] = "canny ROI";
//const char morphFine_win[] = "morph Fine";
//const char cannyFine_win[] = "canny Fine";
//const char blur_win[] = "blur";
//const char color_win[] = "color";
//const char ROI1_win[] = "ROI1";
//const char ROI2_win[] = "ROI2";
//const char depth_win[] = "Depth";
//const char depthCrop_win[] = "Depth Crop";
//const int MINCONTOURWIDTH = 2;
//// trail stuff
//bool g_beginTrail = true;						// trail reset when starting or considered stale
//const int MAXSTALE = 5;						// when g_trailstale reaches this value, trail is considered lost
//int g_trailStaleness = 0;								// increments when no ball found until MAXSTALE
//const int TRAILSIZE = 61;
//
///// Function declarations
//void initTBs();																									// initialize trackbars
//int getRSParameters();																					// get general properties/modules of RS
//void initRS(PXCSenseManager*, cv::Size colorSize,cv::Size depthSize, int rate);					// set up session(s)
//void printMatProperties(cv::Mat*, char*);												// print Mat information, size, etc.
//cv::Rect getRect(vector<vector<Point>> contours, int ROIBuffer, cv::Size windowSize, bool isDepth);
//void overlayImage(cv::Mat src, cv::Mat dst, cv::Point topLeftCorner);
//// RS to OCV conversion functions
//cv::Mat PXCImage2CVMat(PXCImage*, PXCImage::PixelFormat);				// original-ish version
//cv::Mat PXCImage2CVMatAlt(PXCImage*, PXCImage::PixelFormat);		// mix of PXCI... and ConvertP...
//void ConvertPXCImageToOpenCVMat(PXCImage*, Mat*);								// original-ish version
//void RSColorPXCImage2Mat(PXCImage*, Mat*);										// Only for Color camera
//void ConvertDepthPXCImageToOpenCVMat(PXCImage*, Mat*);					// specifically for depth, not complete
//// Mat color conversion funtions
//cv::Mat depth2DepthColor(Mat*, int, int);												// to make rainbow version, not working
//cv::Mat depth2DepthColor(Mat*);																	// to make rainbow version, not working
//// CB functions
//void tb1CB_minH(int value, void *);
//void tb2CB_maxH(int value, void *);
//void tb3CB_minS(int value, void *);
//void tb4CB_maxS(int value, void *);
//void tb5CB_minV(int value, void *);
//void tb6CB_maxV(int value, void *);
//void tb7CB_numMorph(int value, void *);
//void tb8CB_numBlur(int value, void *);
//void tb9CB_cannyLoTh(int value, void *);
//void tb10CB_cannyThRat(int value, void *);
//void tb11CB_depth(int value, void *);
//
///// For contours and ROI
//int g_maxIndex = 0;
//Rect g_rectBound;
//int g_maxArea = 0;
//const int MINCONTOURAREA = 500;
//
//int wmain(int argc, WCHAR* argv[]) {
//		/// Initialize Trackbars
//		initTBs();
//
//		/// Desired OpenCV parameters
//		int frameRate = 60;
//		cv::Size frameSizeC = cv::Size(640, 480);
//		cv::Size frameSizeD = cv::Size(628, 468); //c: 628, r : 468 ?
//		cv::Mat frameColorR = cv::Mat::zeros(frameSizeC, CV_8UC3);
//		cv::Mat frameDepthR;		
//		//cv::Mat frameDepthR = cv::Mat::zeros(frameSizeC, CV_32FC1);
//		//cv::Mat frameDepthRAlt = cv::Mat::zeros(frameSizeD, CV_32FC1);
//		//Mat frameColorBL = cv::Mat::zeros(frameSizeC, CV_8UC3);
//		////cv::Mat frameColorL = cv::Mat::zeros(frameSizeC, CV_8UC3);
//		////cv::Mat frameDepthL = cv::Mat::zeros(frameSizeC, CV_8UC1);
//		////cv::Mat frameDepthLAlt = cv::Mat::zeros(frameSizeC, CV_8UC1);
//		////cv::Mat frameDepthRColor = cv::Mat::zeros(frameSizeC, CV_16UC3);
//		g_smoothedMorphROI.create(frameSizeC, CV_8UC1);
//		g_trailOverlay.create(frameSizeC, CV_8UC3);
//		cv::Mat output_1;
//
//		/// RS stuff
//		//getRSParameters();
//		initRS(g_smR, frameSizeC, frameSizeD, frameRate);				// init right RealSense
//		////initRS(g_smL, frameSizeC, frameRate);		// init left RealSense
//
//		/// For contours
//		int ROIBuffC = 10;						// width of buffer around ROI fround with boundingRect on largest contour
//		int ROIBuffD = 10;
//		vector<vector<Point>> contours; // Vector for storing contour
//		vector<Vec4i> hierarchy;
//		Scalar color(0, 0, 255);
//		Point trail[TRAILSIZE] = { Point(0, 0) };
//		int trailAge[TRAILSIZE] = { 10 };
//		//bool trailConnected[TRAILSIZE] = { false };
//
//		/// Main loop, stream data, process data
//		// Loop vars
//		bool connectedToRS = false;							// Track whether main loop entered/connected		
//		//cv::Mat aveArray[3] = { cv::Mat::zeros(frameSizeC, CV_32FC3) }; // array to keep track of last 3 frames
//		//cv::Mat acc = cv::Mat::zeros(frameSizeC, CV_32FC3); // accumulator for averaging frames
//		//cv::Mat acc8U = cv::Mat::zeros(frameSizeC, CV_8UC3); // 
//		//cv::Mat runningAve8U = cv::Mat::zeros(frameSizeC, CV_8UC3); // rounded average of last three frames
//		
//		while (g_smR->AcquireFrame(true) >= PXC_STATUS_NO_ERROR) {
//				connectedToRS = true;								// Flip warning flag
//
//				/// Aquire data
//				PXCCapture::Sample *sampleR;
//				////PXCCapture::Sample *sampleL;
//				sampleR = g_smR->QuerySample();
//				////sampleL = g_smL->QuerySample();
//
//				PXCImage* colorImage = sampleR->color;
//				PXCImage* depthImage = sampleR->depth;
//
//				if (colorImage && g_enableColor) {
//						//frameColorR = PXCImage2CVMat(sampleR->color, PXCImage::PIXEL_FORMAT_RGB24);
//						//ConvertPXCImageToOpenCVMat(sampleR->color, &frameColorR); //<- last used
//						RSColorPXCImage2Mat(sampleR->color, &frameColorR); //only for colorimage
//						////ConvertPXCImageToOpenCVMat(sampleL->color, &frameColorL);
//				}
//				if (depthImage && g_enableDepth) {
//						ConvertPXCImageToOpenCVMat(sampleR->depth, &frameDepthR);
//						/// Make cropped depth image to get effective area
//						float borderSizeX = 0.03; // 0.1 takes 10% off all edges, 0.2 takes off 20%...
//						float borderSizeY = 0.03; // 0.1 takes 10% off all edges, 0.2 takes off 20%...
//						int newX = cvRound(frameDepthR.cols * borderSizeX);
//						int newY = cvRound(frameDepthR.rows * borderSizeY);
//						int newW = cvRound(frameDepthR.cols * (1 - 2 * borderSizeX));
//						int newH = cvRound(frameDepthR.rows * (1 - 2 * borderSizeY));
//						Rect crop(newX, newY, newW, newH);
//						//cout << "x: " << newX << ", y: " << newY << ", w: " << newW << ", h: " << newH << endl;
//						g_depthCrop = frameDepthR(crop);
//						GaussianBlur(frameDepthR, frameDepthR, Size(3, 3), 0, 0, BORDER_CONSTANT);
//				}
//				imshow(depthCrop_win, g_depthCrop * tb11_depthMult);
//
//				/// Manipulate data, Find tennis ball by its Hue
//				if (g_enableColorSegmentation && g_enableColor) {
//						/// Strip the alpha channel first
//						cvtColor(frameColorR, frameColorR, CV_BGRA2BGR);
//
//						//bilateralFilter(frameColorR, frameColorBL, 9, 150, 150);
//						//imshow("bilateral test", frameColorBL);
//
//						//accumulateWeighted(frameHSV, acc, 0.5);
//						//imshow("acc", acc / 255);
//						//acc.convertTo(acc8U, CV_8UC3);
//
//						///Reset if no lasting trails wanted
//						if (g_disableLastingTrails) {
//								g_trailOverlay = 0;
//						}
//												
//						/// Convert to HSV
//						Mat frameHSV;
//						cvtColor(frameColorR, frameHSV, CV_RGB2HSV);
//
//						/// Two sided threshold to find a particular Hue range
//						/// Convert taskbar 1 and 2 values to a center and range, ensure ranges
//						int HBoundLo = tb1_HCenter - tb2_HRange;
//						int HBoundHi = tb1_HCenter + tb2_HRange;
//						if (HBoundLo < 0) HBoundLo = 0;
//						if (HBoundHi > 255) HBoundHi = 255;
//						Scalar lower(HBoundLo, tb3_SMin, tb5_VMin);
//						Scalar upper(HBoundHi, tb4_SMax, tb6_VMax);
//						inRange(frameHSV, lower, upper, g_hueMask);
//
//						/// Open image tb7_numMorph times to get rid of some noise
//						Mat kernel = Mat::ones(3, 3, CV_8U);
//						morphologyEx(g_hueMask, g_morphROI, MORPH_OPEN, kernel, Point(-1, -1), tb7_numMorph, BORDER_REFLECT);
//												
//						/// Copy g_morphROI into grayscale, run smoothing on it
//						//cvtColor(g_morphROI, g_smoothedMorphROI, );
//						g_morphROI.copyTo(g_smoothedMorphROI);
//
//						/// Blur -------------Disabled now, seems to be making opening less effective
//						for (int i = 1; i < tb8_numBlur; i++) {
//								blur(g_smoothedMorphROI, g_smoothedMorphROI, Size(7, 7));
//								//medianBlur(g_smoothedMorphROI, g_smoothedMorphROI, 3);
//								//GaussianBlur(g_smoothedMorphROI, g_smoothedMorphROI, Size(3, 3), 0, 0, BORDER_CONSTANT);
//						}
//
//						/// Find rough edges with Canny for ROI
//						Canny(g_smoothedMorphROI, g_cannyROI, tb9_lowThresh, tb9_lowThresh * tb10_threshRatio, 5);
//						
//						/// Find contours, find the biggest contour for ROI
//						findContours(g_cannyROI, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image
//						
//						Rect testColorROI = getRect(contours, ROIBuffC, frameSizeC, false);
//						Rect testDepthROI = getRect(contours, ROIBuffD, g_depthCrop.size(), true);
//						Rect colorROI;
//						Rect depthROI;
//
//						if (g_maxArea > MINCONTOURAREA) {
//								colorROI = testColorROI;
//								depthROI = testDepthROI;
//						}
//
//						/// Check if any contours exist, draw the max contour if they do
//						//int contSize = contours.size();
//						//cout << "area: " << g_maxArea << endl;
//						//if (contours.size() && g_rectBound.width > MINCONTOURWIDTH) {
//						if (contours.size() && g_maxArea > MINCONTOURAREA) {
//								
//								rectangle(g_trailOverlay, g_rectBound, Scalar(0, 255, 0), 1, 8, 0);
//								int radius = g_rectBound.width / 2;
//								Point center(g_rectBound.x + radius, g_rectBound.y + radius);
//								circle(g_trailOverlay, center, radius, Scalar(255, 0, 0), 3, 8, 0);
//								// Draw the largest contour using previously stored index.
//								drawContours(g_trailOverlay, contours, g_maxIndex, color, 4, 8, hierarchy);
//																
//								/// set all trail elements to new center if new trail, otherwise
//								/// only set head of trail								
//								if (g_beginTrail) {
//										for (int i = 0; i < TRAILSIZE; i++) {
//												trail[i] = center;												
//										}
//								}
//								else {
//										trail[0] = center;
//								}
//								/// Draw trail
//								for (int i = TRAILSIZE; i > 0; i--) {
//										trail[i] = trail[i - 1];
//								}
//								g_trailStaleness = 0;		// refresh trail
//								g_beginTrail = false;		// refresh trail
//						} else { 
//								g_trailStaleness++;
//								if (g_trailStaleness == MAXSTALE) {
//										g_beginTrail = true;
//										g_trailStaleness = 0;
//								}
//						}
//
//						//cout << "g_trailStaleness: " << g_trailStaleness << ", g_beginTrail: " << g_beginTrail << endl;				
//						for (int i = TRAILSIZE; i > 0; i--) {
//								trailAge[i]--;
//								if (trailAge[i] < 0) trailAge[i] = 0;
//								//trail[i] = trail[i - 1];
//								int trailWidth = cvRound(16 - i * 15 / TRAILSIZE);
//								Scalar trailColor(252, 255 - i * 226 / TRAILSIZE, 28 + i * 119 / TRAILSIZE);
//								line(g_trailOverlay, trail[i - 1], trail[i], trailColor, trailWidth, 8, 0);
//						}
//						
//						/// Combine overlay and main image
//						addWeighted(frameColorR, 1.0, g_trailOverlay, 0.8, 0.0, output_1);
//						
//						/// Use ROI to look at a less opened image
//						Mat ROI_1 = frameColorR(colorROI);
//						if (ROI_1.data) {
//								imshow(ROI1_win, ROI_1);
//						}
//						//Mat ROI_2 = frameDepthR(depthROI) * tb11_depthMult;
//						Mat ROI_2 = g_depthCrop(depthROI) * tb11_depthMult;
//						GaussianBlur(ROI_2, ROI_2, Size(3, 3), 0, 0, BORDER_CONSTANT);
//						if (ROI_2.data) {
//								imshow(ROI2_win, ROI_2);
//						}
//
//						/// Overlay ROI onto output window
//						overlayImage(ROI_1, output_1, Point(0, 0));
//						overlayImage(ROI_2, output_1, Point(output_1.cols, 0));
//
//						/// Get ROI from center of ROI, find average value, call that depth, convert it to real world units
//						float rawSizeX = 0.4; // 0.1 takes 10% off all edges, 0.2 takes off 20%...
//						float rawSizeY = 0.4; // 0.1 takes 10% off all edges, 0.2 takes off 20%...
//
//						int rawROIx = cvRound(ROI_2.cols * rawSizeX);
//						int rawROIy = cvRound(ROI_2.rows * rawSizeY);
//						int rawROIw = cvRound(ROI_2.cols * (1 - 2 * rawSizeX));
//						int rawROIh = cvRound(ROI_2.rows * (1 - 2 * rawSizeY));
//
//						Rect rawROI(rawROIx, rawROIy, rawROIw, rawROIh);
//						Mat depthValueRaw = ROI_2(rawROI);
//
//						Scalar averageDepth = mean(depthValueRaw);
//						cout << "averageDepth: " << averageDepth[0] << endl;
//								//<< ", x: " << rawROIx << ", y: " << rawROIy << ", w: " << rawROIw << ", h: " << rawROIh << endl;
//
//						if (depthValueRaw.data) {
//								namedWindow("raw", CV_WINDOW_NORMAL);
//								//imshow("raw", depthValueRaw);
//						}
//
//						// threshold, canny, find circles or something
//
//						// get closest value in depth image
//						double min, max;
//						cv::minMaxLoc(ROI_2, &min, &max);
//						//cout << "Max Z? " << max << endl;
//				}
//
//
//				
//
//
//				/// Display data
//				if (g_enableColor) {
//						cv::imshow(color_win, output_1);
//						//cv::imshow("Color Right", output);
//						////cv::imshow("Color Left", frameColorL);
//						//cv::imshow("Color", )
//						if (g_enableColorSegmentation) {
//								cv::imshow(hue_win, g_hueMask);
//								cv::imshow(morphROI_win, g_morphROI);
//								cv::imshow(blur_win, g_smoothedMorphROI);
//								cv::imshow(cannyROI_win, g_cannyROI);
//						
//								//cv:imshow("gray blur", blur);
//								//cv::imshow("denoised hue", deNoised);
//						}
//
//				}
//				if (g_enableDepth) {
//						//cv::imshow("DepthAlt", frameDepthRAlt);
//						//cv::imshow(depth_win, frameDepthR * tb11_depthMult);
//						//imshow("Depth8U: ", g_depth8U);
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
//								//printMatProperties(&src, "src");
//								//printMatProperties(&dst, "dst");
//								//printMatProperties(&thr, "thr");
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
//						printMatProperties(&g_morphROI, "morph");
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
///// Function Definitions
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
//		memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));
//
//		inImg->ReleaseAccess(&data);
//}
//void RSColorPXCImage2Mat(PXCImage *inImg, Mat *outImg) {
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
///*This function is based on Intel's Getting Started Guide for the RealSense.
//https://software.intel.com/sites/default/files/Getting_Started_0.pdf */
//int getRSParameters() {
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
///*Initialize the RealSense. Currently needs globals for flags for whether color streams and/or
//depth streams are to be collected. */
//void initRS(PXCSenseManager* senseMan, cv::Size colorSize, cv::Size depthSize, int rate) {
//		/// Enable modules and streams
//		// Enable blob tracking
//		//sm->EnableBlob();
//		if (g_enableColor) {
//				senseMan->EnableStream(PXCCapture::STREAM_TYPE_COLOR, colorSize.width, colorSize.height, rate);
//				//g_smR->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 0, 0, 0);
//		}
//		if (g_enableDepth) {
//				//g_smR->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, frameSizeC.width, frameSizeC.height, frameRate);
//				//senseMan->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 0, 0, 0); 		// c: 628, r: 468?
//				senseMan->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, depthSize.width, depthSize.height, rate); 		// c: 628, r: 468?
//		}		
//		// Initialization
//		senseMan->Init();
//		std::cout << "initRS:\n\tPXC status: " << senseMan->AcquireFrame(true) << std::endl;
//}
///*Prints some properties of Mat for debugging*/
//void printMatProperties(cv::Mat* input, char* name) {
//		cout << "___" << name << "___\n";
//		cout << "Size: " << input->cols << "x" << input->rows << endl;
//		cout << "Depth: " << input->depth() << endl;
//		cout << "Type: " << input->type() << endl;		
//		cout << "Channels: " << input->channels() << endl;
//		cout << "Flags: " << input->flags << endl;		
//}
///*Overlays a smaller image (src) onto larger image (dst) at point given. Returns error if src
//is not smaller than dst. If point given makes dst out of range, point is adjusted to show all
//of src in dst.*/
//void overlayImage(cv::Mat src, cv::Mat dst, cv::Point topLeftCorner) {
//		/*Check that the images have the same number of channels */
//		int dstChan = dst.channels();
//		int srcChan = src.channels();
//		if (dstChan != srcChan) {
//				std::cout << "overlayImage\n\tError. Destination and source images must have the same number of channels.\n";
//				return;
//		}	
//		
//		/*Check that src fits into dst*/
//		if (src.rows > dst.rows || src.cols > dst.cols) {
//				std::cout << "overlayImage\n\tError. Source image larger than destination image.\n";
//				return;
//		}
//
//		/*Check that src is in range of dst. If any part of src would be put outside of dst then
//		correct the top left corner to keep src within bounds*/
//		int cornerX = topLeftCorner.x;
//		int cornerY = topLeftCorner.y;
//
//		if (cornerX < 0) cornerX = 0;
//		if (cornerY < 0) cornerY = 0;
//
//		if (topLeftCorner.x + src.cols > dst.cols) {
//				cornerX = dst.cols - src.cols;
//		}
//		if (topLeftCorner.y + src.rows > dst.rows) {
//				cornerY = dst.rows - src.rows;
//		}
//
//		/*Check that neither src or dst don't have 2 channels (not implemented)*/
//		//if (dstChan == 2 || srcChan == 2){
//		//		std::cout << "overlayImage Error. Not implemented for 2 channel images.\n";
//		//		return;
//		//}
//
//		/*Scan through rows starting from the row to start adding to unitl that
//		row plus the height of the image to add*/
//		for (int y = cornerY; y < cornerY + src.rows; y++) {
//				/*Get ptr to dst image at the row to start copying the pixels*/
//				Vec3b* dstPtr = dst.ptr<Vec3b>(y);
//				/*Get ptr to src image at its beginning row to copy pixels from*/
//				Vec3b* srcPtr = src.ptr<Vec3b>(y - cornerY);
//				/*Scan though columns starting from the column to start adding to
//				until that column plus the width of the image to add*/
//				for (int x = cornerX; x < cornerX + src.cols; x++) {
//						for (int c = 0; c < dstChan; c++) {
//								dstPtr[x][c] = srcPtr[x - cornerX][c];
//						}
//						/*For only 3 channel src and dst Mats*/
//						//dstPtr[x][0] = srcPtr[x - cornerX][0];
//						//dstPtr[x][1] = srcPtr[x - cornerX][1];
//						//dstPtr[x][2] = srcPtr[x - cornerX][2];
//						////{For if src has only one channel and dst has multiple (DOESN'T WORK)}
//						////for (int c = 0; c < dst.channels; c++) {
//						////		/// If src has one channel than set all of dst's channels to channel 0 of src
//						////		int cMod = c;
//						////		int c
//						////		dstPtr[x][c] = srcPtr[x - cornerX][c];
//						////}
//						//////{For if other conditions, not fully tested}
//						//////if (srcChan = dstChan) {
//						//////		for (int c = 0; c < srcChan; c++) {
//						//////				dstPtr[x][c] = srcPtr[x - cornerX][c];
//						//////		}
//						//////}
//						///////*if src is single channel and dst 3 or 4 channels then add the only channel
//						//////of src to first three channels of dst*/
//						//////else if (dstChan == 3 || dstChan == 4 && srcChan == 1) {
//						//////		for (int c = 0; c < 3; c++) {
//						//////				dstPtr[x][c] = srcPtr[x - cornerX][0];
//						//////		}
//						//////		
//						//////}
//				}
//		}
//		return;
//}
///*Returns a Rect that is over the largest contour in a set of them, plus a buffer in all
//directions */
//cv::Rect getRect(vector<vector<Point>> contours, int ROIBuffer, cv::Size windowSize, bool isDepth) {
//		int contSize = contours.size();
//		g_maxArea = 0;
//		g_maxIndex = 0;		
//		Rect rectROI;
//
//		for (int i = 0; i < contSize; i++)			// iterate through each contour. 
//		{
//				double area = contourArea(contours[i], false);  //  Find the area of contour
//				if (area > g_maxArea) {
//						g_maxArea = area;								// Store the area of the largest contour
//						g_maxIndex = i;									//Store the index of largest contour
//				}
//		}
//		/*Return correct Rect if there are any contours. Return empty Rect if not*/
//		if (contSize) {
//				/// Find the bounding rectangle for biggest contour
//				g_rectBound = boundingRect(contours[g_maxIndex]); 
//				/// Put buffer around bounding_rect to make larger ROI, make sure ROI doesn't go out of bounds
//				int ROIx = g_rectBound.x - ROIBuffer;
//				int ROIy = g_rectBound.y - ROIBuffer;
//				int ROIw = g_rectBound.width + 2 * ROIBuffer;
//				int ROIh = g_rectBound.height + 2 * ROIBuffer;
//				/// Correct for smaller depth window (NOT IMPLEMENTED)
//				if (isDepth) {
//						//ROIx = cvRound(ROIx * 640 / 640);
//						//ROIy = cvRound(ROIy * 468 / 480) + 12;
//						//ROIw = cvRound(ROIw * 640 / 640);
//						//ROIh = cvRound(ROIh * 468 / 480) + 12;
//				}
//				if (ROIx < 0) ROIx = 0;
//				if (ROIy < 0) ROIy = 0;
//				if ((ROIx + ROIw) > windowSize.width) ROIw = windowSize.width - ROIx;
//				if ((ROIy + ROIh) > windowSize.height) ROIh = windowSize.height - ROIy;
//				if (isDepth) {
//						//cout << "x: " << ROIx << ", y: " << ROIy << ", w: " << ROIw << ", h: " << ROIh << endl;
//				}
//				rectROI = Rect(ROIx, ROIy, ROIw, ROIh);
//		}		
//		return rectROI;
//}
///*Sets up GUI elements, trackbars, windows*/
//void initTBs() {
//		/*Make windows*/
//		namedWindow(hue_win, CV_WINDOW_NORMAL);
//		namedWindow(morphROI_win, CV_WINDOW_NORMAL);
//		namedWindow(blur_win, CV_WINDOW_NORMAL);
//		namedWindow(cannyROI_win, CV_WINDOW_NORMAL);
//		namedWindow(color_win, CV_WINDOW_NORMAL);
//		namedWindow(ROI1_win, CV_WINDOW_NORMAL);
//		namedWindow(ROI2_win, CV_WINDOW_NORMAL);
//		namedWindow(depth_win, CV_WINDOW_NORMAL);
//		namedWindow(depthCrop_win, CV_WINDOW_NORMAL);
//		/*Create Trackbars*/
//		createTrackbar(track1, hue_win, &tb1_HCenter, 255, NULL);
//		createTrackbar(track2, hue_win, &tb2_HRange, 255, NULL);
//		createTrackbar(track3, hue_win, &tb3_SMin, 255, NULL);
//		createTrackbar(track4, hue_win, &tb4_SMax, 255, NULL);
//		createTrackbar(track5, hue_win, &tb5_VMin, 255, NULL);
//		createTrackbar(track6, hue_win, &tb6_VMax, 255, NULL);
//		createTrackbar(track7, morphROI_win, &tb7_numMorph, 20, NULL);
//		createTrackbar(track8, blur_win, &tb8_numBlur, 20, NULL);
//		createTrackbar(track9, cannyROI_win, &tb9_lowThresh, MAXLOWTHRESH, NULL);
//		createTrackbar(track10, cannyROI_win, &tb10_threshRatio, MAXTHRESHRATIO, NULL);
//		createTrackbar(track11, depth_win, &tb11_depthMult, MAXDEPTHMULT, NULL);
//		/*Run CBs*/
//		tb1CB_minH(tb1_HCenter, 0);
//		tb2CB_maxH(tb2_HRange, 0);
//		tb3CB_minS(tb3_SMin, 0);
//		tb4CB_maxS(tb4_SMax, 0);
//		tb5CB_minV(tb5_VMin, 0);
//		tb6CB_maxV(tb6_VMax, 0);
//		tb7CB_numMorph(tb7_numMorph, 0);
//		tb8CB_numBlur(tb8_numBlur, 0);
//		tb9CB_cannyLoTh(tb9_lowThresh, 0);
//		tb11CB_depth(tb11_depthMult, 0);		
//}
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
//void tb9CB_cannyLoTh(int value, void *) {
//		//Mat blurGray;// , edges;
//		//g_morphROI.convertTo(blurGray, CV_8UC1);
//		//blur(blurGray, g_cannyROI, Size(3, 3));
//		//Canny(g_cannyROI, g_cannyROI, tb9_lowThresh, tb9_lowThresh*3, 3);		
//		return;
//}
//void tb10CB_cannyThRat(int value, void *) {
//		return;
//}
//void tb11CB_depth(int value, void *) {
//		return;
//}
//
//
//
//////////////////////////////////NOT USED\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//
////void ConvertDepthPXCImageToOpenCVMat(PXCImage *inImg, Mat *outImg) {
////		int cvDataType;
////		int cvDataWidth;
////
////		PXCImage::ImageData data;
////		//inImg->AcquireAccess(PXCImage::ACCESS_READ, &data);
////		inImg->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH_F32, &data);
////		PXCImage::ImageInfo imgInfo = inImg->QueryInfo();
////
////		//cout << "data.format: " << data.format << endl;
////
////		switch (data.format) {
////				/* STREAM_TYPE_COLOR */
////		case PXCImage::PIXEL_FORMAT_YUY2: /* YUY2 image  */
////		case PXCImage::PIXEL_FORMAT_NV12: /* NV12 image */
////				throw(0); // Not implemented
////		case PXCImage::PIXEL_FORMAT_RGB32: /* BGRA layout on a little-endian machine */
////				cvDataType = CV_8UC4;
////				cvDataWidth = 4;
////				break;
////		case PXCImage::PIXEL_FORMAT_RGB24: /* BGR layout on a little-endian machine */
////				cvDataType = CV_8UC3;
////				cvDataWidth = 3;
////				break;
////		case PXCImage::PIXEL_FORMAT_Y8:  /* 8-Bit Gray Image, or IR 8-bit */
////				cvDataType = CV_8U;
////				cvDataWidth = 1;
////				break;
////
////				/* STREAM_TYPE_DEPTH */
////		case PXCImage::PIXEL_FORMAT_DEPTH: /* 16-bit unsigned integer with precision mm. */
////		case PXCImage::PIXEL_FORMAT_DEPTH_RAW: /* 16-bit unsigned integer with device specific precision (call device->QueryDepthUnit()) */
////				cvDataType = CV_16U;
////				cvDataWidth = 2;
////				break;
////		case PXCImage::PIXEL_FORMAT_DEPTH_F32: /* 32-bit float-point with precision mm. */
////				cvDataType = CV_32F;
////				cvDataWidth = 4;
////				break;
////
////				/* STREAM_TYPE_IR */
////		case PXCImage::PIXEL_FORMAT_Y16:          /* 16-Bit Gray Image */
////				cvDataType = CV_16U;
////				cvDataWidth = 2;
////				break;
////		case PXCImage::PIXEL_FORMAT_Y8_IR_RELATIVE:    /* Relative IR Image */
////				cvDataType = CV_8U;
////				cvDataWidth = 1;
////				break;
////		}
////		// suppose that no other planes
////		if (data.planes[1] != NULL) throw(0); // not implemented
////																					// suppose that no sub pixel padding needed
////		if (data.pitches[0] % cvDataWidth != 0) throw(0); // not implemented
////
////		outImg->create(imgInfo.height, data.pitches[0] / cvDataWidth, cvDataType);
////
////		memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));
////
////		inImg->ReleaseAccess(&data);
////}
////cv::Mat PXCImage2CVMat(PXCImage *pxcImage, PXCImage::PixelFormat format)
////{
////		PXCImage::ImageData data;
////		pxcImage->AcquireAccess(PXCImage::ACCESS_READ, format, &data);
////
////		int width = pxcImage->QueryInfo().width;
////		int height = pxcImage->QueryInfo().height;
////
////		if (!format)
////				format = pxcImage->QueryInfo().format;
////
////		int type = 0;
////		if (format == PXCImage::PIXEL_FORMAT_Y8)
////				type = CV_8UC1;
////		else if (format == PXCImage::PIXEL_FORMAT_RGB24)
////				type = CV_8UC3;
////		else if (format == PXCImage::PIXEL_FORMAT_DEPTH_F32)
////				type = CV_32FC1;
////
////		cv::Mat ocvImage = cv::Mat(cv::Size(width, height), type, data.planes[0]);
////
////		pxcImage->ReleaseAccess(&data);
////		return ocvImage;
////}
////cv::Mat PXCImage2CVMatAlt(PXCImage *pxcImage, PXCImage::PixelFormat format)
////{
////		PXCImage::ImageData data;
////		pxcImage->AcquireAccess(PXCImage::ACCESS_READ, format, &data);
////
////		int width = pxcImage->QueryInfo().width;
////		int height = pxcImage->QueryInfo().height;
////		int dataWidth;
////
////		if (!format)
////				format = pxcImage->QueryInfo().format;
////
////		int type = 0;
////		if (format == PXCImage::PIXEL_FORMAT_Y8) {
////				type = CV_8UC1;
////				dataWidth = 1;
////		}
////		else if (format == PXCImage::PIXEL_FORMAT_DEPTH_RAW) {
////				type = CV_16U;
////				dataWidth = 2;
////		}
////		else if (format == PXCImage::PIXEL_FORMAT_DEPTH_F32) {
////				type = CV_32FC1;
////				dataWidth = 4;
////		}
////
////		cv::Mat ocvImage;
////
////		ocvImage.create(height, data.pitches[0] / dataWidth, type);
////
////		memcpy(ocvImage.data, data.planes[0], height*width*dataWidth * sizeof(pxcBYTE));
////
////		pxcImage->ReleaseAccess(&data);
////		return ocvImage;
////}