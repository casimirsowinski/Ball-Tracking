///*
//Casimir Sowinski
//ECE-510 Embedded Vision 1
//Project
//Ball tracker using color and depth cameras
//*/
//#include "pxcsensemanager.h"
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//
//using namespace std;
//using namespace cv;
//
///// Global vars
///*RealSense*/
//PXCSenseManager *g_sessionManager = PXCSenseManager::CreateInstance();
//
///*Callback vars*/
//int tb1_HCenter = 57;								// empirically found hue center
//int tb2_HRange = 16;								// empirically found hue range
//int tb3_SMin = 0;										// empirically found S min
//int tb4_SMax = 255;									// empirically found S max
//int tb5_VMin = 0;										// empirically found V min
//int tb6_VMax = 255;									// empirically found V max
//int tb7_numMorph = 6;								// empirically found number of times to open image for ROI location
//int tb8_numBlur = 1;								// empirically found number of times to blur...something
//int tb9_lowThresh = 1;							// empirically found canny lowrer threshold
//int tb10_threshRatio = 2;
//int tb11_depthMult = 10;
///*Callback constants*/
//const int MAXLOWTHRESH = 200;
//const int MAXTHRESHRATIO = 10;
//const int MAXDEPTHMULT = 100;
///*Callback window names*/
//const char HUE_WIN[] = "hue mask";
//const char MORPH1_WIN[] = "morph ROI";
//const char CANNY1_WIN[] = "canny ROI";
//const char MORPH2_WIN[] = "morph Fine";
//const char CANNY2_WIN[] = "canny Fine";
//const char BLUR_WIN[] = "blur";
//const char COLOR_WIN[] = "color";
//const char ROI1_WIN[] = "ROI1";
//const char ROI2_WIN[] = "ROI2";
//const char DEPTH_WIN[] = "Depth";
//const char DEPTHCROP_WIN[] = "Depth Crop";
//const int MINCONTOURWIDTH = 2;
///*Contour vars*/
//Rect g_rectBound;
//int g_maxIndex = 0;
//int g_maxArea = 0;
//const int MINCONTOURAREA = 500;
//
///// Function declarations
///*RealSense Functions*/
//int getRSParameters();																			// get general properties/modules of RS
//void initRS(PXCSenseManager*, Size colorSize, Size depthSize, int rate, bool enColor, bool enDepth);
///*Others*/
//void printMatProperties(Mat*, char*);												// printMat information, size, etc.
//Rect getRect(vector<vector<Point>> contourSet, int ROIBuffer, Size windowSize, bool isDepth);
//void overlayImage(Mat src, Mat dst, Point topLeftCorner);
///*RS to OCV conversion functions*/
//Mat colorPXCImage2Mat(PXCImage *inImg);
//void ConvertPXCImageToOpenCVMat(PXCImage*, Mat*);						// original-ish version
//void RSColorPXCImage2Mat(PXCImage*, Mat*);										// Only for Color camera
//Mat PXCImage2CVMat(PXCImage*, PXCImage::PixelFormat);				// original-ish version
//Mat PXCImage2CVMatAlt(PXCImage*, PXCImage::PixelFormat);		// mix of PXCI... and ConvertP...
//void ConvertDepthPXCImageToOpenCVMat(PXCImage*, Mat*);				// specifically for depth, not complete
//																															/*Mat color conversion funtions*/
//Mat depth2DepthColor(Mat*, int, int);												// to make rainbow version, not working
//Mat depth2DepthColor(Mat*);																	// to make rainbow version, not working
//																														/*Callback functions*/
//void initTBs();
//
///// Main function
////int wmain(int argc, WCHAR* argv[]) {
//int main(int argc, char** argv) {
//		///Main vars
//		/*Flags*/
//		bool firstLoop = true;
//		bool enableColor = true;
//		bool enableDepth = true;
//		bool enableColorSegmentation = true;
//		bool connectedToRS = false;							// Track whether main loop entered/connected		
//																						/*Mat properties*/
//		int frameRate = 60;
//		Size frameSizeC = Size(640, 480);
//		Size frameSizeD = Size(628, 468); //c: 628, r : 468 ?
//																			/*Trail vars*/
//		bool beginTrail = true;						// trail reset when starting or considered stale		
//		int trailStaleness = 0;						// increments when no ball found until MAXSTALE
//		const int MAXSTALE = 5;						// when g_trailstale reaches this value, trail is considered lost
//		const int TRAILSIZE = 6;
//		Point trail[TRAILSIZE] = { Point(0, 0) };
//		//int trailAge[TRAILSIZE] = { 10 };
//
//		/*Contour vars*/
//		vector<vector<Point>> contours; // Vector for storing contour(s)
//		vector<Vec4i> hierarchy;
//
//		/// Initializations
//		initTBs();
//		//getRSParameters();
//		initRS(g_sessionManager, frameSizeC, frameSizeD, frameRate, enableColor, enableDepth);
//		//initRS(g_smL, frameSizeC, frameRate);		
//
//		/*Timing Analysis vars*/
//		const int NUMHIST = 10;
//		double spanHist[NUMHIST] = { 0 };
//
//		/// Main loop
//		while (g_sessionManager->AcquireFrame(true) >= PXC_STATUS_NO_ERROR) {
//				/*Get system time before loop*/
//				double singleTick = (double)getTickCount();
//
//				/*RS has successfully connected*/
//				connectedToRS = true;
//
//				/// Aquire data
//				PXCCapture::Sample *sampleR;
//				sampleR = g_sessionManager->QuerySample();
//				/*Convert from PXCImage to Mat*/
//				PXCImage* colorImage = sampleR->color;
//				Mat frameColor;
//				if (colorImage && enableColor) {
//						//frameColor = PXCImage2CVMat(sampleR->color, PXCImage::PIXEL_FORMAT_RGB24);
//						//ConvertPXCImageToOpenCVMat(sampleR->color, &frameColor); //<- last used
//						RSColorPXCImage2Mat(sampleR->color, &frameColor); //only for colorimage		
//																															//frameColor = colorPXCImage2Mat(sampleR->color);
//				}
//				/*Convert from PXCImage to Mat, produce zoomed in depth Mat to match
//				proportions of RS RGB camera*/
//				PXCImage* depthImage = sampleR->depth;
//				Mat frameDepth;
//				Mat depthCrop;
//				if (depthImage && enableDepth) {
//						ConvertPXCImageToOpenCVMat(sampleR->depth, &frameDepth);
//						/*Make cropped depth image to get effective area*/
//						float borderSizeX = 0.03; // 0.1 takes 10% off all edges, 0.2 takes off 20%...
//						float borderSizeY = 0.03; // 0.1 takes 10% off all edges, 0.2 takes off 20%...
//						int newX = cvRound(frameDepth.cols * borderSizeX);
//						int newY = cvRound(frameDepth.rows * borderSizeY);
//						int newW = cvRound(frameDepth.cols * (1 - 2 * borderSizeX));
//						int newH = cvRound(frameDepth.rows * (1 - 2 * borderSizeY));
//						Rect crop(newX, newY, newW, newH);
//						//cout << "x: " << newX << ", y: " << newY << ", w: " << newW << ", h: " << newH << endl;
//						depthCrop = frameDepth(crop);
//						GaussianBlur(frameDepth,
//								frameDepth,
//								Size(3, 3),
//								0, 0,
//								BORDER_CONSTANT);
//				}
//
//				//imshow("color", frameColor);
//				//imshow("depth", frameDepth);
//
//				/// Manipulate data, Find tennis ball by its Hue
//
//				/*Strip the alpha channel first*/
//				cvtColor(frameColor,
//						frameColor,
//						CV_BGRA2BGR);
//
//				/*Convert to HSV*/
//				Mat frameHSV;
//				cvtColor(frameColor,
//						frameHSV,
//						CV_RGB2HSV);
//
//				/*Two sided threshold to find a particular Hue range
//				Convert taskbar 1 and 2 values to a center and range, ensure ranges*/
//				Mat hueMask;
//				int HBoundLo = tb1_HCenter - tb2_HRange;
//				int HBoundHi = tb1_HCenter + tb2_HRange;
//				if (HBoundLo < 0) HBoundLo = 0;
//				if (HBoundHi > 255) HBoundHi = 255;
//				Scalar lower(HBoundLo, tb3_SMin, tb5_VMin);
//				Scalar upper(HBoundHi, tb4_SMax, tb6_VMax);
//				inRange(frameHSV,
//						lower,
//						upper,
//						hueMask);
//
//				/*Open image tb7_numMorph times to get rid of some noise*/
//				Mat morphROI;
//				Mat kernel = Mat::ones(3, 3, CV_8U);
//				morphologyEx(hueMask,
//						morphROI,
//						MORPH_OPEN,
//						kernel,
//						Point(-1, -1),
//						tb7_numMorph,
//						BORDER_REFLECT);
//
//				/*Copy morphROI into grayscale, run smoothing on it*/
//				//cvtColor(morphROI, smoothedMorphROI, );
//				//Mat smoothedMorphROI(frameSizeC, CV_8UC1);
//				//morphROI.copyTo(smoothedMorphROI);				
//				//imshow("morphROI", morphROI);
//				//imshow("smoothedMorphROI 1", smoothedMorphROI);
//
//				///*Blur opened image*/
//				//for (int i = 1; i < tb8_numBlur; i++) {
//				//		blur(		smoothedMorphROI, 
//				//						smoothedMorphROI, 
//				//						Size(7, 7));
//				//		//medianBlur(smoothedMorphROI, smoothedMorphROI, 3);
//				//		//GaussianBlur(smoothedMorphROI, smoothedMorphROI, Size(3, 3), 0, 0, BORDER_CONSTANT);
//				//}
//
//				//imshow("smoothedMorphROI 2", smoothedMorphROI);
//
//				///*Canny for edges*/
//				//Mat cannyROI;
//				//Canny(	smoothedMorphROI, 
//				//				cannyROI, 
//				//				tb9_lowThresh, 
//				//				tb9_lowThresh * tb10_threshRatio, 
//				//				5);				
//
//				/*Contours*/
//				Scalar color(0, 0, 255); // red
//				findContours(morphROI,
//						contours,
//						hierarchy,
//						CV_RETR_CCOMP,
//						CV_CHAIN_APPROX_SIMPLE);
//
//				/*Find boundaries for ROIs*/
//				int ROIBuffC = 15;
//				int ROIBuffD = 15;
//				Rect testRectColorROI = getRect(contours,
//						ROIBuffC,
//						frameSizeC,
//						false);
//				Rect testRectDepthROI = getRect(contours,
//						ROIBuffD,
//						depthCrop.size(),
//						true);
//				Rect rectColorROI;
//				Rect rectDepthROI;
//				/*Update ROIs if new contour is large enough*/
//				if (g_maxArea > MINCONTOURAREA) {
//						rectColorROI = testRectColorROI;
//						rectDepthROI = testRectDepthROI;
//				}
//
//				/*Check if any contours exist, draw the max contour if they do*/
//				Mat trailOverlay = cv::Mat::zeros(frameSizeC, CV_8UC3);
//				/*{A_1} If there is a contour that is large enough*/
//				if (contours.size() && g_maxArea > MINCONTOURAREA) {
//						rectangle(trailOverlay, g_rectBound, Scalar(0, 255, 0), 1, 8, 0);
//						int radius = g_rectBound.width / 2;
//						Point center(g_rectBound.x + radius, g_rectBound.y + radius);
//						circle(trailOverlay,
//								center,
//								radius,
//								Scalar(255, 0, 0), 3, 8, 0);
//						/*Draw the largest contour using previously stored index*/
//						drawContours(trailOverlay,
//								contours,
//								g_maxIndex,
//								color, 4, 8,
//								hierarchy);
//						/*set all trail elements to new center if new trail, otherwise
//						only set head of trail*/
//						/* {B_1} When first trail, or lost object, set all segments to new found object position*/
//						if (beginTrail) {
//								for (int i = 0; i < TRAILSIZE; i++) {
//										trail[i] = center;
//								}
//						}
//						/* {B_2} Otherwise only set the head of the trail to new found object position*/
//						else {
//								trail[0] = center;
//						}
//						/*Cascade verticies down the array*/
//						for (int i = TRAILSIZE; i > 0; i--) {
//								trail[i] = trail[i - 1];
//						}
//						/*Contour found. Trail is not stale and it has already begun*/
//						trailStaleness = 0;
//						beginTrail = false;
//				}
//				/*{A_2} No large enough contours then increase staleness until trail needs to be reset*/
//				else {
//						trailStaleness++;
//						if (trailStaleness == MAXSTALE) {
//								beginTrail = true;
//								trailStaleness = 0;
//						}
//				}
//
//				/*Draw trail. Vary size and color based on position in trail*/
//				for (int i = TRAILSIZE; i > 0; i--) {
//						//trailAge[i]--;
//						//if (trailAge[i] < 0) trailAge[i] = 0;
//						//trail[i] = trail[i - 1];
//						int trailWidth = cvRound(16 - i * 15 / TRAILSIZE);
//						Scalar trailColor(252, 255 - i * 226 / TRAILSIZE, 28 + i * 119 / TRAILSIZE);
//						line(trailOverlay, trail[i - 1], trail[i], trailColor, trailWidth, 8, 0);
//				}
//
//				/*Combine overlay and main image*/
//				Mat output_1;
//				addWeighted(frameColor, 1.0, trailOverlay, 0.8, 0.0, output_1);
//
//				/*Find ROI around object in color image for more processing*/
//				Mat colorROI = frameColor(rectColorROI);
//
//				/*Find ROI around object in color image for more processing*/
//				//Mat depthROI = depthCrop(rectDepthROI) * tb11_depthMult;
//				//GaussianBlur(depthROI, depthROI, Size(3, 3), 0, 0, BORDER_CONSTANT);
//
//				/*Overlay ROI onto output window*/
//				overlayImage(colorROI,
//						output_1,
//						Point(0, 0));
//
//				///*Get ROI from center of ROI, find average value, call that depth, convert it to real world units*/
//				//float rawSizeX = 0.4; // 0.1 takes 10% off all edges, 0.2 takes off 20%...
//				//float rawSizeY = 0.4; // 0.1 takes 10% off all edges, 0.2 takes off 20%...
//				//int rawROIx = cvRound(depthROI.cols * rawSizeX);
//				//int rawROIy = cvRound(depthROI.rows * rawSizeY);
//				//int rawROIw = cvRound(depthROI.cols * (1 - 2 * rawSizeX));
//				//int rawROIh = cvRound(depthROI.rows * (1 - 2 * rawSizeY));
//				//Rect rawROI(rawROIx, rawROIy, rawROIw, rawROIh);
//				//Mat depthValueRaw = depthROI(rawROI);
//				//Scalar averageDepth = mean(depthValueRaw);
//				////cout << "averageDepth: " << averageDepth[0] << endl;
//				///*get closest/farthest value in depth image*/
//				////double min, max;
//				////minMaxLoc(depthROI, &min, &max);
//				//if (depthValueRaw.data) {
//				//		//namedWindow("raw", CV_WINDOW_NORMAL);
//				//		//imshow("raw", depthValueRaw);
//				//}
//
//				/*Further processing: threshold, canny, find circles or something*/
//
//				/// Display data
//				//imshow(DEPTHCROP_WIN, depthCrop * tb11_depthMult);
//				//imshow(HUE_WIN, hueMask);
//				//imshow(MORPH1_WIN, morphROI);
//				//imshow(BLUR_WIN, smoothedMorphROI);
//				//imshow(CANNY1_WIN, cannyROI);
//				imshow(COLOR_WIN, output_1);
//				if (colorROI.data) {
//						imshow(ROI1_WIN, colorROI);
//				}
//				//if (depthROI.data) {
//				//		//imshow(ROI2_WIN, depthROI);
//				//}
//
//
//				/*Find circle on colorROI*/
//				//Mat colorROICanny = smoothedMorphROI(rectColorROI);
//				//imshow("test", colorROICanny);
//
//
//
//				/// Print info
//				if (firstLoop) {
//						cout << "\n-------------------------------------\n";
//						if (enableColor) {
//								printMatProperties(&frameColor, "frameColor");
//						}
//						if (enableDepth) {
//								printMatProperties(&frameDepth, "frameDepth");
//						}
//						printMatProperties(&hueMask, "hueMask");
//						printMatProperties(&morphROI, "morph");
//						cout << "\n-------------------------------------\n";
//						firstLoop = false;
//				}
//
//				/// Timing analysis
//				/*Find duration since this pass of the loop*/
//				double singletickTime = ((double)getTickCount() - singleTick) / getTickFrequency();
//				/*Cascade durations back in array and set the head to the current loop duration*/
//				for (int i = NUMHIST; i > 0; i--) {
//						spanHist[i] = spanHist[i - 1];
//				}
//				spanHist[0] = singletickTime;
//				/*Find the average of the duration history array*/
//				double averageSpan = 0;
//				for (int i = 0; i < NUMHIST; i++) {
//						averageSpan += spanHist[i];
//				}
//				averageSpan = averageSpan / NUMHIST;
//				cout << "FPS : " << 1 / averageSpan << "T_inst: " << singletickTime
//						<< ", T_ave: " << averageSpan << endl;
//
//
//				/*Release frame to prepare for loop reentry*/
//				g_sessionManager->ReleaseFrame();
//				/*Check for exit request*/
//				int key = waitKey(1);
//				if (key == 27)
//						break;
//		}
//
//		/*Exit, clean up*/
//		if (connectedToRS) {
//				cout << "\n-------------------------------------\n";
//				cout << "Closing.";
//				cout << "\n-------------------------------------\n";
//		}
//		else {
//				cout << "\n-------------------------------------\n";
//				cout << "Could not connect to RealSense.\nClosing.";
//				cout << "\n-------------------------------------\n";
//		}
//		waitKey(0);
//		g_sessionManager->Release();
//		cv::destroyAllWindows();
//		return 0;
//}
//
///// Function Definitions
//Mat depth2DepthColor(Mat* depth) {
//		/// Create returnMat
//		Mat depthColorBGR(Size(depth->cols, depth->rows), CV_8UC3);
//		Mat depthColorHSV(Size(depth->cols, depth->rows), CV_8UC3);
//		Mat GR = Mat::ones(Size(depth->cols, depth->rows), CV_8UC1) * 120;
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
//Mat depth2DepthColor(Mat* depth, int min, int max) {
//		/// Create returnMat
//		Mat depthColor(Size(depth->cols, depth->rows), CV_8SC3);
//
//		int range = max - min;
//
//		cout << "range: " << range << endl;
//
//		return depthColor;
//}
///*This function is based on a function found here:
//http://stackoverflow.com/questions/32609341/convert-a-pxcimage-into-an-opencv-mat/32609342#32609342 */
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
///*This function is based on one found here:
//http://answers.opencv.org/question/52736/how-to-convert-pxcimage-into-mat/ */
//void RSColorPXCImage2Mat(PXCImage *inImg, Mat *outImg) {
//
//		PXCImage::ImageData data;
//
//		//inImg->AcquireAccess(PXCImage::ACCESS_READ, &data);
//		inImg->AcquireAccess(PXCImage::ACCESS_READ,
//				PXCImage::PIXEL_FORMAT_RGB24,
//				PXCImage::ROTATION_0_DEGREE,
//				PXCImage::OPTION_ANY,
//				&data);
//
//		PXCImage::ImageInfo imgInfo = inImg->QueryInfo();
//		int cvDataType = CV_8UC3;
//		int cvDataWidth = 3;
//
//		Mat testOut(imgInfo.width, imgInfo.height, cvDataType);
//
//		// suppose that no other planes
//		if (data.planes[1] != NULL) throw(0); // not implemented
//																					// suppose that no sub pixel padding needed
//		if (data.pitches[0] % cvDataWidth != 0) throw(0); // not implemented
//
//		outImg->create(imgInfo.height, data.pitches[0] / cvDataWidth, cvDataType);
//
//		int copySize = imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE);
//		//memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));
//		memcpy(outImg->data, data.planes[0], copySize);
//		memcpy(testOut.data, data.planes[0], copySize);
//
//		inImg->ReleaseAccess(&data);
//}
//
//cv::Mat colorPXCImage2Mat(PXCImage *inImg) {
//
//		PXCImage::ImageData data;
//
//		//inImg->AcquireAccess(PXCImage::ACCESS_READ, &data);
//		inImg->AcquireAccess(PXCImage::ACCESS_READ,
//				PXCImage::PIXEL_FORMAT_RGB24,
//				PXCImage::ROTATION_0_DEGREE,
//				PXCImage::OPTION_ANY,
//				&data);
//
//		PXCImage::ImageInfo imgInfo = inImg->QueryInfo();
//		int cvDataType = CV_8UC3;
//		int cvDataWidth = 3;
//
//		Mat output(imgInfo.width, imgInfo.height, cvDataType);
//
//		// suppose that no other planes
//		if (data.planes[1] != NULL) throw(0); // not implemented
//																					// suppose that no sub pixel padding needed
//		if (data.pitches[0] % cvDataWidth != 0) throw(0); // not implemented
//
//																											//outImg->create(imgInfo.height, data.pitches[0] / cvDataWidth, cvDataType);
//
//		int copySize = imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE);
//		//memcpy(outImg->data, data.planes[0], imgInfo.height*imgInfo.width*cvDataWidth * sizeof(pxcBYTE));
//		//memcpy(outImg->data, data.planes[0], copySize);
//		memcpy(output.data, data.planes[0], copySize);
//
//		inImg->ReleaseAccess(&data);
//
//		return output;
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
//void initRS(PXCSenseManager* senseMan, Size colorSize, Size depthSize, int rate, bool enColor, bool enDepth) {
//		/// Enable modules and streams
//		// Enable blob tracking
//		//sm->EnableBlob();
//		if (enColor) {
//				senseMan->EnableStream(PXCCapture::STREAM_TYPE_COLOR, colorSize.width, colorSize.height, rate);
//				//senseMan->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 0, 0, 0);
//		}
//		if (enDepth) {
//				//senseMan->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 0, 0, 0); 		// c: 628, r: 468?
//				senseMan->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, depthSize.width, depthSize.height, rate); 		// c: 628, r: 468?
//		}
//		// Initialization
//		senseMan->Init();
//		cout << "initRS:\n\tPXC status: " << senseMan->AcquireFrame(true) << endl;
//}
///*Prints some properties ofMat for debugging*/
//void printMatProperties(Mat* input, char* name) {
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
//void overlayImage(Mat src, Mat dst, Point topLeftCorner) {
//		/*Check that the images have the same number of channels */
//		int dstChan = dst.channels();
//		int srcChan = src.channels();
//		if (dstChan != srcChan) {
//				cout << "overlayImage\n\tError. Destination and source images must have the same number of channels.\n";
//				return;
//		}
//
//		/*Check that src fits into dst*/
//		if (src.rows > dst.rows || src.cols > dst.cols) {
//				cout << "overlayImage\n\tError. Source image larger than destination image.\n";
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
//		//		cout << "overlayImage Error. Not implemented for 2 channel images.\n";
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
//						/*For only 3 channel src and dstMats*/
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
//Rect getRect(vector<vector<Point>> contourSet, int ROIBuffer, Size windowSize, bool isDepth) {
//		int contSize = contourSet.size();
//		g_maxArea = 0;
//		g_maxIndex = 0;
//		Rect rectROI;
//
//		for (int i = 0; i < contSize; i++)			// iterate through each contour. 
//		{
//				double area = contourArea(contourSet[i], false);  //  Find the area of contour
//				if (area > g_maxArea) {
//						g_maxArea = area;								// Store the area of the largest contour
//						g_maxIndex = i;									//Store the index of largest contour
//				}
//		}
//		/*Return correct Rect if there are any contours. Return empty Rect if not*/
//		if (contSize) {
//				/// Find the bounding rectangle for biggest contour
//				g_rectBound = boundingRect(contourSet[g_maxIndex]);
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
//		/*Func vars*/
//		const char track1[] = "H Center";
//		const char track2[] = "H Range";
//		const char track3[] = "S Min";
//		const char track4[] = "S Max";
//		const char track5[] = "V Min";
//		const char track6[] = "V Max";
//		const char track7[] = "Openings";
//		const char track8[] = "Blurs";
//		const char track9[] = "Canny Th";
//		const char track10[] = "Canny Rat";
//		const char track11[] = "Depth";
//		/*Make windows*/
//		namedWindow(HUE_WIN, CV_WINDOW_NORMAL);
//		namedWindow(MORPH1_WIN, CV_WINDOW_NORMAL);
//		namedWindow(BLUR_WIN, CV_WINDOW_NORMAL);
//		namedWindow(CANNY1_WIN, CV_WINDOW_NORMAL);
//		namedWindow(COLOR_WIN, CV_WINDOW_NORMAL);
//		namedWindow(ROI1_WIN, CV_WINDOW_NORMAL);
//		namedWindow(ROI2_WIN, CV_WINDOW_NORMAL);
//		namedWindow(DEPTH_WIN, CV_WINDOW_NORMAL);
//		namedWindow(DEPTHCROP_WIN, CV_WINDOW_NORMAL);
//		/*Create Trackbars*/
//		createTrackbar(track1, HUE_WIN, &tb1_HCenter, 255, NULL);
//		createTrackbar(track2, HUE_WIN, &tb2_HRange, 255, NULL);
//		createTrackbar(track3, HUE_WIN, &tb3_SMin, 255, NULL);
//		createTrackbar(track4, HUE_WIN, &tb4_SMax, 255, NULL);
//		createTrackbar(track5, HUE_WIN, &tb5_VMin, 255, NULL);
//		createTrackbar(track6, HUE_WIN, &tb6_VMax, 255, NULL);
//		createTrackbar(track7, MORPH1_WIN, &tb7_numMorph, 20, NULL);
//		createTrackbar(track8, BLUR_WIN, &tb8_numBlur, 20, NULL);
//		createTrackbar(track9, CANNY1_WIN, &tb9_lowThresh, MAXLOWTHRESH, NULL);
//		createTrackbar(track10, CANNY1_WIN, &tb10_threshRatio, MAXTHRESHRATIO, NULL);
//		createTrackbar(track11, DEPTH_WIN, &tb11_depthMult, MAXDEPTHMULT, NULL);
//}
//
//
//////////////////////////////////NOT USED\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//
////void ConvertDepthPXCImageToOpenCVMat(PXCImage *inImg,Mat *outImg) {
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
///*Based on Sebastian Montabone's function published here:
//http://www.samontab.com/web/2016/04/interfacing-intel-realsense-f200-with-opencv/ */
//Mat PXCImage2CVMat(PXCImage *pxcImage, PXCImage::PixelFormat format)
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
//		Mat ocvImage = Mat(Size(width, height), type, data.planes[0]);
//
//		pxcImage->ReleaseAccess(&data);
//		return ocvImage;
//}
////Mat PXCImage2CVMatAlt(PXCImage *pxcImage, PXCImage::PixelFormat format)
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
////		Mat ocvImage;
////
////		ocvImage.create(height, data.pitches[0] / dataWidth, type);
////
////		memcpy(ocvImage.data, data.planes[0], height*width*dataWidth * sizeof(pxcBYTE));
////
////		pxcImage->ReleaseAccess(&data);
////		return ocvImage;
////}