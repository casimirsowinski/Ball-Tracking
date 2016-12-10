//#include <pxcsensemanager.h>
//#include <iostream>
//#include <opencv2/opencv.hpp>
//
////cv::Mat frameIR;
//cv::Mat frameColor;
//cv::Mat frameDepth;
//cv::Mutex framesMutex;
//
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
//		int type;
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
//
//
//class FramesHandler :public PXCSenseManager::Handler
//{
//public:
//		virtual pxcStatus PXCAPI OnNewSample(pxcUID, PXCCapture::Sample *sample)
//		{
//				framesMutex.lock();
//				//frameIR = PXCImage2CVMat(sample->ir, PXCImage::PIXEL_FORMAT_Y8);
//				frameColor = PXCImage2CVMat(sample->color, PXCImage::PIXEL_FORMAT_RGB24);
//				PXCImage2CVMat(sample->depth, PXCImage::PIXEL_FORMAT_DEPTH_F32).convertTo(frameDepth, CV_8UC1);
//				framesMutex.unlock();
//				return PXC_STATUS_NO_ERROR;
//		}
//};
//
//
//int main(int argc, char* argv[])
//{
//		cv::Size frameSize = cv::Size(640, 480);
//		float frameRate = 60;
//
//		//cv::namedWindow("IR", cv::WINDOW_NORMAL);
//		cv::namedWindow("Color", cv::WINDOW_NORMAL);
//		cv::namedWindow("Depth", cv::WINDOW_NORMAL);
//		//frameIR = cv::Mat::zeros(frameSize, CV_8UC1);
//		frameColor = cv::Mat::zeros(frameSize, CV_8UC3);
//		frameDepth = cv::Mat::zeros(frameSize, CV_8UC1);
//
//		PXCSenseManager *pxcSenseManager = PXCSenseManager::CreateInstance();
//
//		//Enable the streams to be used
//		PXCVideoModule::DataDesc ddesc = {};
//		ddesc.deviceInfo.streams = PXCCapture::STREAM_TYPE_IR | PXCCapture::STREAM_TYPE_COLOR | PXCCapture::STREAM_TYPE_DEPTH;
//
//		pxcSenseManager->EnableStreams(&ddesc);
//
//		FramesHandler handler;
//		pxcSenseManager->Init(&handler);
//		pxcSenseManager->StreamFrames(false);
//
//		//Local images for display
//		//cv::Mat displayIR = frameIR.clone();
//		cv::Mat displayColor = frameColor.clone();
//		cv::Mat displayDepth = frameDepth.clone();
//
//		bool keepRunning = true;
//		while (keepRunning)
//		{
//				framesMutex.lock();
//				//displayIR = frameIR.clone();
//				displayColor = frameColor.clone();
//				displayDepth = frameDepth.clone();
//				framesMutex.unlock();
//
//				//cv::imshow("IR", displayIR);
//				cv::imshow("Color", displayColor);
//				cv::imshow("Depth", displayDepth);
//
//				int key = cv::waitKey(1);
//				if (key == 27)
//						keepRunning = false;
//		}
//		//Stop the frame acqusition thread
//		pxcSenseManager->Close();
//
//		pxcSenseManager->Release();
//
//
//		return 0;
//}