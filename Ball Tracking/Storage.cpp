//////LOOP STYLE 2 WITH NUMBER OF PERMITTED ERRORS
// const in MAX_ERRORS = 100;
// int errors = 0;
//while (errors < MAX_ERRORS) {

//		if (sm->AcquireFrame(true) >= PXC_STATUS_NO_ERROR) {
//				PXCCapture::Sample *sample;
//				sample = sm->QuerySample();

//				if (sample->color)
//						frameColor = PXCImage2CVMat(sample->color, PXCImage::PIXEL_FORMAT_RGB24);

//				if (sample->depth)
//						PXCImage2CVMat(sample->depth, PXCImage::PIXEL_FORMAT_DEPTH_F32).convertTo(frameDepth, CV_8UC1);

//				cv::imshow("Color", frameColor);
//				cv::imshow("Depth", frameDepth);
//				int key = cv::waitKey(1);

//				if (key == 27)
//						break;

//				// Resume next frame processing
//				sm->ReleaseFrame();
//		}
//		else {
//				errors++;
//		}
//}	