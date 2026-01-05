#include "CaptureLibrary.h"
#include <iostream>

class Video: public Capture {
public:
	Video(char *videoFile) {
		captureObj = cvCreateFileCapture(videoFile);
		if (!captureObj) {std::cerr << "Unable to load video file: " << videoFile << std::endl; exit(0); }

		captureWidth = (int)cvGetCaptureProperty(captureObj, CV_CAP_PROP_FRAME_WIDTH); 
		captureHeight = (int)cvGetCaptureProperty(captureObj, CV_CAP_PROP_FRAME_HEIGHT);	
	}

	Video(char *videoFile, char* parametersFile) {
		captureObj = cvCreateFileCapture(videoFile);
		if (!captureObj) { std::cerr << "Unable to load video file: " << videoFile << std::endl; exit(0); }

		if (!loadCaptureParams(parametersFile)) { std::cerr << "Unable to camera parameter file: " << parametersFile << std::endl; cvReleaseCapture(&captureObj); exit(0); }

		if (captureWidth==-1) captureWidth = (int)cvGetCaptureProperty(captureObj, CV_CAP_PROP_FRAME_WIDTH); 
		if (captureHeight==-1) captureHeight = (int)cvGetCaptureProperty(captureObj, CV_CAP_PROP_FRAME_HEIGHT);	
		setUndistort(true);
	}

	~Video() { if (captureObj) cvReleaseCapture( &captureObj ); }
	
	IplImage* getFrame() { 	
		IplImage *newFrame;

		//Make sure we have a capture object
		if (captureObj) {
			if( !cvGrabFrame( captureObj )) return 0;
		}
		//Make a copy of the image
		newFrame = cvCloneImage(cvRetrieveFrame( captureObj ));

		if (captureUndistort && captureParams!=0) {
			//If we are undistorting, create a new Frame, undistort into it, then release the original
			//Frame and set it's pointer to the undistortedFrame
			IplImage *unDistortedFrame = cvCreateImage(cvGetSize(newFrame), newFrame->depth, newFrame->nChannels);
			cvRemap( newFrame, unDistortedFrame, mDistortX, mDistortY);
			//cvUndistort2( newFrame, unDistortedFrame, captureParams, captureDistortion );
			cvReleaseImage(&newFrame); newFrame = unDistortedFrame;
		}

		if( newFrame->origin == IPL_ORIGIN_BL ) {
			//If the frame is upside down, flip it the right way up
			IplImage *flippedFrame = cvCreateImage( cvGetSize(newFrame), newFrame->depth, newFrame->nChannels);
			cvFlip( newFrame, flippedFrame, 0 );
			cvReleaseImage(&newFrame); newFrame = flippedFrame;
		}

		//Return the new frame
		return newFrame;
	}

private:
	CvCapture *captureObj;
};