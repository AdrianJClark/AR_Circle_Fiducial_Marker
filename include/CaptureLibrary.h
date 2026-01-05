#ifndef CAPTURELIBRARY_H
#define CAPTURELIBRARY_H

#include <cv.h>
#include <highgui.h>
#include "videoInput.h"

class Capture {
public:
	Capture() { captureParams=captureDistortion=mDistortX=mDistortY=0; captureUndistort=false;}
	virtual ~Capture() {
		if (captureParams) cvReleaseMat(&captureParams);
		if (captureDistortion) cvReleaseMat(&captureDistortion);
		if (mDistortX) cvReleaseMat(&mDistortX);
		if (mDistortY) cvReleaseMat(&mDistortY);
	}

	virtual IplImage* getFrame() { return 0;}
	CvMat* getCaptureParameters() { return captureParams; }
	CvMat* getCaptureDistortion() { if (captureUndistort) return 0; else return captureDistortion; }

	int getWidth() { return captureWidth; }
	int getHeight() { return captureHeight; }
	
	bool getUndistort() { return captureUndistort; }
	void setUndistort(bool undistort) { 
		captureUndistort = undistort; 
		if (captureUndistort) { 
			captureParams->data.db[2] = captureWidth/2.0; captureParams->data.db[5] = captureHeight/2.0; 
		} else {
			captureParams->data.db[2] = principalX; captureParams->data.db[5] = principalY; 
		}
	}

protected:
	CvMat* captureParams, *captureDistortion;
	CvMat* mDistortX, *mDistortY;
	int captureWidth, captureHeight;
	bool captureUndistort;

	bool loadCaptureParams(char *filename) {
		CvFileStorage* fs = cvOpenFileStorage( filename, 0, CV_STORAGE_READ );
		if (fs==0) return false; 

		CvFileNode* fileparams;
		//Read the Image Width
		fileparams = cvGetFileNodeByName( fs, NULL, "image_width" );
		captureWidth = cvReadInt(fileparams,-1);
		//Read the Image Height
		fileparams = cvGetFileNodeByName( fs, NULL, "image_height" );
		captureHeight = cvReadInt(fileparams,-1);
		//Read the Camera Parameters
		fileparams = cvGetFileNodeByName( fs, NULL, "camera_matrix" );
		captureParams = (CvMat*)cvRead( fs, fileparams );
		principalX = (int)captureParams->data.db[2]; principalY = (int)captureParams->data.db[5];
		//Read the Camera Distortion 
		fileparams = cvGetFileNodeByName( fs, NULL, "distortion_coefficients" );
		captureDistortion = (CvMat*)cvRead( fs, fileparams );
		cvReleaseFileStorage( &fs );

		//Initialize Undistortion Maps
		mDistortX = cvCreateMat(captureHeight, captureWidth, CV_32F);
		mDistortY = cvCreateMat(captureHeight, captureWidth, CV_32F);
		cvInitUndistortMap(captureParams, captureDistortion, mDistortX, mDistortY);

		return true;
	}
private:
	int principalX, principalY;
};

#endif
