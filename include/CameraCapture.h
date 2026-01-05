#include "CaptureLibrary.h"

class Camera: public Capture {
public:
	Camera() {init(0, cvSize(320,240), 0);}
	Camera(char* parametersFile) {init(0, cvSize(320,240), parametersFile);}
	Camera(int cameraIndex) {init(cameraIndex, cvSize(320,240), 0);};
	Camera(int cameraIndex, char* parametersFile) {init(cameraIndex, cvSize(320,240), parametersFile);};
	Camera(int cameraIndex, CvSize imgSize, char* parametersFile) {init(cameraIndex, imgSize, parametersFile);};
	~Camera() { delete vi; }

	IplImage* getFrame()	{
		IplImage* newFrame = cvCreateImage(cvSize(captureWidth, captureHeight), IPL_DEPTH_8U, 3);
		if (vi) newFrame->imageData = (char*)vi->getPixels(camIndex, false, true);
		
		if (captureUndistort) {
			IplImage *unDistortedFrame = cvCreateImage(cvGetSize(newFrame), newFrame->depth, newFrame->nChannels);
			cvRemap( newFrame, unDistortedFrame, mDistortX, mDistortY);
			cvReleaseImage(&newFrame); newFrame = unDistortedFrame;
		}

		return newFrame;
	}

private:
	void init(int cameraIndex, CvSize imgSize, char* parametersFile) {
		vi = new videoInput();
		vi->setVerbose(false); vi->setUseCallback(true);
		vi->setupDevice(cameraIndex, imgSize.width, imgSize.height);

		if (parametersFile) { loadCaptureParams(parametersFile); setUndistort(true); }

		captureWidth = vi->getWidth(cameraIndex); captureHeight = vi->getHeight(cameraIndex);
		camIndex = cameraIndex;
	}
	videoInput *vi;
	int camIndex;
};