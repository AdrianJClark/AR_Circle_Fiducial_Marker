#include "CaptureLibrary.h"

float readCode(IplImage *im1);
void calcTransform(CvMat *rotVector, CvMat *transVector, double *transMat[16]);
bool searchForCode(IplImage *marker, IplImage *code, CvPoint2D32f markerCorners[4], CvMat* homography, CvMat *rotMatInv, Capture *cameraMain, double **transMat, int offset = 0);

int barcodeLength = 10;
int barcode[] = {1,1,3,1,1,3,2,1,1,2};

IplImage *drawingFrame;

void renderBox(IplImage *image1, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4);

bool findMarker(IplImage* new_frame, Capture *cameraMain, double **transMat) {
	bool retVal = false;
	CvMemStorage *ms = cvCreateMemStorage(0);
	CvSeq *contours;

	IplImage *bw_frame = cvCreateImage(cvGetSize(new_frame), IPL_DEPTH_8U, 1);
	cvConvertImage(new_frame, bw_frame);
	//cvAdaptiveThreshold(bw_frame, bw_frame, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, 5);
	//cvDilate(bw_frame, bw_frame); cvErode(bw_frame, bw_frame);
	cvThreshold(bw_frame, bw_frame, 128, 255, CV_THRESH_BINARY);

	cvShowImage("bwframe", bw_frame);

	int contourCount = cvFindContours(bw_frame, ms, &contours);
	drawingFrame = cvCloneImage(new_frame);

	if (contours) {
		int i=0;
		for (CvSeq *c=contours; c!=NULL; c=c->h_next) {
			

			if (c->total >50) {
				i++;
				
				CvBox2D box = cvFitEllipse2(c);

				box.angle = -box.angle+90;

			CvPoint2D32f pts[4];
			cvBoxPoints(box, pts);

			cvCircle(drawingFrame, cvPoint(pts[0].x, pts[0].y), 2, cvScalar(255,0,0));
			cvLine(drawingFrame, cvPoint(pts[0].x, pts[0].y), cvPoint(pts[1].x, pts[1].y), cvScalarAll(255));
			cvCircle(drawingFrame, cvPoint(pts[1].x, pts[1].y), 2, cvScalar(0,255,0));
			cvLine(drawingFrame, cvPoint(pts[1].x, pts[1].y), cvPoint(pts[2].x, pts[2].y), cvScalarAll(255));
			cvCircle(drawingFrame, cvPoint(pts[2].x, pts[2].y), 2, cvScalar(0,0,255));
			cvLine(drawingFrame, cvPoint(pts[2].x, pts[2].y), cvPoint(pts[3].x, pts[3].y), cvScalarAll(255));
			cvCircle(drawingFrame, cvPoint(pts[3].x, pts[3].y), 2, cvScalar(255,0,255));
			cvLine(drawingFrame, cvPoint(pts[3].x, pts[3].y), cvPoint(pts[0].x, pts[0].y), cvScalarAll(255));


			IplImage *rotatedImage = cvCreateImage(cvGetSize(new_frame), IPL_DEPTH_8U, 3);
			CvMat* rotMat = cvCreateMat(2,3,CV_32FC1); CvMat* rotMatInv = cvCreateMat(2,3,CV_32FC1);
			cv2DRotationMatrix(box.center, -box.angle, 1.0, rotMat); cv2DRotationMatrix(box.center, box.angle, 1.0, rotMatInv);
			cvWarpAffine(new_frame, rotatedImage, rotMat);


			{
				CvMat *oldPoints = cvCreateMat(4,1,CV_32FC2);
				CvMat *newPoints = cvCreateMat(4,1,CV_32FC2);

				oldPoints->data.fl[0] = pts[0].x; oldPoints->data.fl[1] = pts[0].y;
				oldPoints->data.fl[2] = pts[1].x; oldPoints->data.fl[3] = pts[1].y;
				oldPoints->data.fl[4] = pts[2].x; oldPoints->data.fl[5] = pts[2].y;
				oldPoints->data.fl[6] = pts[3].x; oldPoints->data.fl[7] = pts[3].y;

				cvTransform(oldPoints, newPoints, rotMat);
				
				CvPoint2D32f newPts[4];
				newPts[0] = cvPoint2D32f(newPoints->data.fl[0], newPoints->data.fl[1]);
				newPts[1] = cvPoint2D32f(newPoints->data.fl[2], newPoints->data.fl[3]);
				newPts[2] = cvPoint2D32f(newPoints->data.fl[4], newPoints->data.fl[5]);
				newPts[3] = cvPoint2D32f(newPoints->data.fl[6], newPoints->data.fl[7]);

				cvReleaseMat(&oldPoints);
				cvReleaseMat(&newPoints);


				CvMat *pointsObj = cvCreateMat(4,3,CV_32FC1);
				CvMat *pointsIm = cvCreateMat(4,2,CV_32FC1);

				float height = (newPts[3].y + newPts[2].y)/2.0 - (newPts[1].y + newPts[0].y)/2.0;
				float width = (newPts[1].x + newPts[2].x)/2.0 - (newPts[0].x + newPts[3].x)/2.0;
				float maxLength = height>width?height/2.0:width/2.0;

				pointsObj->data.fl[0] = -maxLength; pointsObj->data.fl[1] = -maxLength; pointsObj->data.fl[2] = 0; 
				pointsObj->data.fl[3] = maxLength; pointsObj->data.fl[4] = -maxLength; pointsObj->data.fl[5] = 0; 
				pointsObj->data.fl[6] = maxLength; pointsObj->data.fl[7] = maxLength; pointsObj->data.fl[8] = 0; 
				pointsObj->data.fl[9] = -maxLength; pointsObj->data.fl[10] = maxLength; pointsObj->data.fl[11] = 0; 

				pointsIm->data.fl[0] = box.center.x-maxLength; pointsIm->data.fl[1] = box.center.y-maxLength;
				pointsIm->data.fl[2] = box.center.x+maxLength; pointsIm->data.fl[3] = box.center.y-maxLength;
				pointsIm->data.fl[4] = box.center.x+maxLength; pointsIm->data.fl[5] = box.center.y+maxLength;
				pointsIm->data.fl[6] = box.center.x-maxLength; pointsIm->data.fl[7] = box.center.y+maxLength;

				CvMat *rotVect= cvCreateMat(3,1,CV_32FC1);
				CvMat *transVect = cvCreateMat(3,1,CV_32FC1);

				cvFindExtrinsicCameraParams2(pointsObj, pointsIm, cameraMain->getCaptureParameters(), cameraMain->getCaptureDistortion(), rotVect, transVect);

				CvMat *rotMat2 = cvCreateMat(3,3,CV_32FC1);
				float theta = -acos(height/width);
				rotMat2->data.fl[0] = 1; rotMat2->data.fl[1] = 0; rotMat2->data.fl[2] = 0;
				rotMat2->data.fl[3] = 0; rotMat2->data.fl[4] = cos(theta); rotMat2->data.fl[5] = -sin(theta);
				rotMat2->data.fl[6] = 0; rotMat2->data.fl[7] = sin(theta); rotMat2->data.fl[8] = cos(theta);
				cvRodrigues2(rotMat2, rotVect);
				cvReleaseMat(&rotMat2);
				
				CvMat *pointsOut = cvCreateMat(4,2, CV_32FC1);
				cvProjectPoints2(pointsObj, rotVect, transVect, cameraMain->getCaptureParameters(), cameraMain->getCaptureDistortion(), pointsOut);

				float topMiddle = (newPts[1].x+newPts[0].x)*0.5;
				float bottomMiddle = (newPts[3].x+newPts[2].x)*0.5;
				float topWidth = abs(pointsOut->data.fl[2]-pointsOut->data.fl[0])*0.5;
				float bottomWidth = abs(pointsOut->data.fl[6]-pointsOut->data.fl[4])*0.5;

				cvReleaseMat(&pointsOut);

				/* UPRIGHT */
				CvPoint2D32f pts3[4];
				pts3[0] = cvPoint2D32f(topMiddle - topWidth, newPts[0].y);
				pts3[1] = cvPoint2D32f(topMiddle + topWidth,newPts[1].y);
				pts3[2] = cvPoint2D32f(bottomMiddle + bottomWidth,newPts[2].y);
				pts3[3] = cvPoint2D32f(bottomMiddle - bottomWidth, newPts[3].y);

				CvPoint2D32f pts2[4]; 
				double size = 180;
				pts2[0] = cvPoint2D32f(0,0);
				pts2[1] = cvPoint2D32f(size,0);
				pts2[2] = cvPoint2D32f(size,size);
				pts2[3] = cvPoint2D32f(0,size);

				CvMat *homography = cvCreateMat(3,3,CV_32FC1);
				cvGetPerspectiveTransform(pts3, pts2, homography);
				IplImage *circleIm = cvCreateImage(cvSize(size, size), IPL_DEPTH_8U, 3);
				cvWarpPerspective(rotatedImage, circleIm, homography);
				IplImage *barcode = cvCreateImage(cvGetSize(circleIm), IPL_DEPTH_8U, 3);
				float magnitude = (barcode->width/2.0) / log10(pow(circleIm->width*circleIm->width*.5 + circleIm->height*circleIm->height*.5,0.5));
				cvLogPolar(circleIm, barcode, cvPoint2D32f(circleIm->width/2.0, circleIm->height/2.0), magnitude, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);

				/* UPSIDE DOWN */
				CvPoint2D32f pts3a[4];
				pts3a[0] = cvPoint2D32f(bottomMiddle - bottomWidth, newPts[0].y);
				pts3a[1] = cvPoint2D32f(bottomMiddle + bottomWidth,newPts[1].y);
				pts3a[2] = cvPoint2D32f(topMiddle + topWidth,newPts[2].y);
				pts3a[3] = cvPoint2D32f(topMiddle - topWidth, newPts[3].y);

				CvPoint2D32f pts2a[4];
				pts2a[0] = cvPoint2D32f(size,size);
				pts2a[1] = cvPoint2D32f(0,size);
				pts2a[2] = cvPoint2D32f(0,0);
				pts2a[3] = cvPoint2D32f(size,0);

				CvMat *homography2 = cvCreateMat(3,3,CV_32FC1);
				cvGetPerspectiveTransform(pts3a, pts2a, homography2);
				IplImage *circleIm2 = cvCreateImage(cvSize(size, size), IPL_DEPTH_8U, 3);
				cvWarpPerspective(rotatedImage, circleIm2, homography2);
				IplImage *barcode2 = cvCreateImage(cvGetSize(circleIm2), IPL_DEPTH_8U, 3);
				cvLogPolar(circleIm2, barcode2, cvPoint2D32f(circleIm2->width/2.0, circleIm2->height/2.0), magnitude, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);

				if (searchForCode(circleIm, barcode, pts2, homography, rotMatInv, cameraMain, transMat)) {
					retVal = true;
				} else if (searchForCode(circleIm2, barcode2, pts2a, homography2, rotMatInv, cameraMain, transMat, 180)) {
					retVal = true;
				}
				

				cvReleaseMat(&homography);
				cvReleaseMat(&homography2);

				cvReleaseImage(&circleIm);	cvReleaseImage(&circleIm2);
				cvReleaseImage(&barcode); cvReleaseImage(&barcode2);
				cvReleaseMat(&pointsIm); cvReleaseMat(&pointsObj);
				cvReleaseMat(&rotVect); cvReleaseMat(&transVect);
			}
			cvReleaseImage(&rotatedImage);
			cvReleaseMat(&rotMat); cvReleaseMat(&rotMatInv);

			

			
			}
		}
	}

			
	cvShowImage("contours", drawingFrame);

	cvReleaseMemStorage(&ms);
	cvReleaseImage(&bw_frame); cvReleaseImage(&drawingFrame);
	return retVal;
}

bool searchForCode(IplImage *marker, IplImage *code, CvPoint2D32f markerCorners[4], CvMat* homography, CvMat *rotMatInv, Capture *cameraMain, double **transMat, int offset) {
	float rotVal = readCode(code);
	if (rotVal>-1) {
		cvShowImage("-1", code);
		CvMat *mat = cvCreateMat(2,3, CV_32FC1);
		cv2DRotationMatrix(cvPoint2D32f(marker->width/2.0, marker->height/2.0), -(90+rotVal)+offset, 1.0, mat);
		
		CvMat *pIn = cvCreateMat(4,1,CV_32FC2);
		CvMat *pOut = cvCreateMat(4,1,CV_32FC2);
		
		pIn->data.fl[0] = markerCorners[0].x; pIn->data.fl[1] = markerCorners[0].y;
		pIn->data.fl[2] = markerCorners[1].x; pIn->data.fl[3] = markerCorners[1].y;
		pIn->data.fl[4] = markerCorners[2].x; pIn->data.fl[5] = markerCorners[2].y;
		pIn->data.fl[6] = markerCorners[3].x; pIn->data.fl[7] = markerCorners[3].y;

		cvTransform(pIn, pOut, mat);
		cvReleaseMat(&mat);

		CvMat* homographyInv = cvCreateMat(3,3,CV_32FC1);
		cvInvert(homography, homographyInv);
		cvPerspectiveTransform(pOut, pIn, homographyInv);
		cvTransform(pIn, pOut, rotMatInv);
		cvReleaseMat(&homographyInv);
		renderBox(drawingFrame, pOut->data.fl[0], pOut->data.fl[1], pOut->data.fl[2], pOut->data.fl[3], pOut->data.fl[4], pOut->data.fl[5], pOut->data.fl[6], pOut->data.fl[7]);


		CvMat *pointsObj = cvCreateMat(4,1,CV_32FC3);
		pointsObj->data.fl[0] = 0; pointsObj->data.fl[1] = 0; pointsObj->data.fl[2] = 0; 
		pointsObj->data.fl[3] = 100; pointsObj->data.fl[4] = 0; pointsObj->data.fl[5] = 0; 
		pointsObj->data.fl[6] = 100; pointsObj->data.fl[7] = 100; pointsObj->data.fl[8] = 0; 
		pointsObj->data.fl[9] = 0; pointsObj->data.fl[10] = 100; pointsObj->data.fl[11] = 0; 

		CvMat* rotVector= cvCreateMat(1, 3, CV_32F); CvMat*transVector= cvCreateMat(1, 3, CV_32F);

		cvFindExtrinsicCameraParams2(pointsObj, pOut, cameraMain->getCaptureParameters(), cameraMain->getCaptureDistortion(), rotVector, transVector);

		calcTransform(rotVector, transVector, transMat);
		cvReleaseMat(&pointsObj);
		cvReleaseMat(&pIn); cvReleaseMat(&pOut);
		cvReleaseMat(&rotVector); cvReleaseMat(&transVector);
		return true;
	}
	return false;
}


float readCode(IplImage *im) {
	float retVal = -1;
	IplImage *imBW = cvCreateImage(cvGetSize(im), IPL_DEPTH_8U, 1);
	cvConvertImage(im, imBW);
	cvThreshold(imBW, imBW, 128, 255, CV_THRESH_BINARY);

	CvMat *imRow = cvCreateMat(1, imBW->width, CV_8U);

	std::vector<int> pixelCounts;
	std::vector<int> pixelHeights;

	cvGetRow(imBW, imRow, 0);
	int pDist = cvCountNonZero(imRow);
	
	int pRun=1;
	for (int i=1; i<imBW->height; i++) {
		cvGetRow(imBW, imRow, i);
		int curRow = cvCountNonZero(imRow);
		if (abs(pDist-curRow)>5) {
			pixelCounts.push_back(pRun); pixelHeights.push_back(pDist); 
			pDist = curRow;	pRun=1;
		} else {
			pRun++;
		}
		pDist = (pDist + curRow) *0.5;
	}
	pixelCounts.push_back(pRun);pixelHeights.push_back(pDist);


	int offset=0;
	if (abs(pixelHeights.at(0)-pixelHeights.at(pixelHeights.size()-1))<5) {
		offset = pixelCounts.at(0);
		pixelCounts.at(0) += pixelCounts.at(pixelHeights.size()-1);
		pixelCounts.pop_back(); pixelHeights.pop_back();
	}

	std::vector<int> barcodes;
	for (int i=0; i<pixelCounts.size(); i++) {
		barcodes.push_back (cvRound((double)pixelCounts.at(i)/11.25));
	}

	
	if (barcodes.size() == barcodeLength) {
		
		for (int i=0; i<pixelCounts.size(); i++) {
			
			for (int j=0; j<barcodeLength; j++) {
				if (barcodes.at((j+i)%barcodeLength) != barcode[j]) break;
				if (j==barcodeLength-1) {
					//We found it!
					int rotValue;
					if (i==0) {
						rotValue = 180-(pixelCounts.at(0)-offset);
					} else {
						rotValue =offset;
						for (int j=1; j<i; j++) {
							rotValue+=pixelCounts.at(j);
						}
					}
					retVal = rotValue;
				}
			}
		}


	}

	if (retVal!=-1) retVal= (retVal/180)*360;
	cvReleaseImage(&imBW); cvReleaseMat(&imRow);
	return retVal;
}

	void calcTransform(CvMat *rotVector, CvMat *transVector, double *transMat[16]) {
		*transMat = (double*)malloc(16*sizeof(double));

		CvMat *rotMat = cvCreateMat(3,3, CV_32FC1);
		
		//cvRodrigues converts from a rotational vector to a rotational matrix
		cvRodrigues2(rotVector, rotMat);

		//Reflection matrix
		CvMat *cvReflect = cvCreateMat(4,4,CV_64FC1);
		double reflect[16] = 
		{1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 1 }; 
		cvSetData(cvReflect, reflect, 4*sizeof(double));

		CvMat *cvTransMat = cvCreateMat(4,4, CV_64FC1);
 		cvTransMat->data.db[0] = rotMat->data.fl[0]; cvTransMat->data.db[1] = rotMat->data.fl[1]; cvTransMat->data.db[2] = rotMat->data.fl[2];  cvTransMat->data.db[3] = transVector->data.fl[0];
		cvTransMat->data.db[4] = rotMat->data.fl[3]; cvTransMat->data.db[5] = rotMat->data.fl[4]; cvTransMat->data.db[6] = rotMat->data.fl[5];  cvTransMat->data.db[7] = transVector->data.fl[1];
		cvTransMat->data.db[8] = rotMat->data.fl[6]; cvTransMat->data.db[9] = rotMat->data.fl[7]; cvTransMat->data.db[10] = rotMat->data.fl[8]; cvTransMat->data.db[11] = transVector->data.fl[2];
		cvTransMat->data.db[12] = 0;				 cvTransMat->data.db[13] = 0;				  cvTransMat->data.db[14] = 0;					cvTransMat->data.db[15] = 1;

		CvMat *cvOutput = cvCreateMat(4,4,CV_64FC1);
		cvTranspose(cvTransMat, cvOutput);
		
//		cvCopy(cvOutput, cvTransMat);
		cvMatMul(cvOutput, cvReflect, cvTransMat);

		cvReleaseMat(&cvOutput);
		cvReleaseMat(&cvReflect);

		for (int i=0; i<16; i++) (*transMat)[i] = cvTransMat->data.db[i];
		cvReleaseMat(&cvTransMat);
		cvReleaseMat(&rotMat);
	}


	void renderBox(IplImage *image1, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {

		CvPoint p1 = cvPoint(x1,y1);
		CvPoint p2 = cvPoint(x2,y2);
		CvPoint p3 = cvPoint(x3,y3);
		CvPoint p4 = cvPoint(x4,y4);

		cvCircle(image1, p1, 3, cvScalar(255,0,0,0));
		cvCircle(image1, p2, 3, cvScalar(0,255,0,0));
		cvCircle(image1, p3, 3, cvScalar(0,0,255,0));
		cvCircle(image1, p4, 3, cvScalar(255,0,255,0));

		cvLine(image1, p1, p2, cvScalarAll(128));
		cvLine(image1, p2, p3, cvScalarAll(128));
		cvLine(image1, p3, p4, cvScalarAll(128));
		cvLine(image1, p4, p1, cvScalarAll(128));

		CvPoint p5 = cvPoint((p1.x + p2.x)/2.0, (p1.y+p2.y)/2.0);
		CvPoint p6 = cvPoint((p3.x + p4.x)/2.0, (p3.y+p4.y)/2.0);

		cvLine(image1, p5, p6, cvScalarAll(128));
		cvCircle(image1, p5, 3, cvScalarAll(128));

	}
