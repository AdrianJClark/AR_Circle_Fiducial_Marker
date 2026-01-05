#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#include <GL/glut.h>
#include <windows.h>

#include "CameraCapture.h"
#include "circleMarker.h"

#include "highgui.h"

GLuint GLTextureID;
IplImage *GLImage;

using namespace std;

void initGLTextures();
void draw(IplImage* frame_input, Capture *camera, double *translationMat);
double* calcProjection(CvMat* captureParams, CvMat* captureDistortion, CvSize imgSize);

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

bool doloop = true;

void main(int argc, char **argv) {
	_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	//_CrtSetBreakAlloc(576);

	//Initialise Glut so we have an OpenGL context to render to
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("Metric Detector");

	//Initialise our OpenGL Textures
	initGLTextures();
	
	//Initialise our Camera
	Capture* camera = new Camera("camera.yml");

	while (doloop) {
		//Grab a frame 
		IplImage *new_frame = camera->getFrame();

		if (new_frame==0) break;

		//Perform registration
		double *transMat;
		
		//draw the scene
		if (findMarker(new_frame, camera, &transMat)) {
			draw(new_frame, camera, transMat);
		}

		//Check for the escape key and give the computer some processing time
		if (cvWaitKey(1)==27) doloop = false;
	
		//Clean up
		cvReleaseImage(&new_frame);

	//	for (unsigned int i=0; i<mt.size(); i++) { mt.at(i).clear(); } mt.clear();

	};

	delete camera;

}


void draw(IplImage* frame_input, Capture *camera, double *translationMat)
{
	//Clear the depth buffer 
	glClearDepth( 1.0 ); glClear(GL_DEPTH_BUFFER_BIT); glDepthFunc(GL_LEQUAL);

	//Set the viewport to the window size
	glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
    //Set the Projection Matrix to an ortho slightly larger than the window
	glMatrixMode(GL_PROJECTION); glLoadIdentity();
	glOrtho(-0.5, WINDOW_WIDTH-0.5, WINDOW_HEIGHT-0.5, -0.5, 1.0, -1.0);
    //Set the modelview to the identity
	glMatrixMode(GL_MODELVIEW); glLoadIdentity();

	//Turn off Light and enable a texture
	glDisable(GL_LIGHTING);	glEnable(GL_TEXTURE_2D); glDisable(GL_DEPTH_TEST);

	glBindTexture(GL_TEXTURE_2D, GLTextureID);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, frame_input->width, frame_input->height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, frame_input->imageData);
	
	//Draw the background
	glPushMatrix();
        glColor3f(255, 255, 255);
        glBegin(GL_TRIANGLE_STRIP);
            glTexCoord2f(0.0, 0.0);	glVertex2f(0.0, 0.0);
            glTexCoord2f(1.0, 0.0);	glVertex2f(WINDOW_WIDTH, 0.0);
            glTexCoord2f(0.0, 1.0);	glVertex2f(0.0, WINDOW_HEIGHT);
            glTexCoord2f(1.0, 1.0);	glVertex2f(WINDOW_WIDTH, WINDOW_HEIGHT);
        glEnd();
	glPopMatrix();

	//Turn off Texturing
	glBindTexture(GL_TEXTURE_2D, 0);
	glEnable(GL_LIGHTING);	glDisable(GL_TEXTURE_2D); glEnable(GL_DEPTH_TEST);

	//Loop through all the markers found
		double* projectionMat = calcProjection(camera->getCaptureParameters(), camera->getCaptureDistortion(), cvSize(320,240));// mt.at(i).projMat;
		CvSize markerSize = cvSize(100,100);

		//Set the Viewport Matrix
		glViewport(0,0,WINDOW_WIDTH,WINDOW_HEIGHT);

		//Load the Projection Matrix
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixd( projectionMat );

		//Load the camera modelview matrix 
		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixd( translationMat );

				printf("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n\n", 
			translationMat[0], translationMat[1], translationMat[2], translationMat[3], 
			translationMat[4], translationMat[5], translationMat[6], translationMat[7], 
			translationMat[8], translationMat[9], translationMat[10], translationMat[11], 
			translationMat[12], translationMat[13], translationMat[14], translationMat[15]);

		//Draw the boundry rectangle
		glColor3f(0,0,0);
		glBegin(GL_LINE_LOOP);
			glVertex3d(0,					0,0);
			glVertex3d(markerSize.width,	0,0);
			glVertex3d(markerSize.width,	markerSize.height,0);
			glVertex3d(0,					markerSize.height,0);
		glEnd();

		//Draw the infamous Teapot
		glEnable(GL_LIGHTING);
			glTranslatef(markerSize.width/2.0, markerSize.height/2.0,-25);
			glRotatef(-90, 1.0,0,0.0);
			glColor3f(1,0,0);
			glutSolidTeapot(50.0);
		glDisable(GL_LIGHTING);
	
	
	//Copy the OpenGL Graphics context into an IPLImage
	IplImage* outImage = cvCreateImage(cvSize(WINDOW_WIDTH,WINDOW_HEIGHT), IPL_DEPTH_8U, 3);
	glReadPixels(0,0,WINDOW_WIDTH,WINDOW_HEIGHT,GL_RGB, GL_UNSIGNED_BYTE, outImage->imageData);
	cvCvtColor( outImage, outImage, CV_BGR2RGB );
	cvFlip(outImage, outImage);

	cvNamedWindow("Registered Image"), cvShowImage("Registered Image", outImage);
	cvReleaseImage(&outImage);
}


void initGLTextures() {
	//Set up Materials 
	GLfloat mat_specular[] = { 0.4, 0.4, 0.4, 1.0 };
	GLfloat mat_diffuse[] = { .8,.8,.8, 1.0 };
	GLfloat mat_ambient[] = { .4,.4,.4, 1.0 };

	glShadeModel(GL_SMOOTH);//smooth shading
	glMatrixMode(GL_MODELVIEW);
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT, GL_SHININESS, 100.0);//define the material
	glColorMaterial(GL_FRONT, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);//enable the material
	glEnable(GL_NORMALIZE);

	//Set up Lights
	GLfloat light0_ambient[] = {0.1, 0.1, 0.1, 0.0};
	float light0_diffuse[] = { 0.8f, 0.8f, 0.8, 1.0f };
	glLightfv (GL_LIGHT0, GL_AMBIENT, light0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
    glEnable(GL_LIGHT0);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_NORMALIZE);
	glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);	
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glEnable (GL_LINE_SMOOTH);	

	//Initialise the OpenCV Image for GLRendering
	glGenTextures(1, &GLTextureID); 	// Create a Texture object
    glBindTexture(GL_TEXTURE_2D, GLTextureID);  //Use this texture
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);	
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);	
	glBindTexture(GL_TEXTURE_2D, 0);

}
	/* Calculate the OpenGL Projection Matrix */
	double* calcProjection(CvMat* captureParams, CvMat* captureDistortion, CvSize imgSize) {
	
		double *projMat=(double*)malloc(sizeof(double)*16);

		//Get camera parameteres
		double fovy = 2 * atan(imgSize.height/(2*captureParams->data.db[4])) * 57.295779513082320876798154814105;
		double aspect = ((double)imgSize.width/(double)imgSize.height) * ((double)captureParams->data.db[0] / (double)captureParams->data.db[4]) ;
		double projNear = 0.01 ; double projFar =  100000.0;

		//Set Frustum Co-ordinates
		double top = projNear*tan(fovy * 0.0087266462599716478846184538424431);
		double bottom = -top;
		double left = bottom*aspect;
		double right = top*aspect;

		//Fill in Matrix
		projMat[0] = (2*projNear)/(right-left);
		projMat[2] = (right+left)/(right-left);
		projMat[5] = (2*projNear)/(top-bottom);
		projMat[9] = (top+bottom)/(top-bottom);
		projMat[10] = -(projFar+projNear)/(projFar-projNear);
		projMat[11] = -1;
		projMat[14] = -(2*projFar*projNear)/(projFar-projNear);
	
		return projMat;
	}; 
