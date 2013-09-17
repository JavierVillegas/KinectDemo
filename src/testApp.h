#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    void UpdateGeometry(void);
	void UpdateGeometryLow(void);
	ofxKinect kinect;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	
	bool bDrawPointCloud;
	
	int Threshold;
	
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
    
    
    //  for geometry manipulation
    ofCamera TheCam;
    ofFbo Frame;
    ofVbo AllVertex;
    ofShader TheShader;
    ofImage TextuKin1;
    
    
};
