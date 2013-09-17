#include "testApp.h"

ofVec2f ControP;
cv::Mat ThreshImage;

const int RxF = 640;
const int RyF = 480;
ofVec3f vertKinet[RxF*RyF];
ofFloatColor ColorKinet[RxF*RyF];
unsigned int IndKinet[RxF*2*(RyF-1)+2*(RyF-2)];
bool DokinnectShader=false;
unsigned short depthval[RxF*RyF];
unsigned char depthChar[RxF*RyF];
int mode =0;
float Thetaxus =0;
float ThresTemp=1400;
float TheR=1000;
float ThePhI = 0;
int HayZebra =0;

//--------------------------------------------------------------
void testApp::setup() {
	//ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
//	if(kinect.isConnected()) {
//		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
//		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
//		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
//		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
//	}
	

	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);

	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
    
    ControP = ofVec2f(0,0);
    Threshold =128;
    ThreshImage.create(kinect.height, kinect.width, CV_8UC1);
    
    // Geometry related
    
    Frame.allocate(kinect.width, kinect.height);
    Frame.begin();
        ofClear(255, 255, 255);
    Frame.end();
 
    TextuKin1.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR_ALPHA);
    
    TheShader.load("shaders/Kinectshader1.vert","shaders/Kinectshader1.frag");
    
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
        if(mode==2){
        // coping the depth data to the vertexes
               memcpy(depthval, kinect.getRawDepthPixels(), RxF*RyF*sizeof(unsigned short));
            // copy values to the vertex array
            UpdateGeometry();            
        }
        
        
        
        
        
        
        
        // thresholding
        cv::Mat CVDepth;
        CVDepth = grayImage.getCvImage();
        
        cv::threshold(CVDepth, ThreshImage, Threshold, 255, CV_THRESH_BINARY);
        cv::Moments mu;
        mu = cv::moments(ThreshImage,true);
        ControP.x = mu.m10/mu.m00;
        ControP.y = mu.m01/mu.m00;
      
        //cv::blur(CVDepth, CVDepth, cv::Size(17,17));
        
        if(mode==3){
            // coping the depth data to the vertexes
            memcpy(depthChar, CVDepth.data, RxF*RyF*sizeof(unsigned char));
            // copy values to the vertex array
            UpdateGeometryLow();
        }
        
		// update the cv images
		grayImage.flagImageChanged();
	}
}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(mode==1) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	}
    if(mode ==0)
    {
		// draw from the live kinect
		kinect.drawDepth(0, 0);
		kinect.draw(kinect.width, 0);
        ofSetColor(255, 0, 0);
        ofCircle(kinect.width+ControP.x, ControP.y, 10);
        ofxCvGrayscaleImage tempogray;
        tempogray.allocate(ThreshImage.cols, ThreshImage.rows);
        tempogray =  ThreshImage.data;
        tempogray.draw(0, kinect.height);
        
	}
    if ((mode ==2)||(mode==3)) {
        
        //  ofEnableBlendMode(OF_BLENDMODE_ALPHA);
        float left,right,top,bottom;
        float wd2;
        float Sratio = kinect.width/kinect.height;
        float G_fov = CV_PI*50/180;
        float GNear = 10; // near cliping plane
        float Gf0 = 2000; // zero parallax plane
        float Gfar = 10000;
        ofVec3f evec;
        ofVec3f CamerasDir;
        ofVec3f CiclopLookAt;
        ofVec3f CicloPos;
        float G_D=0;
        
        wd2 = GNear*tan(G_fov/2.0);
        CiclopLookAt = (ofVec3f(0.0,0.0,0.0));
        CicloPos = ofVec3f(TheR*sin(Thetaxus)*cos(ThePhI),
                           TheR*sin(Thetaxus)*sin(ThePhI),
                           TheR*cos(Thetaxus));
        CamerasDir = CiclopLookAt - CicloPos;
        CamerasDir.normalize();
        // cross product to find the vector between cameras
        evec = CamerasDir.getCrossed(ofVec3f(0.0,1.0,0.0));
        evec.normalize();
        evec.x *= G_D/2.0;
        evec.y *= G_D/2.0;
        evec.z *= G_D/2.0;
//        glEnable(GL_BLEND);
//        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        Frame.begin();
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        left  = - Sratio * wd2 + 0.5 * G_D * GNear/Gf0;
        right =   Sratio * wd2 + 0.5 * G_D * GNear/Gf0;
        top    =   wd2;
        bottom = - wd2;
        glFrustum(left,right,bottom,top,GNear,Gfar);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(CicloPos.x - evec.x,CicloPos.y - evec.y,CicloPos.z - evec.z,
                  CicloPos.x - evec.x + CamerasDir.x,
                  CicloPos.y - evec.y + CamerasDir.y,
                  CicloPos.z - evec.z + CamerasDir.z,
                  0.0,1.0,0.0);
        ofClear(255,255,255,255);
        TheShader.begin();
        ofClear(0, 40, 200,255);
        glEnable( GL_DEPTH_TEST );
      //  glDisable( GL_CULL_FACE );
      //  glCullFace( GL_FRONT_AND_BACK );
        TheShader.setUniform1i("zebra", HayZebra);
        AllVertex.drawElements( GL_TRIANGLE_STRIP, RxF*2*(RyF-1)+2*(RyF-2));
        TheShader.end();
        Frame.end();
//        glDisable(GL_BLEND);
//        ofSetColor(255,255,255);
        Frame.draw(0.0, 0.0);
   
    }
    
}

void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			break;
			
		case'p':
			mode++;
			if(mode>3){mode=0;}
            break;
			
		
		case 't':
            Thetaxus+=PI/100.0;
			break;
			
	
		case 'g':
            Thetaxus-=PI/100.0;
			break;
        case 'r':
            ThePhI+=2*PI/100.0;
			break;
			
            
		case 'f':
            ThePhI-=2*PI/100.0;
			break;
        case 'e':
            TheR++;
			break;
			
            
		case 'd':
            TheR--;
			break;

		case 'q':
            ThresTemp++;
            cout<<ThresTemp<<endl;
			break;
			
		case 'a':
            ThresTemp--;
            cout<<ThresTemp<<endl;
			break;
            
        case 'z':
            HayZebra = (HayZebra==0);
            break;
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
		case OF_KEY_LEFT:
			Threshold--;
			break;
			
		case OF_KEY_RIGHT:
			Threshold  ++;
            cout<<ControP.x<<endl;
			cout<<ControP.y<<endl;
            break;
	}
}



void testApp::UpdateGeometry(void){
    
       
    // Setting the floor vertex
    
    
    float xlF =-kinect.width/2.0;
    float xuF =kinect.width/2.0;
    float ylF = -kinect.height/2.0;
    float yuF = kinect.height/2.0;
    float kinMax = 9758;
    float kinMin = 468;
    
    
    // Edges
//    cv::Mat RGBKinect;
//    RGBKinect.create(kinect.height, kinect.width, CV_8UC3);
//    ofxCvColorImage tempoCI;
//    tempoCI.allocate(kinect.width, kinect.height);
////    memcpy(RGBKinect.data, kinect.getPixels(), kinect.height*kinect.width*sizeof(unsigned char));
//    tempoCI.setFromPixels(kinect.getPixels(),kinect.width,kinect.height);
//    RGBKinect = tempoCI.getCvImage();
//    cv::Mat gris;
//    cv::cvtColor(RGBKinect, gris, CV_RGB2GRAY);
//    cv::Mat EdgeMat;
//    cv::Canny(gris, EdgeMat, 20, 100);
    
    
    
    
    // color data
    
    ofPixels RGBPix;
    RGBPix.setFromPixels(kinect.getPixels(), kinect.width, kinect.height, 3);

    
    
    for (int y =0; y<RyF; y++) {
        for (int x=0; x < RxF ; x++) {
            
            float xval = xlF +x*(xuF-xlF)/(float)RxF;
            vertKinet[RxF*y+x].x= xval;
            float yval = ylF +y*(yuF-ylF)/(float)RyF;
            vertKinet[RxF*y+x].y= yval;
            
            // Zero points that are too far away
            if ((float)depthval[RxF*y+x]>ThresTemp) {
              vertKinet[RxF*y+x].z = 0;
            }
            else{
                // give closer objects a bigger Z value
              if((float)depthval[RxF*y+x]>kinMin){
                vertKinet[RxF*y+x].z = (0.1)*(ThresTemp - (float)depthval[RxF*y+x]);
               }
                // zero points that are numerically low (errors)
               else{
                 vertKinet[RxF*y+x].z =0;
                }
            }
            //  vertKinet[RxF*y+x].z = (0.01)*((float)depthval[RxF*y+x]);
              //  vertKinet[RxF*y+x].z = sqrt(xval*xval+yval*yval);
            
//            ColorKinet[RxF*y+x].set((float)EdgeMat.data[RxF*y+x]/255.0,
//                                    (float)EdgeMat.data[RxF*y+x]/255.0,
//                                    (float)EdgeMat.data[RxF*y+x]/255.0);
            ColorKinet[RxF*y+x].set((float)RGBPix[RxF*3*y+3*x]/255.0,
                                    (float)RGBPix[RxF*3*y+3*x+1]/255.0,
                                    (float)RGBPix[RxF*3*y+3*x+2]/255.0);
                   }
    }
    
    // index vector
    
    int q =0;
    for (int y =0; y<RyF-1; y++) {
        for (int x=0; x < RxF ; x++) {
            IndKinet[q] = x + y*RxF;
            q++;
            IndKinet[q] = x + (y+1)*RxF;
            q++;
        }
        if( y < RyF-2){ // the degenerate triangles
            //repiting last one
            IndKinet[q] = (RxF-1) + (y+1)*RxF;
            q++;
            //repiting next one
            IndKinet[q] =  (y+1)*RxF;
            q++;
        }
        
    }
    
    
    AllVertex.setVertexData( &vertKinet[0], RxF*RyF, GL_DYNAMIC_DRAW);
    
    AllVertex.setIndexData( &IndKinet[0], RxF*2*(RyF-1)+2*(RyF-2), GL_DYNAMIC_DRAW );
    AllVertex.setColorData( &ColorKinet[0], RxF*RyF, GL_DYNAMIC_DRAW);
 
    
    
    
    
}







////////////////////LOW RES DEPTH IMAGE !!!!!//////////////////////
///////////////////////////////////////////////////////////////////
void testApp::UpdateGeometryLow(void){
    
    
    // Setting the floor vertex
    
    
    float xlF =-kinect.width/2.0;
    float xuF =kinect.width/2.0;
    float ylF = -kinect.height/2.0;
    float yuF = kinect.height/2.0;

    
    
    // color data
    
    ofPixels RGBPix;
    RGBPix.setFromPixels(kinect.getPixels(), kinect.width, kinect.height, 3);
    
    
    
    for (int y =0; y<RyF; y++) {
        for (int x=0; x < RxF ; x++) {
            
            float xval = xlF +x*(xuF-xlF)/(float)RxF;
            vertKinet[RxF*y+x].x= xval;
            float yval = ylF +y*(yuF-ylF)/(float)RyF;
            vertKinet[RxF*y+x].y= yval;
            
            vertKinet[RxF*y+x].z = 0.25*(float)depthChar[RxF*y+x];
            
            ColorKinet[RxF*y+x].set((float)RGBPix[RxF*3*y+3*x]/255.0,
                                    (float)RGBPix[RxF*3*y+3*x+1]/255.0,
                                    (float)RGBPix[RxF*3*y+3*x+2]/255.0);
        }
    }
    
    // index vector
    
    int q =0;
    for (int y =0; y<RyF-1; y++) {
        for (int x=0; x < RxF ; x++) {
            IndKinet[q] = x + y*RxF;
            q++;
            IndKinet[q] = x + (y+1)*RxF;
            q++;
        }
        if( y < RyF-2){ // the degenerate triangles
            //repiting last one
            IndKinet[q] = (RxF-1) + (y+1)*RxF;
            q++;
            //repiting next one
            IndKinet[q] =  (y+1)*RxF;
            q++;
        }
        
    }
    
    
    AllVertex.setVertexData( &vertKinet[0], RxF*RyF, GL_DYNAMIC_DRAW);
    
    AllVertex.setIndexData( &IndKinet[0], RxF*2*(RyF-1)+2*(RyF-2), GL_DYNAMIC_DRAW );
    AllVertex.setColorData( &ColorKinet[0], RxF*RyF, GL_DYNAMIC_DRAW);
    
    
    
    
    
}






//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
