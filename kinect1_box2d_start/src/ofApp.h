#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxCv.h"
#include "ofxBox2d.h"

using namespace cv;
using namespace ofxCv;

class ofApp : public ofBaseApp{
	
public:
	void setup();
	void setupGui();
	void update();
	void draw();
	void drawGui(ofEventArgs & args);
	void exit();
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	
	
	void initBoxesPosition();
	
	int mainW  = 800;
	int mainH  = 600;
	int guiW   = 300;
	int guiH   = 300;
	
	ofxBox2d box2d;   // the box2d world
	ofPolyline bodyLine;
//	ofPath bodyPath;


	// number of drop point when combine bodyLine for bodyGround , biger ,smoohter
	ofParameter<int> bodyLineSmooth;
//	int bodyLineSmooth;
	
	ofxBox2dEdge bodyGround;
	
	ofxBox2dEdge leftBar; // the box2d edge/line shape (min 2 points)
	ofxBox2dEdge rightBar; // the box2d edge/line shape (min 2 points)
	ofxBox2dEdge topBar; // the box2d edge/line shape (min 2 points)

	
//	vector <shared_ptr<ofxBox2dCircle>>	circles; //	default box2d circles
//	int circlesNum = 100;

	std::vector<std::shared_ptr<ofxBox2dRect>> boxes;
	int boxNum = 20;
	float boxDense = 0.1f;
	float boxBounce = 0.53;
	float boxFriction = 0.5;
	int boxSize = 50;
	
	
	bool initBoxesPositionBool = false;
	
	
	ofParameterGroup   parameters;
//	ofParameter<float> radius;
//	ofParameter<ofColor> color;
	ofxPanel gui;
	
	ofxKinect kinect;
	
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
	
	int markerPosX;
	int markerPosY;
	ofRectangle marker;
	
	
	
	
	
	
};
