#include "ofApp.h"
using namespace cv;

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
	
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 245;
	farThreshold = 213;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(30);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
	
	markerPosX = 0;
	markerPosY = 0;
	
	
	
	//	init box2d
	box2d.init();
	box2d.setGravity(0, 5);
	box2d.createGround();
	box2d.setFPS(30.0);
	box2d.registerGrabbing();
	
	leftBar.addVertex(0,mainH * 2);
	leftBar.addVertex(0,0);
	leftBar.setPhysics(0.0, 0.5, 0.5);
	leftBar.create(box2d.getWorld());
	
	rightBar.addVertex(mainW,mainH * 2);
	rightBar.addVertex(mainW,0);
	rightBar.setPhysics(0.0, 0.5, 0.5);
	rightBar.create(box2d.getWorld());
	
	topBar.addVertex(0,mainH * 2);
	topBar.addVertex(mainW,mainH * 2);
	topBar.setPhysics(0.0, 0.5, 0.5);
	topBar.create(box2d.getWorld());
	
	bodyMesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
	
}

void ofApp::initBoxesPosition(){
	for (int i=0; i<boxNum; i++) {
		boxes.push_back(std::make_shared<ofxBox2dRect>());
		boxes.back()->setPhysics(boxDense, boxBounce, boxFriction);
		boxes.back()->setup(box2d.getWorld(),
							ofRandom(0,mainW - boxSize),
							0,
							boxSize,
							boxSize);
		
	}
}
//--------------------------------------------------------------
void ofApp::setupGui(){
	parameters.setName("parameters");
	parameters.add(bodyLineSmooth.set("body line smooth",1,1,20));
	//	parameters.add(color.set("color",100,ofColor(0,0),255));
	gui.setup(parameters);
	ofSetBackgroundColor(0);
}

//--------------------------------------------------------------
void ofApp::drawGui(ofEventArgs & args){
	gui.draw();
}


//--------------------------------------------------------------
void ofApp::exit(){
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------

void ofApp::update(){
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		//		Canny(grayImage, grayImage, 100, 200, 3);
		
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20,
								   // if use approximate points
								   //								   true
								   false
								   );
		
		
		//	get points of contour .............................
		
		if(contourFinder.blobs.size() > 0){
			
			//clear all
//			bodyLine.clear();
			bodyPath.clear();
			bodyMesh.clear();
			
			
			//		cout << contourFinder.blobs.at(0).nPts << endl;
			int nPts = contourFinder.blobs.at(0).nPts;
			for (int i = 0; i<nPts; i++) {
				
//				if(i * bodyLineSmooth < nPts){
//					int x = contourFinder.blobs.at(0).pts.at(i * bodyLineSmooth)[0];
//					int y = contourFinder.blobs.at(0).pts.at(i * bodyLineSmooth)[1];
//					bodyLine.addVertex(x,y);
//
//
//				}

				if(i * bodyLineSmooth < nPts){
					int x = contourFinder.blobs.at(0).pts.at(i * bodyLineSmooth)[0];
					int y = contourFinder.blobs.at(0).pts.at(i * bodyLineSmooth)[1];

					if(i == 0){
						bodyPath.moveTo(x,y);
					}else{
						bodyPath.lineTo(x,y);
					}
				
				}
				
				
			}
			
			bodyPath.close();
			bodyPath.simplify();
			bodyMesh = bodyPath.getTessellation();
			
			
			vector<ofPoint> tempPointVector;
			for (int i = 0; i < bodyMesh.getNumIndices(); i++) {
//				cout << bodyMesh.getVertex(bodyMesh.getIndex(i)) << endl;
				ofPoint p = bodyMesh.getVertex(bodyMesh.getIndex(i));
				tempPointVector.push_back(p);
				
			}
//			bodyLine.close();
//			bodyLine.simplify();
			
//			bodyGround.clear();
//			bodyGround.addVertexes(bodyLine);
//			bodyGround.create(box2d.getWorld());
		}
		//	...................................................
		
		
		
		//		update when kinect updated , let box2d waiting kinect
		box2d.update();
		
	}
	
	
	
	
	if(initBoxesPositionBool){
		initBoxesPosition();
		initBoxesPositionBool = !initBoxesPositionBool;
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(255, 255, 255);
	
	// draw from the live kinect
	//	kinect.drawDepth(10, 10, 400, 300);
	//	kinect.draw(420, 10, 400, 300);
	
	//		grayImage.draw(10, 320, 400, 300);
	//	contourFinder.draw(10, 320, 400, 300);
	
	ofSetColor(0, 255, 0);
//	bodyGround.updateShape();
//	bodyGround.draw();
	
	
	
	// draw instructions
	ofSetColor(255, 255, 255);
	
	//	show boxes by box2d
	for (int i = 0;i<boxes.size();i++) {
		boxes[i]->draw();
		
	}
	
	
	
	stringstream reportStream;
	
	if(kinect.hasAccelControl()) {
		reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
		<< ofToString(kinect.getMksAccel().y, 2) << " / "
		<< ofToString(kinect.getMksAccel().z, 2) << endl;
	} else {
		reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
	}
	
	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
	
	if(kinect.hasCamTiltControl()) {
		reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
		<< "press 1-5 & 0 to change the led mode" << endl;
	}
	
	ofDrawBitmapString(reportStream.str(), 20, 652);
	
	ofSetColor(255, 0, 0);
	ofFill();
	//	cout << markerPosX << ":" << markerPosY << endl;
	//	ofRectangle(10 + markerPosX,320 + markerPosY,30,30);
	
	//	markerPosX = contourFinder.blobs.at(0).pts.at(0)[0];
	//	cout << contourFinder.blobs.size() << endl;
	
	
	
	
	// draw contour via opencv ............................
	//	if(contourFinder.blobs.size() > 0){
	//
	//
	//		cout << contourFinder.blobs.at(0).nPts << endl;
	//
	//		for (int i = 0; i<contourFinder.blobs.at(0).nPts; i++) {
	//			//		cout << contourFinder.blobs.at(0).pts.at(0)[1] << endl;
	//
	//			ofRect(contourFinder.blobs.at(0).pts.at(i)[0],contourFinder.blobs.at(0).pts.at(i)[1],10,10);
	//		}
	//
	//	}
	//	.................................................
	
	
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case 's':
			initBoxesPositionBool = true;
			break;
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
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
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
	
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
	
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
	
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
	
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
	
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
	
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
	
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
	
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
	
}
