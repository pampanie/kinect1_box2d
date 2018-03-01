#include "ofMain.h"
#include "ofApp.h"

#include "ofAppGLFWWindow.h"

//========================================================================
int main( ){
	//	ofSetupOpenGL(1024,768,OF_WINDOW);			// <-------- setup the GL context
	//
	//	// this kicks off the running of my app
	//	// can be OF_WINDOW or OF_FULLSCREEN
	//	// pass in width and height too:
	//	ofRunApp(new ofApp());
	
	
//	better way to include var from ofapp.h ***********  TDOO
	int mainW = 800;
	int mainH = 600;
	int guiW = 300;
	int guiH = 300;

	
	
	
	
	
	//	main window ===============
	ofGLFWWindowSettings settings;
//	main window size
	settings.width = mainW;
	settings.height = mainH;
//	main window position
	settings.setPosition(ofVec2f(guiW,0));
	//	settings.resizable = true;
	shared_ptr<ofAppBaseWindow> mainWindow = ofCreateWindow(settings);

//	gui window size
	settings.width = guiW;
	settings.height = guiH;
//	gui window position
	settings.setPosition(ofVec2f(0,0));
	settings.resizable = false;
	// uncomment next line to share main's OpenGL resources with gui
	//settings.shareContextWith = mainWindow;
	shared_ptr<ofAppBaseWindow> guiWindow = ofCreateWindow(settings);
	guiWindow->setVerticalSync(false);
	
	shared_ptr<ofApp> mainApp(new ofApp);
//	self custome function setupGui
	mainApp->setupGui();
//	self make function drawGui
	ofAddListener(guiWindow->events().draw,mainApp.get(),&ofApp::drawGui);
	
	ofRunApp(mainWindow, mainApp);
	ofRunMainLoop();
	
	
}
