#pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"") // Hide console window

#include "ofMain.h"
#include "ofApp.h"

/*
*  kinect2share
*
*  Created by Ryan Webber
*  http://www.DusXproductions.com
*  https://github.com/rwebber
*
*  The goal of this project is to make as many Kinect2 features available to creative platforms as possible,
*  using open standards including OSC (opensoundcontrol), Spout, and NDI (https://www.newtek.com/ndi/).
*
*  Specific care has been given to providing a demo file for use with the Isadora creativity server.
*  The demo file provides basic functional examples that Isadora users can build upon.
*  http://troikatronix.com/
*
*  MIT License http://en.wikipedia.org/wiki/MIT_License
*
*  This project is built using OpenFrameWorks and utilizes a number of amazing addons offered by the community.
*  Please read the ReadMe file included in the github reprository, for details.
*/

//========================================================================
int main( ){

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:

	//ofSetupOpenGL(1024,768,OF_WINDOW); // <-------- setup the GL context
	//ofRunApp(new ofApp());


	// custom windows setup.
	ofGLFWWindowSettings settings;
	//settings.iconified = true;
	settings.resizable = false;
	settings.width = 1024;
	settings.height = 768;
	ofCreateWindow(settings);
	return ofRunApp(new ofApp);

}
