#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxOsc.h"
#include "ofxGui.h"
#include "ofxInputField.h"
#include "ofxSpout2.h"
#include "ofxNDI.h"

//  ** added from NDI sender example **
// BGRA definition should be in glew.h 
// but define it here just in case it is not
#ifndef GL_BGRA_EXT
#define GL_BGRA_EXT 0x80E1
#endif
//  ^^ added from NDI sender example ^^

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();
	void exit(); // added

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofxKFW2::Device kinect;
	ICoordinateMapper* coordinateMapper;

	// void HostFieldChanged(string & HostField);
	void HostFieldChanged();

	// BG
	ofImage bgCB; // background checkerboard

	// offscreen buffers (frame buffer object)
	ofFbo fboDepth; // draw to for spout, setup at Kinect native 512x
	ofFbo fboColor; // draw to for spout, setup at 1080x

	// Spout obj
	ofxSpout2 spout;


	//  *** added from NDI sender example ***
	// NDI definitions
	ofxNDIsender ndiSender;    // NDI sender object
	string color_StreamName;
	char senderName[256];      // for conversions... char[] required for ndiSender functions

	// NOTE using W+H defined already
	//unsigned int senderWidth;  // Width of the sender output
	//unsigned int senderHeight; // Height of the sender output

	// ofFbo ndiFbo;              // NOTE using fbos created for spout

	// NOTE: need a Buffer array and a index used for async for EACH NDIstream being created
	ofPixels color_ndiBuffer[2];     // Two pixel buffers for async sending
	int color_idx;                   // Index used for async buffer swapping			???

	// TODO ???? may need a set of these for each NDIstream
	GLuint ndiPbo[2];
	int PboIndex;
	int NextPboIndex;
	bool bUsePBO;
	bool ReadFboPixels(ofFbo fbo, unsigned int width, unsigned int height, unsigned char *data);
	//  ^^^ added from NDI sender example ^^^


	// OSC
	ofxOscSender oscSender;

	// GUI
	ofxPanel gui;

	ofxGuiGroup OSCgroup;
	ofxToggle jsonGrouped;
	// ofxInputField
	ofxIntField oscPort;
	ofxTextField HostField;

	ofxGuiGroup SPOUTgroup;
	ofxToggle spoutCutOut;
	ofxToggle spoutColor;
	ofxToggle spoutKeyed;
	ofxToggle spoutDepth;


	// added for coordmapping
	ofImage bodyIndexImg, foregroundImg;
	vector<ofVec2f> colorCoords;
	int numBodiesTracked;
	bool bHaveAllStreams;

	// helper Functions
	string escape_quotes(const string & before);
	void body2JSON(vector<ofxKinectForWindows2::Data::Body> bodies, const char * jointNames[]);
	void sendNDI(ofFbo & sourceFBO, int senderWidth, int senderHeight, char senderName[256], ofPixels ndiBuffer[], int idx);
};