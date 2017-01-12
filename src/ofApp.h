#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxOsc.h"
#include "ofxGui.h"
#include "ofxInputField.h"
#include "ofxSpout2.h"

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

	string escape_quotes(const string & before);
	void body2JSON(vector<ofxKinectForWindows2::Data::Body> bodies, const char * jointNames[]);

	// void HostFieldChanged(string & HostField);
	void HostFieldChanged();

	// BG
	ofImage bgCB; // background checkerboard

	// offscreen buffers (frame buffer object)
	ofFbo fboDepth; // draw to for spout, setup at Kinect native 512x
	ofFbo fboColor; // draw to for spout, setup at 1080x

	// Spout obj
	ofxSpout2 spout;

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
};