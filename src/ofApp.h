#pragma once

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
	bool NDIlock; // used to block NDI functions incase the ON/OFF param is activated.
	ofxNDIsender ndiSender1;    // NDI sender object, HD format (color_)
	ofxNDIsender ndiSender2;	// Depth-Image format (cutout_)
	ofxNDIsender ndiSender3;	// INFRARED
	ofxNDIsender ndiSender4;	// KEYED
	string color_StreamName;
	string cutout_StreamName;
	string depth_StreamName;
	string keyed_StreamName;
	string infrared_StreamName;  // TODO add this.. currently not setup
	char senderName[256];      // for conversions... char[] required for ndiSender functions

	// NOTE using W+H defined already
	//unsigned int senderWidth;  // Width of the sender output
	//unsigned int senderHeight; // Height of the sender output

	// ofFbo ndiFbo;              // NOTE using fbos created for spout

	// NOTE: need a Buffer array and a index used for async for EACH NDIstream being created
	ofPixels color_ndiBuffer[2];     // Two pixel buffers for async sending
	ofPixels cutout_ndiBuffer[2];     // Two pixel buffers for async sending
	ofPixels depth_ndiBuffer[2];     // Two pixel buffers for async sending
	ofPixels keyed_ndiBuffer[2];     // Two pixel buffers for async sending
	//ofPixels infrared_ndiBuffer[2];     // Two pixel buffers for async sending
	int color_idx;                   // Index used for async buffer swapping			???
	int cutout_idx;
	int depth_idx;
	int keyed_idx;
	// int infrared_idx;

	// PBO and control vars for ndiSender1 HD format
	GLuint ndiPbo1[2];
	int Pbo1Index;
	int NextPbo1Index;
	bool bUsePBO1;

	// PBO and control vars for ndiSender2+ DepthImage sized
	GLuint ndiPbo2[2];
	int Pbo2Index;
	int NextPbo2Index;
	bool bUsePBO2;


	bool ReadFboPixels(ofFbo fbo, unsigned int width, unsigned int height, unsigned char *data);
	//  ^^^ added from NDI sender example ^^^


	// OSC
	ofxOscSender oscSender;
	ofxOscReceiver oscReceiver;

	// custom functions DX
	void oscSendMsg(std::string message, std::string address);


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

	ofxGuiGroup NDIgroup;
	ofxToggle ndiActive;
	ofxToggle ndiCutOut;
	ofxToggle ndiColor;
	ofxToggle ndiKeyed;
	ofxToggle ndiDepth;


	// added for coordmapping
	ofImage bodyIndexImg, foregroundImg;
	vector<ofVec2f> colorCoords;
	int numBodiesTracked;
	bool bHaveAllStreams;

	// helper Functions
	string escape_quotes(const string & before);
	void body2JSON(vector<ofxKinectForWindows2::Data::Body> bodies, const char * jointNames[]);
	void sendNDI(ofxNDIsender & ndiSender, ofFbo & sourceFBO, bool bUsePBO,  int senderWidth, int senderHeight, char senderName[256], ofPixels ndiBuffer[], int idx);
};