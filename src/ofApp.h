#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxSpout2.h"
#include "ofxOsc.h"

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	string escape_quotes(const string & before);

	ofxKFW2::Device kinect;

	// offscreen buffers (frame buffer object)
	ofFbo fboDepth; // draw to for spout
	ofFbo fboColor; // draw to for spout

	// Spout obj
	ofxSpout2 spout;

	// OSC
	ofxOscSender oscSender;
};
