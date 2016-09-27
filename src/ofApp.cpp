#include "ofApp.h"

#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424
#define DEPTH_SIZE DEPTH_WIDTH * DEPTH_HEIGHT

#define COLOR_WIDTH 1920
#define COLOR_HEIGHT 1080

int previewWidth = DEPTH_WIDTH; // width and hieght of Depth Camera scaled
int previewHeight = DEPTH_HEIGHT;

//--------------------------------------------------------------
void ofApp::setup() {
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();


	// added for coordmapping
	if (kinect.getSensor()->get_CoordinateMapper(&coordinateMapper) < 0) {
		ofLogError() << "Could not acquire CoordinateMapper!";
	}

	numBodiesTracked = 0;
	bHaveAllStreams = false;

	//bodyIndexImg.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);
	foregroundImg.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR_ALPHA);

	colorCoords.resize(DEPTH_WIDTH * DEPTH_HEIGHT);
	// end add for coordmapping


	ofSetWindowShape(previewWidth * 3, previewHeight * 2);

	//ofDisableArbTex();
	bgCB.load("images/checkerbg.png");

	fboDepth.allocate(previewWidth, previewHeight, GL_RGBA); //setup offscreen buffer in openGL RGBA mode
	fboColor.allocate(COLOR_WIDTH, COLOR_HEIGHT, GL_RGB); //setup offscreen buffer in openGL RGB mode

	gui.setup("Parameters", "settings.xml");
	gui.add(jsonGrouped.setup("OSC as JSON", true));
	gui.add(HostField.setup("OSC host ip", "localhost"));
	gui.add(spoutCutOut.setup("BnW cutouts -> spout", true));
	gui.add(spoutColor.setup("Color -> spout", true));

	gui.loadFromFile("settings.xml");
	
	// HostField.addListener(this, &ofApp::HostFieldChanged);

	oscSender.disableBroadcast();
	oscSender.setup(HostField, 1234);
}


//--------------------------------------------------------------
void ofApp::HostFieldChanged() {
	cout << "fieldChange" << endl;
	oscSender.disableBroadcast();
	oscSender.setup(HostField, 1234);
	cout << "updated" << endl;
}


//--------------------------------------------------------------
void ofApp::update() {
	kinect.update();

	// Get pixel data
	auto& depthPix = kinect.getDepthSource()->getPixels();
	auto& bodyIndexPix = kinect.getBodyIndexSource()->getPixels();
	auto& colorPix = kinect.getColorSource()->getPixels();

	// Make sure there's some data here, otherwise the cam probably isn't ready yet
	if (!depthPix.size() || !bodyIndexPix.size() || !colorPix.size()) {
		bHaveAllStreams = false;
		return;
	}
	else {
		bHaveAllStreams = true;
	}

	// Count number of tracked bodies
	numBodiesTracked = 0;
	auto& bodies = kinect.getBodySource()->getBodies();
	for (auto& body : bodies) {
		if (body.tracked) {
			numBodiesTracked++;
		}
	}

	// Do the depth space -> color space mapping
	// More info here:
	// https://msdn.microsoft.com/en-us/library/windowspreview.kinect.coordinatemapper.mapdepthframetocolorspace.aspx
	// https://msdn.microsoft.com/en-us/library/dn785530.aspx
	coordinateMapper->MapDepthFrameToColorSpace(DEPTH_SIZE, (UINT16*)depthPix.getPixels(), DEPTH_SIZE, (ColorSpacePoint*)colorCoords.data());

	// Loop through the depth image
	for (int y = 0; y < DEPTH_HEIGHT; y++) {
		for (int x = 0; x < DEPTH_WIDTH; x++) {
			int index = (y * DEPTH_WIDTH) + x;

			ofColor trans(0,0,0,0);
			//ofColor wht(255, 255, 255);

			//bodyIndexImg.setColor(x, y, wht);
			foregroundImg.setColor(x, y, trans);

			// This is the check to see if a given pixel is inside a tracked  body or part of the background.
			// If it's part of a body, the value will be that body's id (0-5), or will > 5 if it's
			// part of the background
			// More info here:
			// https://msdn.microsoft.com/en-us/library/windowspreview.kinect.bodyindexframe.aspx
			float val = bodyIndexPix[index];
			if (val >= bodies.size()) {
				continue; // exit for loop without executing the following code
			}

			// Give each tracked body a color value so we can tell
			// them apart on screen
			//ofColor c = ofColor::fromHsb(val * 255 / bodies.size(), 200, 128, 64);
			//bodyIndexImg.setColor(x, y, c);

			// For a given (x,y) in the depth image, lets look up where that point would be
			// in the color image
			ofVec2f mappedCoord = colorCoords[index];

			// Mapped x/y coordinates in the color can come out as floats since it's not a 1:1 mapping
			// between depth <-> color spaces i.e. a pixel at (100, 100) in the depth image could map
			// to (405.84637, 238.13828) in color space
			// So round the x/y values down to ints so that we can look up the nearest pixel
			mappedCoord.x = floor(mappedCoord.x);
			mappedCoord.y = floor(mappedCoord.y);

			// Make sure it's within some sane bounds, and skip it otherwise
			if (mappedCoord.x < 0 || mappedCoord.y < 0 || mappedCoord.x >= COLOR_WIDTH || mappedCoord.y >= COLOR_HEIGHT) {
				continue;
			}

			// Finally, pull the color from the color image based on its coords in
			// the depth image
			foregroundImg.setColor(x, y, colorPix.getColor(mappedCoord.x, mappedCoord.y));
		}
	}

	// Update the images since we manipulated the pixels manually. This uploads to the
	// pixel data to the texture on the GPU so it can get drawn to screen
	//bodyIndexImg.update();
	foregroundImg.update();










	//--
	//Getting joint positions (skeleton tracking)
	//--
	//

	// *****************  ENUM copied from kinectv2 addon
	//  JointType_SpineBase = 0,
	//	JointType_SpineMid = 1,
	//	JointType_Neck = 2,
	//	JointType_Head = 3,
	//	JointType_ShoulderLeft = 4,
	//	JointType_ElbowLeft = 5,
	//	JointType_WristLeft = 6,
	//	JointType_HandLeft = 7,
	//	JointType_ShoulderRight = 8,
	//	JointType_ElbowRight = 9,
	//	JointType_WristRight = 10,
	//	JointType_HandRight = 11,
	//	JointType_HipLeft = 12,
	//	JointType_KneeLeft = 13,
	//	JointType_AnkleLeft = 14,
	//	JointType_FootLeft = 15,
	//	JointType_HipRight = 16,
	//	JointType_KneeRight = 17,
	//	JointType_AnkleRight = 18,
	//	JointType_FootRight = 19,
	//	JointType_SpineShoulder = 20,
	//	JointType_HandTipLeft = 21,
	//	JointType_ThumbLeft = 22,
	//	JointType_HandTipRight = 23,
	//	JointType_ThumbRight = 24,
	//	JointType_Count = (JointType_ThumbRight + 1)

	//const char * jointNames[] = { "SpineBase", "SpineMid", "Neck", "Head",
	//	"ShoulderLeft", "ElbowLeft", "WristLeft", "HandLeft",
	//	"ShoulderRight", "ElbowRight", "WristRight", "HandRight",
	//	"HipLeft", "KneeLeft", "AnkleLeft", "FootLeft",
	//	"HipRight", "KneeRight", "AnkleRight", "FootRight",
	//	"SpineShoulder", "HandTipLeft", "ThumbLeft", "HandTipRight", "ThumbRight", "Count" };

	// shorten names to minimize packet size
	const char * jointNames[] = { "SpineBase", "SpineMid", "Neck", "Head",
		"ShldrL", "ElbowL", "WristL", "HandL",
		"ShldrR", "ElbowR", "WristR", "HandR",
		"HipL", "KneeL", "AnkleL", "FootL",
		"HipR", "KneeR", "AnkleR", "FootR",
		"SpineShldr", "HandTipL", "ThumbL", "HandTipR", "ThumbR", "Count" };

	// MORE joint. values >>>
	// second. positionInWorld[] x y z , positionInDepthMap[] x y
	// second. orientation. _v[] x y z w  ??what is this
	// second. trackingState

	// MORE body. values >>>
	// body. tracked (bool)
	// body. leftHandState (_Handstate) enum?
	// body. rightHandState (_Handstate)
	// body. activity  ??what is this

					//TODO: implement switch for message type, and add a single JSON output for all data
					// http://stackoverflow.com/questions/31121378/json-cpp-how-to-initialize-from-string-and-get-string-value
					// http://uscilab.github.io/cereal/

// defined in new coordmap section as &
//	auto bodies = kinect.getBodySource()->getBodies();


	if (jsonGrouped) {
			for (auto body : bodies) {
				string bdata = ""; // start JSON array build of body data
				string jdata = ""; // start JSON array build of joints data
				for (auto joint : body.joints) {
					auto pos = joint.second.getPositionInWorld();
					string name = jointNames[joint.first];
					//jdata = "\"" + name + "\"," + to_string(pos.x) + "," + to_string(pos.y) + "," + to_string(pos.z);
					jdata = "\"j\":";
					jdata = jdata + "\"" + name + "\",";
					jdata = jdata + "\"x\":" + to_string(pos.x) + ",";
					jdata = jdata + "\"y\":" + to_string(pos.y) + ",";
					jdata = jdata + "\"z\":" + to_string(pos.z);
					jdata = "{" + jdata + "}";
					// format= {"\joint\":\"jointName\",\"x\":0.1,\"y\":0.2,\"z\":0.3 }
					if (bdata == "") {  // if bdata = "" no comma
						bdata = jdata;
					}
					else {
						bdata = bdata + "," + jdata;
					}
				} // end inner joints loop
				// need to escape all " in bdata
				bdata = escape_quotes(bdata);
				bdata = "[" + bdata + "]";
				bdata = "{\"b" + to_string(body.bodyId) + "\": \"" + bdata + "\"}";
				//cout << bdata << endl;
				ofxOscMessage m;
				//string adrs = "/body/" + to_string(body.bodyId);
				string adrs = "/kV2/body/" + to_string(body.bodyId);
				m.setAddress(adrs);
				m.addStringArg(bdata);
				oscSender.sendMessage(m);

				//cout << bdata.length() << endl;  // TEST
				//if (bdata.length() < 1000) {
				//	cout << "data: " + bdata << endl;  // TEST
				//}
			} // end body loop
	}else{
			for (auto body : bodies) {
				for (auto joint : body.joints) {
					auto pos = joint.second.getPositionInWorld();
					ofxOscMessage m;
					string adrs = "/" + to_string(body.bodyId) + "/" + jointNames[joint.first];
					m.setAddress(adrs);
					//float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
					//m.addFloatArg( r );
					//m.addFloatArg(joint.second.positionInWorld.x);  // .x access allowed in watcher, but not here (positionInWorld is type protected in, class Jount)
					m.addFloatArg(pos.x);
					m.addFloatArg(pos.y);
					m.addFloatArg(pos.z);
					m.addStringArg(jointNames[joint.first]);
					oscSender.sendMessage(m);

				} // end inner joints loop
			} // end body loop

	} // end if/else



	//--
	//Getting bones (connected joints)
	//--
	//

	{
		// Note that for this we need a reference of which joints are connected to each other.
		// We call this the 'boneAtlas', and you can ask for a reference to this atlas whenever you like
		auto bodies = kinect.getBodySource()->getBodies();
		auto boneAtlas = ofxKinectForWindows2::Data::Body::getBonesAtlas();

		for (auto body : bodies) {
			for (auto bone : boneAtlas) {
				auto firstJointInBone = body.joints[bone.first];
				auto secondJointInBone = body.joints[bone.second];

				//now do something with the joints
			}
		}
	}

	//
	//--
}



string ofApp::escape_quotes(const string &before)
// sourced from: http://stackoverflow.com/questions/1162619/fastest-quote-escaping-implementation
{
	string after;
	after.reserve(before.length() + 4);  // TODO: may need to increase reserve...

	for (string::size_type i = 0; i < before.length(); ++i) {
		switch (before[i]) {
		case '"':
		case '\\':
			after += '\\';
			// Fall through.
		default:
			after += before[i];
		}
	}
	return after;
}

//--------------------------------------------------------------
void ofApp::draw() {
	bgCB.draw(0, 0, ofGetWidth(), ofGetHeight());

	// Color is at 1920x1080 instead of 512x424 so we should fix aspect ratio
	float colorHeight = previewWidth * (kinect.getColorSource()->getHeight() / kinect.getColorSource()->getWidth());
	float colorTop = (previewHeight - colorHeight) / 2.0;

	{
		// Draw Depth Source
		// TODO: brighten depth image. https://github.com/rickbarraza/KinectV2_Lessons/tree/master/3_MakeRawDepthBrigther
		// MORE: https://forum.openframeworks.cc/t/kinect-v2-pixel-depth-and-color/18974/4
		kinect.getDepthSource()->draw(0, 0, previewWidth, previewHeight);  // note that the depth texture is RAW so may appear dark
	}


	{
		// Draw Color Source
		fboColor.begin(); // start drawing to off screenbuffer
		kinect.getColorSource()->draw(0, 0, COLOR_WIDTH, COLOR_HEIGHT);
		//kinect.getColorSource()->draw(previewWidth, 0 + colorTop, previewWidth, colorHeight);
		fboColor.end();
		//Spout
		if (spoutColor) {
			spout.sendTexture(fboColor.getTextureReference(), "kv2_color");
		}
		//Draw from FBO
		fboColor.draw(previewWidth, 0 + colorTop, previewWidth, colorHeight);
		//fboColor.clear();
	}


	{
		// Draw IR Source
		kinect.getInfraredSource()->draw(0, previewHeight, previewWidth, previewHeight);
		//kinect.getLongExposureInfraredSource()->draw(0, previewHeight, previewWidth, previewHeight);
	}


	{
		// Draw B+W cutout of Bodies
		fboDepth.begin(); // start drawing to off screenbuffer
		kinect.getBodyIndexSource()->draw(0, 0, previewWidth, previewHeight);
		fboDepth.end();
		//Spout
		if (spoutCutOut) {
			spout.sendTexture(fboDepth.getTextureReference(), "kv2_cutout");
		}
		//Draw from FBO
		fboDepth.draw(previewWidth, previewHeight, previewWidth, previewHeight);
		//fboDepth.clear();
	}


	{
		// Draw bodies joints+bones over
		kinect.getBodySource()->drawProjected(previewWidth, previewHeight, previewWidth, previewHeight, ofxKFW2::ProjectionCoordinates::DepthCamera);
		//kinect.getBodySource()->drawProjected(0, 0, previewWidth, previewHeight);
	}

	//basic from coordmaping
	//ofSetColor(255, 255, 255, 255);  //allows to draw with a constant opacity
	//bodyIndexImg.draw(previewWidth * 2, previewHeight);
	
	{
		//ofClear(255, 255, 255, 0);
		// greenscreen fx from coordmaping
		//foregroundImg.draw(previewWidth * 2, 0, previewWidth, previewHeight);
		//fboDepth.clear();
		fboDepth.begin(); // start drawing to off screenbuffer
			//kinect.getBodyIndexSource()->draw(0, 0, previewWidth, previewHeight);
			ofClear(255, 255, 255, 0);
			foregroundImg.draw(0, 0, previewWidth, previewHeight);
		fboDepth.end();
		//Spout
		if (spoutCutOut) {
			spout.sendTexture(fboDepth.getTextureReference(), "kv2_keyed");
		}
		//Draw from FBO
		ofEnableBlendMode(OF_BLENDMODE_ALPHA);
		fboDepth.draw(previewWidth * 2, 0, previewWidth, previewHeight);
		//fboDepth.clear();
	}


	stringstream ss;
	ss << "fps : " << ofGetFrameRate() << endl;
	ss << "Tracked bodies: " << numBodiesTracked;
	if (!bHaveAllStreams) ss << endl << "Not all streams detected!";
	ofDrawBitmapStringHighlight(ss.str(), 20, previewHeight * 2 - 20);

	gui.draw();
}


void ofApp::exit() {
	gui.saveToFile("settings.xml");
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
	// https://forum.openframeworks.cc/t/keypressed-and-getting-the-special-keys/5727
	if (key == OF_KEY_RETURN) {
		cout << "ENTER" << endl;
		HostFieldChanged();
	}

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
	// textField = ofToString(w) + "x" + ofToString(h);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}


//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
