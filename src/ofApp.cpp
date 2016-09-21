#include "ofApp.h"

int previewWidth = 512; // width and hieght of Depth Camera
int previewHeight = 424;

//--------------------------------------------------------------
void ofApp::setup() {
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	ofSetWindowShape(previewWidth * 2, previewHeight * 2);

	fboDepth.allocate(previewWidth, previewHeight, GL_RGB); //setup offscreen buffer in openGL RGB mode
	fboColor.allocate(1920, 1080, GL_RGB); //setup offscreen buffer in openGL RGB mode

	oscSender.setup("localhost", 1234);

	// TODO: add text input for ip address etc.. https://github.com/fx-lange/ofxInputField/

	gui.setup("Parameters", "settings.xml");
	gui.add(jsonGrouped.setup("as JSON", false));

	gui.loadFromFile("settings.xml");
}



//--------------------------------------------------------------
void ofApp::update() {
	kinect.update();

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

	auto bodies = kinect.getBodySource()->getBodies();

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
	// Draw Depth Source
	kinect.getDepthSource()->draw(0, 0, previewWidth, previewHeight);  // note that the depth texture is RAW so may appear dark

	// Color is at 1920x1080 instead of 512x424 so we should fix aspect ratio
	float colorHeight = previewWidth * (kinect.getColorSource()->getHeight() / kinect.getColorSource()->getWidth());
	float colorTop = (previewHeight - colorHeight) / 2.0;

	// Draw Color Source
	fboColor.begin(); // start drawing to off screenbuffer
		kinect.getColorSource()->draw(0, 0, 1920, 1080);
	fboColor.end();
	//Spout
	spout.sendTexture(fboColor.getTextureReference(), "kv2_color");
	//Draw from FBO
	fboColor.draw(previewWidth, 0 + colorTop, previewWidth, colorHeight);
	//kinect.getColorSource()->draw(previewWidth, 0 + colorTop, previewWidth, colorHeight);
	//fboColor.clear();


	// Draw bodies joints+bones over
	kinect.getBodySource()->drawProjected(0, 0 + colorTop, previewWidth, colorHeight);

	// Draw IR Source
	kinect.getInfraredSource()->draw(0, previewHeight, previewWidth, previewHeight);
	//kinect.getLongExposureInfraredSource()->draw(0, previewHeight, previewWidth, previewHeight);


	// Draw B+W cutout of Bodies
	fboDepth.begin(); // start drawing to off screenbuffer
		kinect.getBodyIndexSource()->draw(0, 0, previewWidth, previewHeight);
	fboDepth.end();
	//Spout
	spout.sendTexture(fboDepth.getTextureReference(), "kv2_cutout");
	//Draw from FBO
	fboDepth.draw(previewWidth, previewHeight, previewWidth, previewHeight);
	//fboDepth.clear();

	//// Draw B+W cutout of Bodies
	//fboDepth.clear();
	//fboColor.begin(); // start drawing to off screenbuffer
	//	kinect.getBodyIndexSource()->draw(0, 0, 1920, 1080);
	//fboColor.end();
	////Spout
	//spout.sendTexture(fboColor.getTextureReference(), "kv2_cutout");
	////Draw from FBO
	//fboColor.draw(previewWidth, previewHeight, previewWidth, previewHeight);
	////fboDepth.clear();

	//kinect.drawWorld()->draw(0,0, previewWidth *2, previewHeight *2);

	// Draw bodies joints+bones over
	// kinect.getBodySource()->drawProjected(previewWidth, previewHeight, previewWidth, previewHeight, ofxKFW2::ProjectionCoordinates::DepthCamera);

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

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}


//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
