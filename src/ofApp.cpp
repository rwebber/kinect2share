#include "ofApp.h"

#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424
#define DEPTH_SIZE DEPTH_WIDTH * DEPTH_HEIGHT

#define COLOR_WIDTH 1920
#define COLOR_HEIGHT 1080

int previewWidth = DEPTH_WIDTH / 2; // width and hieght of Depth Camera scaled
int previewHeight = DEPTH_HEIGHT / 2;

// REF: http://www.cplusplus.com/reference/cstring/

// TODO: look into https://forum.openframeworks.cc/t/ofxkinectforwindows2-depth-threshold-for-blob-tracking/19012/2

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetWindowTitle("kinect2share");
	ofSetFrameRate(30);
	ofSetVerticalSync(true);

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
	foregroundImg.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR_ALPHA);
	colorCoords.resize(DEPTH_WIDTH * DEPTH_HEIGHT);
	// end add for coordmapping

	ofSetWindowShape(previewWidth * 3, previewHeight * 2);

	//ofDisableArbTex(); // needed for textures to work... May be needed for NDI?
	// seems above is needed if loading an image file -> texture
	bgCB.load("images/checkerbg.png");

	// TODO: depth and IR to be added -> fboDepth
	fboDepth.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, GL_RGBA); //setup offscreen buffer in openGL RGBA mode (used for Keyed and B+W bodies.)
	fboColor.allocate(COLOR_WIDTH, COLOR_HEIGHT, GL_RGB); //setup offscreen buffer in openGL RGB mode




	// NDI setup * * * * * * * * * * * * * 
	// NDI setup * * * * * * * * * * * * * 
	// NDI setup * * * * * * * * * * * * * 

	cout << "NDI SDK copyright NewTek (http:\\NDI.NewTek.com)" << endl;
	// Set the dimensions of the sender output here
	// This is independent of the size of the display window
	//senderWidth = 1920; // HD	-	PBO 150fps / 120fps Async/Sync unclocked
	//senderHeight = 1080; //			FBO  80fps /  75fps Async/Sync unclocked


	// Optionally set NDI asynchronous sending instead of clocked at 60fps
	ndiSender1.SetAsync(false); // change to true for async
	ndiSender2.SetAsync(false); // change to true for async
	ndiSender3.SetAsync(false); // change to true for async

	int senderWidth;
	int senderHeight;

	//==================================================
	//==================================================
	senderWidth = COLOR_WIDTH;
	senderHeight = COLOR_HEIGHT;

	// Initialize ofPixel buffers
	color_ndiBuffer[0].allocate(senderWidth, senderHeight, 4);
	color_ndiBuffer[1].allocate(senderWidth, senderHeight, 4);

	//// Create a new sender
	color_StreamName = "kv2_color";
	strcpy(senderName, color_StreamName.c_str());
	ndiSender1.CreateSender(senderName, senderWidth, senderHeight, NDIlib_FourCC_type_RGBA); //// Specify RGBA format here
	cout << "Created NDI sender [" << senderName << "] (" << senderWidth << "x" << senderHeight << ")" << endl;
	color_idx = 0; // index used for buffer swapping


	//==================================================
	//==================================================
	senderWidth = DEPTH_WIDTH;
	senderHeight = DEPTH_HEIGHT;

	// Initialize ofPixel buffers
	cutout_ndiBuffer[0].allocate(senderWidth, senderHeight, 4);
	cutout_ndiBuffer[1].allocate(senderWidth, senderHeight, 4);

	// Create a new sender
	cutout_StreamName = "kv2_cutout";
	strcpy(senderName, cutout_StreamName.c_str());
	ndiSender2.CreateSender(senderName, senderWidth, senderHeight, NDIlib_FourCC_type_RGBA); //// Specify RGBA format here
	cout << "Created NDI sender [" << senderName << "] (" << senderWidth << "x" << senderHeight << ")" << endl;
	cutout_idx = 0; // index used for buffer swapping


	//==================================================
	//==================================================
	senderWidth = DEPTH_WIDTH;
	senderHeight = DEPTH_HEIGHT;

	// Initialize ofPixel buffers
	infrared_ndiBuffer[0].allocate(senderWidth, senderHeight, 4);
	infrared_ndiBuffer[1].allocate(senderWidth, senderHeight, 4);

	// Create a new sender
	infrared_StreamName = "kv2_infrared";
	strcpy(senderName, infrared_StreamName.c_str());
	ndiSender3.CreateSender(senderName, senderWidth, senderHeight, NDIlib_FourCC_type_RGBA); //// Specify RGBA format here
	cout << "Created NDI sender [" << senderName << "] (" << senderWidth << "x" << senderHeight << ")" << endl;
	infrared_idx = 0; // index used for buffer swapping




	//NDI SENDER 1 colorSize============================
	//==================================================
	// Initialize OpenGL pbos for asynchronous read of fbo data

	senderWidth = COLOR_WIDTH; // DUSX resetting to work with color_ndi
	senderHeight = COLOR_HEIGHT;
	glGenBuffers(2, ndiPbo1);
	glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, ndiPbo1[0]);
	glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, senderWidth*senderHeight * 4, 0, GL_STREAM_READ);
	glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, ndiPbo1[1]);
	glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, senderWidth*senderHeight * 4, 0, GL_STREAM_READ);
	glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);
	Pbo1Index = NextPbo1Index = 0;
	bUsePBO1 = true; // Change to false to compare  // DUSX was originally true



	//NDI SENDER 2 depthSize============================
	//==================================================
	// Initialize OpenGL pbos for asynchronous read of fbo data

	senderWidth = DEPTH_WIDTH; // DUSX resetting to work with color_ndi
	senderHeight = DEPTH_HEIGHT;
	glGenBuffers(2, ndiPbo2);
	glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, ndiPbo2[0]);
	glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, senderWidth*senderHeight * 4, 0, GL_STREAM_READ);
	glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, ndiPbo2[1]);
	glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, senderWidth*senderHeight * 4, 0, GL_STREAM_READ);
	glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);
	Pbo2Index = NextPbo2Index = 0;
	bUsePBO2 = true; // Change to false to compare  // DUSX was originally true
	
	// NDI setup DONE ^ ^ ^ * * * * * * * * * *
	// NDI setup DONE ^ ^ ^ * * * * * * * * * *
	// NDI setup DONE ^ ^ ^ * * * * * * * * * *




	gui.setup("Parameters", "settings.xml");
	//ofColor paramBG(0, 255, 0);
	gui.setHeaderBackgroundColor(ofColor::darkRed);
	gui.setBorderColor(ofColor::darkRed);
	//gui.setBackgroundColor(ofColor::darkRed);
	//gui.setFillColor(ofColor::darkRed);

	OSCgroup.setup("OSC");
	OSCgroup.add(jsonGrouped.setup("OSC as JSON", true));
	OSCgroup.add(HostField.setup("OSC host ip", "localhost"));
	OSCgroup.add(oscPort.setup("OSC port", 1234));
	gui.add(&OSCgroup);
	
	SPOUTgroup.setup("Spout");
	SPOUTgroup.add(spoutCutOut.setup("BnW cutouts -> spout", true));
	SPOUTgroup.add(spoutColor.setup("Color -> spout", true));
	SPOUTgroup.add(spoutKeyed.setup("Keyed -> spout", true));
	SPOUTgroup.add(spoutDepth.setup("Depth -> spout", true));
	gui.add(&SPOUTgroup);

	gui.loadFromFile("settings.xml");
	
	// HostField.addListener(this, &ofApp::HostFieldChanged);

	oscSender.disableBroadcast();
	oscSender.setup(HostField, oscPort);
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
	if (spoutKeyed) {
		for (int y = 0; y < DEPTH_HEIGHT; y++) {
			for (int x = 0; x < DEPTH_WIDTH; x++) {
				int index = (y * DEPTH_WIDTH) + x;

				ofColor trans(0, 0, 0, 0);
				foregroundImg.setColor(x, y, trans);

				// This is the check to see if a given pixel is inside a tracked  body or part of the background.
				// If it's part of a body, the value will be that body's id (0-5), or will > 5 if it's
				// part of the background
				// More info here: https://msdn.microsoft.com/en-us/library/windowspreview.kinect.bodyindexframe.aspx
				float val = bodyIndexPix[index];
				if (val >= bodies.size()) {
					continue; // exit for loop without executing the following code
				}

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
	}

	// Update the images since we manipulated the pixels manually. This uploads to the
	// pixel data to the texture on the GPU so it can get drawn to screen
	foregroundImg.update();

	//--
	//Getting joint positions (skeleton tracking)
	//--
	//

	 /******************  ENUM copied from kinectv2 addon
	  JointType_SpineBase = 0,
		JointType_SpineMid = 1,
		JointType_Neck = 2,
		JointType_Head = 3,
		JointType_ShoulderLeft = 4,
		JointType_ElbowLeft = 5,
		JointType_WristLeft = 6,
		JointType_HandLeft = 7,
		JointType_ShoulderRight = 8,
		JointType_ElbowRight = 9,
		JointType_WristRight = 10,
		JointType_HandRight = 11,
		JointType_HipLeft = 12,
		JointType_KneeLeft = 13,
		JointType_AnkleLeft = 14,
		JointType_FootLeft = 15,
		JointType_HipRight = 16,
		JointType_KneeRight = 17,
		JointType_AnkleRight = 18,
		JointType_FootRight = 19,
		JointType_SpineShoulder = 20,
		JointType_HandTipLeft = 21,
		JointType_ThumbLeft = 22,
		JointType_HandTipRight = 23,
		JointType_ThumbRight = 24,
		JointType_Count = (JointType_ThumbRight + 1)
		*/


	// shorten names to minimize packet size
	const char * jointNames[] = { "SpineBase", "SpineMid", "Neck", "Head",
		"ShldrL", "ElbowL", "WristL", "HandL",
		"ShldrR", "ElbowR", "WristR", "HandR",
		"HipL", "KneeL", "AnkleL", "FootL",
		"HipR", "KneeR", "AnkleR", "FootR",
		"SpineShldr", "HandTipL", "ThumbL", "HandTipR", "ThumbR", "Count" };

	/* MORE joint. values >>>
	 second. positionInWorld[] x y z , positionInDepthMap[] x y
	 second. orientation. _v[] x y z w  ??what is this
	 second. trackingState
	 */

	 /* MORE body. values >>>
	 body. tracked (bool)
	 body. leftHandState (_Handstate) enum?
	 body. rightHandState (_Handstate)
	 body. activity  ??what is this
	 */

	// defined in new coordmap section as &
	//	auto bodies = kinect.getBodySource()->getBodies();


	if (jsonGrouped) {
		body2JSON(bodies, jointNames);
	}else{
		// TODO:: seperate function and add additional features like hand open/closed
		// NON JSON osc messages
			for (auto body : bodies) {
				for (auto joint : body.joints) {
					auto pos = joint.second.getPositionInWorld();
					ofxOscMessage m;
					string adrs = "/" + to_string(body.bodyId) + "/" + jointNames[joint.first];
					m.setAddress(adrs);
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
		//// Note that for this we need a reference of which joints are connected to each other.
		//// We call this the 'boneAtlas', and you can ask for a reference to this atlas whenever you like
		//auto bodies = kinect.getBodySource()->getBodies();
		//auto boneAtlas = ofxKinectForWindows2::Data::Body::getBonesAtlas();

		//for (auto body : bodies) {
		//	for (auto bone : boneAtlas) {
		//		auto firstJointInBone = body.joints[bone.first];
		//		auto secondJointInBone = body.joints[bone.second];

		//		//now do something with the joints
		//	}
		//}
	}

	//
	//--
}


//--------------------------------------------------------------
void ofApp::draw() {
	stringstream ss;

	ofClear(0, 0, 0);
	bgCB.draw(0, 0, ofGetWidth(), ofGetHeight());

	// Color is at 1920x1080 instead of 512x424 so we should fix aspect ratio
	float colorHeight = previewWidth * (kinect.getColorSource()->getHeight() / kinect.getColorSource()->getWidth());
	float colorTop = (previewHeight - colorHeight) / 2.0;

	{
		// Draw Depth Source
		// TODO: brighten depth image. https://github.com/rickbarraza/KinectV2_Lessons/tree/master/3_MakeRawDepthBrigther
		// MORE: https://forum.openframeworks.cc/t/kinect-v2-pixel-depth-and-color/18974/4
		//kinect.getDepthSource()->draw(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);  // note that the depth texture is RAW so may appear dark

		fboDepth.begin(); // start drawing to off screenbuffer
		ofClear(255, 255, 255, 0);
		kinect.getDepthSource()->draw(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);
		fboDepth.end();
		//Spout
		if (spoutDepth) {
			spout.sendTexture(fboDepth.getTextureReference(), "kv2_depth");
		}
		//Draw from FBO
		fboDepth.draw(0, 0, previewWidth, previewHeight);
		//fboDepth.clear();
	}

	{
		// Draw Color Source
		
		fboColor.begin(); // start drawing to off screenbuffer
		ofClear(255, 255, 255, 0);
		kinect.getColorSource()->draw(0, 0, COLOR_WIDTH, COLOR_HEIGHT);
		fboColor.end();
		//Spout
		if (spoutColor) {
			spout.sendTexture(fboColor.getTextureReference(), color_StreamName);
		}

		//NDI
		// Set the sender name
		strcpy(senderName, color_StreamName.c_str()); // convert from std string to cstring
		sendNDI(ndiSender1, fboColor, bUsePBO1, COLOR_WIDTH, COLOR_HEIGHT, senderName, color_ndiBuffer, color_idx);

		//Draw from FBO to UI
		fboColor.draw(previewWidth, 0 + colorTop, previewWidth, colorHeight);
		//fboColor.clear();
	}

	{
		// Draw IR Source
		kinect.getInfraredSource()->draw(0, previewHeight, DEPTH_WIDTH, DEPTH_HEIGHT);
		//kinect.getLongExposureInfraredSource()->draw(0, previewHeight, previewWidth, previewHeight);

		// NDI
		// Set the sender name
		strcpy(senderName, infrared_StreamName.c_str()); // convert from std string to cstring
		sendNDI(ndiSender3, fboDepth, bUsePBO2, DEPTH_WIDTH, DEPTH_HEIGHT, senderName, infrared_ndiBuffer, infrared_idx);
	}

	{
		// Draw B+W cutout of Bodies
		fboDepth.begin(); // start drawing to off screenbuffer
		ofClear(255, 255, 255, 0);
		kinect.getBodyIndexSource()->draw(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);
		fboDepth.end();
		//Spout
		if (spoutCutOut) {
			spout.sendTexture(fboDepth.getTextureReference(), "kv2_cutout");
		}

		// NDI
		// Set the sender name
		strcpy(senderName, cutout_StreamName.c_str()); // convert from std string to cstring
		sendNDI(ndiSender2, fboDepth, bUsePBO2, DEPTH_WIDTH, DEPTH_HEIGHT, senderName, cutout_ndiBuffer, cutout_idx);

		//Draw from FBO
		fboDepth.draw(previewWidth, previewHeight, previewWidth, previewHeight);
		//fboDepth.clear();
	}

	{
		// greenscreen/keyed fx from coordmaping
		//fboDepth.clear();
		//ofClear(255, 255, 255, 0);
		fboDepth.begin(); // start drawing to off screenbuffer
		ofClear(255, 255, 255, 0);
		foregroundImg.draw(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);
		fboDepth.end();
		//Spout
		if (spoutKeyed) {
			//ofSetFrameRate(30);
			spout.sendTexture(fboDepth.getTextureReference(), "kv2_keyed");
			//Draw from FBO, removed if not checked
			ofEnableBlendMode(OF_BLENDMODE_ALPHA);
			fboDepth.draw(previewWidth * 2, 0, previewWidth, previewHeight);
		}
		else {
			//ofSetFrameRate(60);
			ss.str("");
			ss << "Keyed image only shown when" << endl;
			ss << "checked in Parameters window" << endl;
			ss << "and, a body is being tracked.";
			ofDrawBitmapStringHighlight(ss.str(), previewWidth * 2 + 20, previewHeight - (previewHeight / 2 + 60));
		}
	}

	{
		// Draw bodies joints+bones over
		kinect.getBodySource()->drawProjected(previewWidth * 2, previewHeight, previewWidth, previewHeight, ofxKFW2::ProjectionCoordinates::DepthCamera);
	}

	ss.str("");
	ss << "fps : " << ofGetFrameRate();
	if (!bHaveAllStreams) ss << endl << "Not all streams detected!";
	ofDrawBitmapStringHighlight(ss.str(), 20, previewHeight * 2 - 20);

	ss.str("");
	ss << "Keyed FX : cpu heavy";
	ofDrawBitmapStringHighlight(ss.str(), previewWidth * 2 + 20, 20);

	ss.str("");
	ss << "Color : HD 1920x1080";
	ofDrawBitmapStringHighlight(ss.str(), previewWidth + 20, 20);

	ss.str("");
	ss << "BnW : body outlines";
	ofDrawBitmapStringHighlight(ss.str(), previewWidth + 20, previewHeight + 20);

	ss.str("");
	ss << "Bodies : coordinates -> OSC" << endl;
	ss << "Tracked bodies: " << numBodiesTracked;
	ofDrawBitmapStringHighlight(ss.str(), previewWidth * 2 + 20, previewHeight + 20);

	ss.str("");
	ss << "Depthmap : ";
	ofDrawBitmapStringHighlight(ss.str(), 20, 20);

	ss.str("");
	ss << "Infrared : ";
	ofDrawBitmapStringHighlight(ss.str(), 20, previewHeight + 20);

	gui.draw();
}

void ofApp::exit() {
	gui.saveToFile("settings.xml");
	if (ndiPbo1[0]) glDeleteBuffers(2, ndiPbo1); // clean up NDI_1 - HD
	if (ndiPbo2[0]) glDeleteBuffers(2, ndiPbo2); // clean up NDI_2 - DepthsSize
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
void ofApp::HostFieldChanged() {
	cout << "fieldChange" << endl;
	oscSender.disableBroadcast();
	oscSender.setup(HostField, oscPort);
	cout << "updated" << endl;
}
void ofApp::body2JSON(vector<ofxKinectForWindows2::Data::Body> bodies, const char * jointNames[]) {
	// TODO: create factory
	for (auto body : bodies) {
		string bdata = ""; // start JSON array build of body data
		string newData = ""; // start JSON array build of joints data
		for (auto joint : body.joints) {
			auto pos = joint.second.getPositionInWorld();
			string name = jointNames[joint.first];
			newData = "\"j\":";  // j for joint ;)
			newData = newData + "\"" + name + "\",";
			newData = newData + "\"x\":" + to_string(pos.x) + ",";
			newData = newData + "\"y\":" + to_string(pos.y) + ",";
			newData = newData + "\"z\":" + to_string(pos.z);
			newData = "{" + newData + "}";
			// format= {"\joint\":\"jointName\",\"x\":0.1,\"y\":0.2,\"z\":0.3 }
			if (bdata == "") {  // if bdata = "" no comma
				bdata = newData;
			}
			else {
				bdata = bdata + "," + newData;
			}
		} // end inner joints loop

		  // format= {"\joint\":\"jointName\",\"x\":0.1,\"y\":0.2,\"z\":0.3 }
		  // {"j":"SpineBase","x":-0.102359,"y":-0.669035,"z":1.112273}

		  // TODO: add below features to non Json OSC
		  // body.activity ?? contains more.. worth looking into 
		newData = "\"LH-state\":" + to_string(body.leftHandState);
		newData = "{" + newData + "}";
		// if tracked add ',' and bdata, otherwise bdata = newData. Fixes trailing ',' for non tracked bodies
		if (!body.tracked) {
			bdata = newData;
		}
		else {
			bdata = newData + "," + bdata;
		}

		newData = "\"RH-state\":" + to_string(body.rightHandState);
		newData = "{" + newData + "}";
		bdata = newData + "," + bdata;

		newData = "\"trackingID\":" + to_string(body.trackingId);
		newData = "{" + newData + "}";
		bdata = newData + "," + bdata;

		newData = "\"tracked\":" + to_string(body.tracked);
		newData = "{" + newData + "}";
		bdata = newData + "," + bdata;

		// need to escape all " in bdata
		bdata = escape_quotes(bdata);
		bdata = "[" + bdata + "]";
		bdata = "{\"b" + to_string(body.bodyId) + "\": \"" + bdata + "\"}";
		//cout << bdata << endl;
		ofxOscMessage m;
		string adrs = "/kV2/body/" + to_string(body.bodyId);
		m.setAddress(adrs);
		m.addStringArg(bdata);
		oscSender.sendMessage(m);
	} // end body loop
}

// NDI
// makes sending more modular
void ofApp::sendNDI(ofxNDIsender & ndiSender_, ofFbo & sourceFBO_, 
	bool bUsePBO_, int senderWidth_, int senderHeight_, char senderName_[256], ofPixels ndiBuffer_[], int idx_)
{
	if (ndiSender_.GetAsync())
		idx_ = (idx_ + 1) % 2;

	// Extract pixels from the fbo.
	if (bUsePBO_) {
		// Read fbo using two pbos
		// if (&ndiSender == &ndiSender1) {}
			ReadFboPixels(sourceFBO_, senderWidth_, senderHeight_, ndiBuffer_[idx_].getPixels());
		
	}
	else {
		// Read fbo directly
		sourceFBO_.bind();
		glReadPixels(0, 0, senderWidth_, senderHeight_, GL_RGBA, GL_UNSIGNED_BYTE, ndiBuffer_[idx_].getPixels());
		sourceFBO_.unbind();
	}

	// Send the RGBA ofPixels buffer to NDI
	// If you did not set the sender pixel format to RGBA in CreateSender
	// you can convert to bgra within SendImage (specify true for bSwapRB)
	if (ndiSender_.SendImage(ndiBuffer_[idx_].getPixels(), senderWidth_, senderHeight_)) {
		// Show what it is sending
		char str[256];
		sprintf(str, "Sending as : [%s] (%dx%d)", senderName, senderWidth_, senderHeight_);
		ofDrawBitmapString(str, 20, 50);
		// Show fps
		sprintf(str, "fps: %3.3d", (int)ofGetFrameRate());
		ofDrawBitmapString(str, 20, 70); // ofGetWidth() - 120
	}
}

// NDI
// Asynchronous Read-back
//
// adapted from : http://www.songho.ca/opengl/gl_pbo.html
//
bool ofApp::ReadFboPixels(ofFbo fbo, unsigned int width, unsigned int height, unsigned char *data)
{
	void *pboMemory;

	if (width == COLOR_WIDTH) {
		// dealing with HD size video
		Pbo1Index = (Pbo1Index + 1) % 2;
		NextPbo1Index = (Pbo1Index + 1) % 2;

		// Bind the fbo passed in
		fbo.bind();

		// Set the target framebuffer to read
		glReadBuffer(GL_FRONT);

		// Bind the current PBO
		glBindBuffer(GL_PIXEL_PACK_BUFFER, ndiPbo1[Pbo1Index]);

		// Read pixels from framebuffer to the current PBO - glReadPixels() should return immediately.
		//glReadPixels(0, 0, width, height, GL_BGRA_EXT GL_RGBA, GL_UNSIGNED_BYTE, (GLvoid *)0);
		// Send RGBA
		glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, (GLvoid *)0);

		// Map the previous PBO to process its data by CPU
		glBindBuffer(GL_PIXEL_PACK_BUFFER, ndiPbo1[NextPbo1Index]);
	}
	else {
		// dealing with depth size video
		Pbo2Index = (Pbo2Index + 1) % 2;
		NextPbo2Index = (Pbo2Index + 1) % 2;
		fbo.bind();
		glReadBuffer(GL_FRONT);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, ndiPbo2[Pbo2Index]);
		glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, (GLvoid *)0);
		glBindBuffer(GL_PIXEL_PACK_BUFFER, ndiPbo2[NextPbo2Index]);
	}


	pboMemory = glMapBuffer(GL_PIXEL_PACK_BUFFER, GL_READ_ONLY);
	if (pboMemory) {
		// Use SSE2 mempcy
		ofxNDIutils::CopyImage((unsigned char *)pboMemory, data, width, height, width * 4);
		glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
	}
	else {
		glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
		fbo.unbind();
		return false;
	}

	// Back to conventional pixel operation
	glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
	fbo.unbind();

	return true;
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
void ofApp::gotMessage(ofMessage msg) {

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
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
