#### kinect2share
Share skeleton and video data from the Kinect V2 with Isadora and any other software which supports OSC (Open Sound Control) and Spout. 

![Screen Shot including Isadora sample file](https://raw.githubusercontent.com/rwebber/kinect2share/master/screenshots/sampleapp1.png)

Kinect2share is a *Windows only* OpenFrameworks app based on the example projects included in [Elliot Woods](https://github.com/elliotwoods) fantastic OpenFrameworks plugin, [ofxKinectForWindows2](https://github.com/elliotwoods/ofxKinectForWindows2).

The goal of this project is to make as many Kinect2 features available to creative platforms as possible using open standards including OSC (opensoundcontrol), and [Spout](http://spout.zeal.co/).

Specific care has been given to providing a demo file for use with the [Isadora creativity server](http://troikatronix.com/).
This demo file provides basic functional examples that Isadora users can build upon.
Currently the [Isadora version 2.5 beta](http://troikatronix.com/isa25beta/) is required to run the demo file.

## Features
### Video
Supported (provided via Spout):
- Black and White Mask (512 x 424)
- Color (1920 x 1080)
- Keyed (512 x 424) *CPU process

### OSC : Skeleton
Supported (provided via Spout):
- Body data as JSON
- Body elements as individual OSC addresses

### License
MIT License http://en.wikipedia.org/wiki/MIT_License

#### TODO
- [video] Infrared
- [video] Long Exposure Infrared
- ~~[video] Depth~~
- [video] Keyed via GLSL
- ~~[OSC] add body features (eg hands open/closed)~~
- [video] add option to adjust levels for depth image / IR images

## Notes
1. The release version requires the Kinect V2 SDK. https://www.microsoft.com/en-us/download/details.aspx?id=44561
2. To work with the code, you'll need the Kinect v2 SDK and the NewTek NDI sdk http://pages.newtek.com/NDI-Developers.html

Additional ofx addons are used:
- ofxGui (create parameters gui) *core addon
- ofxOsc (shares skeleton data) *core addon
- ofxInputField (adds text field for editing of OSC port & ip) https://github.com/fx-lange/ofxInputField
- ofxSpout2 (shares video with other applications) https://github.com/Kj1/ofxSpout2



