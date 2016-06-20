#pragma once

#include "ofMain.h"
#include "ofxOsc.h"

#define PORT 12345
#define HOST "localhost"
#define TX 12346

const string joints[] = {
    "righthand",
    "lefthand",
    "rightelbow",
    "leftelbow",
    "rightfoot",
    "leftfoot",
    "rightknee",
    "leftknee",
    "head",
    "torso"
};

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();

	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
    
    ofxOscReceiver receiver;
    ofxOscSender sender;
    vector<ofVec3f> limbs;
};
