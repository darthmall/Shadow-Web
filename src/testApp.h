#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxBox2d.h"
#include "ofxTriangle.h"
#include "ofxCvContourSimplify.h"

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
    void drawContours();
    void connect(ofxBox2dCircle a, ofxBox2dCircle b);
    void spin();
    void startContact(ofxBox2dContactArgs &e);
    
    vector<ofxBox2dCircle> anchors;
    ofxBox2dCircle center;

	ofxKinect kinect;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image

	ofxCvContourFinder contourFinder;
	
	bool bThreshWithOpenCV;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;
    
    ofxBox2d box2d;
    vector<ofxBox2dCircle> circles;
    vector<ofxBox2dJoint> joints;
    vector<ofxBox2dPolygon> body;
    
    ofxTriangle triangle;

    ofxCvContourSimplify contourSimp;
    vector<ofPoint> contourRough;
    vector<ofPoint> contourSmooth;

};
