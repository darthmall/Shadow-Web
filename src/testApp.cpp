#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
    ofSetVerticalSync(true);
	ofBackgroundHex(0xffffff);
	ofSetLogLevel(OF_LOG_NOTICE);
    
	box2d.init();
	box2d.setGravity(0, 10);
	box2d.setFPS(30.0);
	box2d.registerGrabbing();
    
    ofVec2f *anchors[4] = {
        new ofVec2f(0, ((float) ofGetHeight()) / 2.f),
        new ofVec2f(((float) ofGetWidth()) / 2.f, 0),
        new ofVec2f(((float) ofGetWidth()), ((float) ofGetHeight()) / 2.f),
        new ofVec2f(((float) ofGetWidth()) / 2.f, ((float) ofGetHeight()))
    };
    
    cursor.setup(box2d.getWorld(), ofGetMouseX(), ofGetMouseY(), 20);
    
    //    center.setup(box2d.getWorld(), ofGetWidth() / 2, ofGetHeight() / 2, 40);
    
    for (int i = 0; i < 4; i++) {
        ofVec2f *a = anchors[i];
        ofVec2f *b = anchors[(i + 1 < 4) ? i + 1 : 0];
        
        for (int j = 0; j < 20; j++) {
            ofxBox2dCircle circle;
            ofVec2f pos;
            pos.x = a->x + (j * ((b->x - a->x) / 20));
            pos.y = a->y + (j * ((b->y - a->y) / 20));
            
            if (pos.x > 0 && pos.x < ofGetWidth() && pos.y > 0 && pos.y < ofGetHeight()) {
                circle.setPhysics(0.1, 0.53, 0.1);
            } 
            
            circle.setup(box2d.getWorld(), pos.x, pos.y, 4);
            
            circles.push_back(circle);
        }
    }
    
	// now connect each circle with a joint
	for (int i=0; i<circles.size(); i++) {
		
		ofxBox2dJoint joint;
        
        ofxBox2dCircle c1 = circles[i];
        ofxBox2dCircle c2 = circles[(i + 1) < circles.size() ? i + 1 : 0];
        joint.setup(box2d.getWorld(), c1.body, c2.body, 4.f);
        
		joint.setLength(c1.getPosition().distance(c2.getPosition()) * 0.8);
		joints.push_back(joint);
	}
    
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();
	
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
}

//--------------------------------------------------------------
void testApp::update() {
	ofBackground(255, 255, 255);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}

    cursor.setPosition(ofGetMouseX(), ofGetMouseY());
	box2d.update();
}

//--------------------------------------------------------------
void testApp::draw() {
    for(int i=0; i<joints.size(); i++) {
		ofSetHexColor(0x444342);
		joints[i].draw();
	}
	
    // draw from the live kinect
    kinect.drawDepth(10, 10, 400, 300);
    
    grayImage.draw(10, 320, 400, 300);
    contourFinder.draw(10, 320, 400, 300);
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
