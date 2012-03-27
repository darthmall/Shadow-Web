#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
    ofSetVerticalSync(true);
	ofBackground(255, 255, 255);
    ofSetLogLevel(OF_LOG_VERBOSE);
    
	box2d.init();
	box2d.setGravity(0, 1);
	box2d.setFPS(60.0);
	box2d.registerGrabbing();

    center.setup(box2d.getWorld(), ofGetWidth() / 2.f, ofGetHeight() / 2.f, 4);
    spin();
    
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	kinect.open();
	
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 255;
	farThreshold = 120;
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
        grayImage.mirror(false, true);
        
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
    
    // Stolen from IDEO
    triangle.clear();
    
    for (int i = 0; i < contourFinder.nBlobs; i++) {
		triangle.triangulate(contourFinder.blobs[i].pts, max( 3.0f, (float)contourFinder.blobs[i].pts.size()/12));
	}
    
    for (int i = body.size() - 1; i >= 0; i--) {
        box2d.world->DestroyBody(body[i].body);
    }
    
    body.clear();
    
    // Stolen from IDEO
	//Triangulate contour in order to add to box2d
	ofxTriangleData* tData;
	for (int i=triangle.triangles.size()-1; i>=0; i--) {
		
		tData = &triangle.triangles[i];
		
		ofxBox2dPolygon poly;
		
		ofPoint t1,t2,t3;
		
		t1.x=ofMap(tData->a.x, 0, grayImage.width, 0, ofGetWidth());
		t1.y=ofMap(tData->a.y, 0, grayImage.height, 0, ofGetHeight());
		t2.x = ofMap(tData->b.x, 0, grayImage.width, 0, ofGetWidth());
		t2.y=ofMap(tData->b.y, 0, grayImage.height, 0, ofGetHeight());
		
		t3.x = ofMap(tData->c.x, 0, grayImage.width, 0, ofGetWidth());
		t3.y = ofMap(tData->c.y, 0, grayImage.height, 0, ofGetHeight());
		
		poly.addVertex(t1.x,t1.y);
		poly.addVertex(t2.x,t2.y);
		poly.addVertex(t3.x,t3.y);
        
        poly.create(box2d.world);
		
        b2Filter filter;
        filter.categoryBits = 0x0004;
        filter.maskBits = 0x0002;
        filter.groupIndex = 1;        
        poly.setFilterData(filter);

        body.push_back(poly);
	}
    
	box2d.update();
    
    for (vector<ofxBox2dJoint>::iterator it = joints.begin(); it < joints.end(); it++) {
        if (it->getReactionForce(1.f).squareLength() > 100) {
            box2d.world->DestroyJoint(it->joint);
            it = joints.erase(it);
        }
    }
}

//--------------------------------------------------------------
void testApp::draw() {
    for(int i=0; i<joints.size(); i++) {
		ofSetHexColor(0x444342);
		joints[i].draw();
	}

    drawContours();
}

//--------------------------------------------------------------
void testApp::drawContours(){
	
	//contourFinder.drawAlt(0,0,ofGetWidth(),ofGetHeight());	
	
	int numBlobs = contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, true);
	
	for(int i=0;i<6;i++){
		
		
		if(numBlobs > 0){			
			
			
			for(int j=0;j<numBlobs;j++){
				
				//lets get out the contour data
				int length_of_contour = contourFinder.blobs[j].pts.size();
				
				//clear the old contours
				contourRough.clear();
				contourRough.assign(length_of_contour, ofPoint());
				contourSmooth.clear();
				contourSmooth.assign(length_of_contour, ofPoint());
				
				//lets make a copy for ourselves
				for(int k = 0; k < length_of_contour; k++){
					contourRough[k] = contourFinder.blobs[j].pts[k];
				}
				
				contourSimp.simplify(contourRough, contourSmooth, 0.001*i);
				
				
				glPushMatrix();
				
				ofSetLineWidth(0.25);
				
                ofEnableAlphaBlending();
				ofSetColor(0,0,0,76);
				ofBeginShape();
				
				for(int i = 0; i < contourSmooth.size(); i++){
					float xC = ofMap(contourSmooth[i].x, 0, 640, 0, ofGetWidth());
					float yC = ofMap(contourSmooth[i].y, 0, 480, 0, ofGetHeight());
					ofVertex(xC,yC);
				}
				
				ofEndShape(true);
                ofDisableAlphaBlending();
				glPopMatrix();
				
			}
		}
	}
}

void testApp::spin() {
    
    for (vector<ofxBox2dJoint>::reverse_iterator it = joints.rbegin(); it < joints.rend(); it++) {
        box2d.getWorld()->DestroyJoint(it->joint);
    }
    
    joints.clear();
    
    for (vector<ofxBox2dCircle>::reverse_iterator it = circles.rbegin(); it < circles.rend(); it++) {
        box2d.getWorld()->DestroyBody(it->body);
    }
    
    circles.clear();
    
    for (vector<ofxBox2dCircle>::reverse_iterator it = anchors.rbegin(); it < anchors.rend(); it++) {
        box2d.getWorld()->DestroyBody(it->body);
    }
    
    anchors.clear();
    
    for (int i = 0; i < 4; i++) {
        int num_anchors = (int) ofRandom(1, 5);
        
        for (int j = 0; j < num_anchors; j++) {
            ofxBox2dCircle circle;
        
            float x = 0;
            float y = 0;
            
            switch (i) {
                case 0:
                    y = ofRandom(ofGetHeight());
                    break;
                case 1:
                    x = ofRandom(ofGetWidth());
                    break;
                case 2:
                    x = ofGetWidth();
                    y = ofRandom(ofGetHeight());
                    break;
                case 3:
                    x = ofRandom(ofGetWidth());
                    y = ofGetHeight();
                    break;
            }

            circle.setup(box2d.getWorld(), x, y, 4);

            b2Filter filter;
            filter.categoryBits = 0x0002;
            filter.maskBits = 0x0004;
            filter.groupIndex = -2;
            circle.setFilterData(filter);

            anchors.push_back(circle);
        }
    }
    
    for (int i = 0; i < anchors.size(); i++) {
        connect(anchors[i], anchors[(i + 1 < anchors.size()) ? i + 1 : 0]);
        connect(anchors[i], center);
    }
    
    for (int i = 0; i < ofRandom(30, 50); i++) {
        ofxBox2dCircle a = circles[ofRandom(circles.size())];
        ofxBox2dCircle b;
        
        do {
            b = circles[ofRandom(circles.size())];
        } while (a.getPosition() == b.getPosition());
        
        connect(a, b);
    }

}

void testApp::connect(ofxBox2dCircle a, ofxBox2dCircle b) {
    vector<ofxBox2dCircle> thread;
    
    thread.push_back(a);
    for (int i = 0; i < 20; i++) {
        ofxBox2dCircle circle;
        ofVec2f pos;

        pos.x = a.getPosition().x + (i * ((b.getPosition().x - a.getPosition().x) / 20));
        pos.y = a.getPosition().y + (i * ((b.getPosition().y - a.getPosition().y) / 20));
        
        if (pos.x > 0 && pos.x < ofGetWidth() && pos.y > 0 && pos.y < ofGetHeight()) {
            circle.setPhysics(1, 0.53, 0);
        } 

        circle.setup(box2d.world, pos.x, pos.y, 10.f);
        
        b2Filter filter;
        filter.categoryBits = 0x0002;
        filter.maskBits = 0x0004;
        filter.groupIndex = -2;
        
        circle.setFilterData(filter);

        circles.push_back(circle);
        thread.push_back(circle);
    }
    
    thread.push_back(b);
    
    // now connect each circle with a joint
	for (int i = 0; i < thread.size() - 1; i++) {
		ofxBox2dJoint joint;
        
        ofxBox2dCircle c1 = thread[i];
        ofxBox2dCircle c2 = thread[i + 1];
        joint.setup(box2d.getWorld(), c1.body, c2.body, 10.f);
        
		joint.setLength(c1.getPosition().distance(c2.getPosition()) * 0.8);
		joints.push_back(joint);
	}
}
         
//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
    stringstream logstream;
    int joint_index;
    
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
            logstream << "far: " << farThreshold;
            ofLog(OF_LOG_NOTICE, logstream.str());
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
            logstream << "far: " << farThreshold;
            ofLog(OF_LOG_NOTICE, logstream.str());
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
            logstream << "near: " << nearThreshold;
            ofLog(OF_LOG_NOTICE, logstream.str());
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
            logstream << "near: " << nearThreshold;
            ofLog(OF_LOG_NOTICE, logstream.str());
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
        
        case 'r':
            spin();
            break;

        case 'q':
            joint_index = (int) ofRandom(joints.size());
            box2d.getWorld()->DestroyJoint(joints[joint_index].joint);
            joints.erase(joints.begin() + joint_index);
            break;
            
        case 't':
            ofToggleFullscreen();
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
