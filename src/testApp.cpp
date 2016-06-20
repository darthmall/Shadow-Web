#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
    ofSetVerticalSync(true);
	ofBackground(255, 255, 255);
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    receiver.setup(PORT);
    sender.setup(HOST, TX);
}

//--------------------------------------------------------------
void testApp::update() {
    ofBackground(255, 255, 255);

    for (int i = 0; i < 10; i++) {
        ofxOscMessage keepalive;
        stringstream address;
        
        address << "/" << joints[i] << "_trackjointpos";
        
        keepalive.setAddress(address.str());
        keepalive.addIntArg(3);
        
        sender.sendMessage(keepalive);
    }
    
    limbs.clear();

    while (receiver.hasWaitingMessages()) {
        ofxOscMessage msg;
        receiver.getNextMessage(&msg);
        
        string address = msg.getAddress();
        ofLog(OF_LOG_NOTICE, address);

        if (address.find("pos_screen") != string::npos) {

            ofVec3f position;

            for (int i = 0; i < msg.getNumArgs(); i++) {
                switch (i) {
                    case 0:
                        position.x = msg.getArgAsFloat(i);
                        break;
                        
                    case 1:
                        position.y = msg.getArgAsFloat(i);
                        break;
                        
                    case 2:
                        position.z = msg.getArgAsFloat(i);
                        break;
                        
                    default:
                        break;
                }
            }
            
            limbs.push_back(position);
        }
    }
}

//--------------------------------------------------------------
void testApp::draw() {
    ofFill();
    ofSetColor(0, 0, 0);

    for (int i = 0; i < limbs.size(); i++) {
        ofVec3f position = limbs[i];
        
        ofEllipse(position.x, position.y, 20, 20);
    }
}

//--------------------------------------------------------------
void testApp::exit() {
    ofLog(OF_LOG_NOTICE, "Exiting");
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {

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
