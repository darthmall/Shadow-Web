//
//  branch.cpp
//  Shadow World
//
//  Created by Evan Sheehan on 4/12/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#include "branch.h"

branch::branch(int depth, float width) {
    stringstream ss;
    rotation = ofRandom(-90, 90);
    this->width = width;
    
    if (depth > 0) {
        for (int i = 0; i < ofRandom(1, 3); i++) {
            branch *twig = new branch(depth - 1, width * 0.5);
            children.push_back(twig);
        }
    }
}

void branch::draw(bool start) {
    ofPushMatrix();
    
    ofSetLineWidth(width);
    ofNoFill();
    ofSetHexColor(0x000000);
    ofTranslate(0, -ofGetHeight() / 5);
    if (!start) {
        ofRotate(rotation);
    }
    ofScale(0.7, 0.7);
    ofLine(0, 0, 0, -ofGetHeight() / 5);
    
    for (int i = 0; i < children.size(); i++) {
        children[i]->draw(false);
    }
    
    ofPopMatrix();
}
