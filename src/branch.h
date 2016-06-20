
#pragma once
#include "ofMain.h"

class branch {
private:
    float rotation;
    float width;
    
    vector<branch*> children;
    
public:
    branch(int depth, float width);
    void draw(bool start);
};