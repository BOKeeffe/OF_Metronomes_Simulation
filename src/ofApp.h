#pragma once

/**
	@file 		ofApp.h
	@author		Willaim O'Keeffe
    @brief      ofApp header file
	*/

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxGuiExtended.h"
#include "ofxXmlSettings.h"

#include "YAMPE/Particle/ForceGeneratorRegistry.h"
#include "YAMPE/Particle/ForceGeneratorRegistry.h"
#include "YAMPE/Particle/ContactRegistry.h"
#include "YAMPE/Particle/ContactGenerators.h"
#include "YAMPE/Particle/Constraints.h"

class ofApp : public ofBaseApp {
    
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
		
    // simple 3D world with ground and axes
    const float RANGE = 16;
    ofEasyCam easyCam;
    ofPlanePrimitive ground;
    
    // ofxGui code
    ofxPanelExtended gui;
    ofParameterGroup guiParameters;
    ofParameter<bool> isGuiVisible;
    ofParameter<bool> isDebugVisible;
    ofParameter<bool> isAxisVisible;
    ofParameter<bool> isXGridVisible;
    ofParameter<bool> isYGridVisible;
    ofParameter<bool> isZGridVisible;
    ofParameter<bool> isGroundVisible;
    ofParameter<bool> isFullScreen;
    ofParameter<bool> Spring;
    ofParameter<int> amtOfPendulem;
    ofParameter<std::string> position;
    
    ofParameter<float> cameraHeightRatio;
    void cameraHeightRatioChanged(float & cameraHeightRatio);

    void reset();
    ofxMinimalButton resetButton;

    ofTrueTypeFont font;
    
    // simulation specific stuff goes here
    
    // debug code
    ofxTabbedPages debugPages;
    ofxGuiPage debugTextPage, debugGraphicsPage;
    ofxPanelExtended debugGraphicsPanel;

    ofxLabel debugStringLabel;
    string debugString;

    // simulation specific stuff goes here (debug)

private:

    // or here
    
    bool isForceVisible;
    YAMPE::ParticleRegistry particles;
    YAMPE::Particle::Ref pin;
    ofVec3f pinAnchor;
   
    vector<YAMPE::P::EqualityAnchoredConstraint::Ref> anchoredPinConstraints;
    vector<YAMPE::P::EqualityConstraint::Ref> pinBobConstraint;
    vector<YAMPE::P::EqualityConstraint::Ref> pinPinConstraint;
    
    YAMPE::P::ForceGeneratorRegistry forceGenerators;
    YAMPE::P::ContactRegistry::Ref contacts;
    YAMPE::P::GroundContactGenerator groundContactGenerators;
    YAMPE::P::ParticleParticleContactGenerator ppContactGenerators;
    
   
};
