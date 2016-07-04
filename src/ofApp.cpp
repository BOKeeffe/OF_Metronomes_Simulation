#include "ofApp.h"

/**
	@file 		ofApp.cpp
	@author		Willaim O'Keeffe
	*/


using namespace YAMPE; using namespace P;

int j = 0;

//--------------------------------------------------------------
void ofApp::setup() {
    
    ofSetVerticalSync(true);
    
    // repatable randomness
    ofSeedRandom(10);
    
    // create a minimal gui
    gui.setup("Metronomes2");
    
    cameraHeightRatio.addListener(this, &ofApp::cameraHeightRatioChanged);
    guiParameters.setName("GUI");
    guiParameters.add(isGuiVisible.set("GUI visible", true));
    guiParameters.add(isDebugVisible.set("Debug visible", true));
    guiParameters.add(isAxisVisible.set("Unit vectors", true));
    guiParameters.add(isXGridVisible.set("Grid (X-axis)", false));
    guiParameters.add(isYGridVisible.set("Grid (Y-axis)", true));
    guiParameters.add(isZGridVisible.set("Grid (Z-axis)", false));
    guiParameters.add(isGroundVisible.set("Ground", true));
    guiParameters.add(Spring.set("Set Spring", false));
    guiParameters.add(amtOfPendulem.set("Amount of Pendulems",0.5 ,0, 200));
    resetButton.addListener(this, &ofApp::reset);
    guiParameters.add(cameraHeightRatio.set("Camera (height)", 0.1f, 0., 0.999f));
    
    gui.ofxGuiGroup::add(guiParameters);
    gui.add(resetButton.setup("Reset"));
    
    // simulation specific stuff goes here
    
    // load gui parameters from file
    gui.loadFromFile("settings.xml");
    
    // instantiate the ground
    ground.set(RANGE, RANGE);
    ground.rotate(90, 1,0,0);
    
    // lift camera to 'eye' level
    easyCam.setDistance(RANGE);
    float d = easyCam.getDistance();
    easyCam.setPosition(0, cameraHeightRatio*d, d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio));
    easyCam.setTarget(ofVec3f::zero());
    
    font.load(OF_TTF_SANS, 9, true, true);
    
    // simulation specific stuff goes here
    
    isForceVisible = false;
    
    contacts = ContactRegistry::Ref(new ContactRegistry(100, "All contacts"));

    // debug GUI
    
    // build text debug page
    debugTextPage.setup("Text");
    debugString = "";
    debugTextPage.add(&debugStringLabel);
    
    // build text debug page (using a panel to allow for multiple graphs)
    debugGraphicsPage.setup("Graphics");
    
    debugGraphicsPanel.setup("TODO");
    debugGraphicsPage.add(&debugGraphicsPanel);
    
    // finally build tabbed pages
    debugPages.setup("Debug", "", debugTextPage.getShape().getRight()+10);
    debugPages.setSize(500, 300);
    debugPages.add(&debugTextPage);
    debugPages.add(&debugGraphicsPage);
    
    // set position of debug GUI to bottom left corner
    debugPages.setPosition(0, ofGetHeight()-debugPages.getHeight());
    
}


void ofApp::reset() {
    
    // simulation specific stuff goes here
    particles.clear();
    anchoredPinConstraints.clear();
    pinPinConstraint.clear();
    pinBobConstraint.clear();
    forceGenerators.clear();
    ppContactGenerators.particles.clear();
    
    float targetLenght = 2.0f;
    float restitution = 0.0f;
    
    contacts = ContactRegistry::Ref(new ContactRegistry(100, "All Contacts"));
    ForceGenerator::Ref gravity(new GravityForceGenerator(ofVec3f(0,-4,0)));
    
    
    for (int i = 0; i < amtOfPendulem.get(); ++i){
        
        // Particle pin
        pin = Particle::Ref(new Particle());
        pin->setPosition(ofVec3f(i,0,0)).setBodyColor(ofColor::blue).setWireColor(ofColor::blue);
        particles.push_back(pin);
        forceGenerators.add(pin, gravity);
        groundContactGenerators.particles.push_back(pin);
        ppContactGenerators.particles.push_back(pin);
        
        //Particle bob
        Particle::Ref bob = Particle::Ref(new Particle());
        bob->setPosition(ofVec3f(i,0,0)).setBodyColor(ofColor::red)
        .setWireColor(ofColor::red)
        .setVelocity(ofVec3f(ofRandom(-1, 1),0,0));
        particles.push_back(bob);
        forceGenerators.add(bob, gravity);
        groundContactGenerators.particles.push_back(bob);
        ppContactGenerators.particles.push_back(bob);
        
        //Pin anchored equality constraints
        EqualityAnchoredConstraint::Ref eac(new EqualityAnchoredConstraint(pin, ofVec3f(i, 7, 0)));
        anchoredPinConstraints.push_back(eac);
        pinAnchor = pin->position + ofVec3f(i,7,0);
        
        //pin to pin equality constraints
        EqualityConstraint::Ref ec(new EqualityConstraint(pin, bob, targetLenght, restitution));
        pinBobConstraint.push_back(ec);
        std::cout << " array empty";
    
        //pin to pin constraints
        ec = EqualityConstraint::Ref(new EqualityConstraint(pin, pin, targetLenght, restitution));
        pinPinConstraint.push_back(ec);
    }

}

void ofApp::update() {
    
    float dt = ofClamp(ofGetLastFrameTime(), 0.01, 0.02);
    
    // simulation specific stuff goes here
    forceGenerators.applyForce(dt);
    
    // update all particles
    for (auto p: particles) p->integrate(dt);
    
    //Update constraints
    for(auto c: anchoredPinConstraints) (*c).generate(contacts);
    for(auto c: pinBobConstraint) (*c).generate(contacts);
    for(auto c: pinPinConstraint) (*c).generate(contacts);
    
    
    groundContactGenerators.generate(contacts);
    ppContactGenerators.generate(contacts);
    
    contacts->resolve(dt);
    contacts->clear();
}


void ofApp::draw() {
    
    ofEnableDepthTest();
    ofBackgroundGradient(ofColor(128), ofColor(0), OF_GRADIENT_BAR);
    
    ofPushStyle();
    easyCam.begin();
    
    ofDrawGrid(RANGE/(2*8), 8, false, isXGridVisible, isYGridVisible, isZGridVisible);
    
    if (isAxisVisible) ofDrawAxis(1);
    
    if (isGroundVisible) {
        ofPushStyle();
        ofSetHexColor(0xB87333);
        ground.draw();
        ofPopStyle();
    }
    
    // simulation specific stuff goes here
    foreach(p, particles) (**p).draw();
    
    //draw line from pin to anchor point;
    /*
    ofPolyline pendulum;
    pendulum.addVertex(pinAnchor);
    pendulum.addVertex(pin->position);
    pendulum.draw();
     */
    
    float count = 0;
    
    easyCam.end();
    ofPopStyle();
    
    // finally render any GUI/HUD
    ofDisableDepthTest();
    if (isGuiVisible) gui.draw();
    if (isDebugVisible) {
        
        debugStringLabel.setup("",
                               "FPS = " + ofToString((int)ofGetFrameRate()) + "\n\n"  + "k = = " + ofToString(amtOfPendulem) + debugString);
        debugPages.draw();
    }

    
}


//--------------------------------------------------------------
// GUI events and listeners
//--------------------------------------------------------------

void ofApp::keyPressed(int key) {
    
    float d = easyCam.getDistance();
    
    switch (key) {
            
        case 'h':                               // toggle GUI/HUD
            isGuiVisible = !isGuiVisible;
            break;
        case 'b':                               // toggle debug
            isDebugVisible = !isDebugVisible;
            break;
        case 'a':                               // toggle axis unit vectors
            isAxisVisible = !isAxisVisible;
            break;
        case '1':                               // toggle grids (X)
            isXGridVisible = !isXGridVisible;
            break;
        case '2':                               // toggle grids (Y)
            isYGridVisible = !isYGridVisible;
            break;
        case '3':                               // toggle grids (Z)
            isZGridVisible = !isZGridVisible;
            break;
        case 'g':                               // toggle ground
            isGroundVisible = !isGroundVisible;
            break;
        case 'u':                               // set the up vecetor to be up (ground to be level)
            easyCam.setTarget(ofVec3f::zero());
            break;
            
        case 'S' :                              // save gui parameters to file
            gui.saveToFile("settings.xml");
            
            break;
        case 'L' :                              // load gui parameters
            gui.loadFromFile("settings.xml");
            break;
            
        case 'z':
            easyCam.setPosition(0, cameraHeightRatio*d,
                                d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio));
            easyCam.setTarget(ofVec3f::zero());
            break;
        case 'Z':
            easyCam.setPosition(0, 0, d);
            easyCam.setTarget(ofVec3f::zero());
            break;
        case 'x':
            easyCam.setPosition(d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio), cameraHeightRatio*d, 0);
            easyCam.setTarget(ofVec3f::zero());
            break;
        case 'X':
            easyCam.setPosition(d, 0, 0);
            easyCam.setTarget(ofVec3f::zero());
            break;
        case 'Y':
            easyCam.setPosition(0.001, d, 0);
            easyCam.setTarget(ofVec3f::zero());
            break;
            
        case 'f':                               // toggle fullscreen
            // BUG: window size is not preserved
            isFullScreen = !isFullScreen;
            if (isFullScreen) {
                ofSetFullscreen(false);
            } else {
                ofSetFullscreen(true);
            }
            break;
            
            // simulation specific stuff goes here
            
    }
}

void ofApp::cameraHeightRatioChanged(float & cameraHeightRatio) {
    
    float d = easyCam.getDistance();
    easyCam.setPosition(0, cameraHeightRatio*d, d*sqrt(1.0f-cameraHeightRatio*cameraHeightRatio));
    easyCam.setTarget(ofVec3f::zero());
}


//--------------------------------------------------------------
// Unused
//--------------------------------------------------------------
void ofApp::keyReleased(int key) {}
void ofApp::mouseMoved(int x, int y ) {}
void ofApp::mouseDragged(int x, int y, int button) {}
void ofApp::mousePressed(int x, int y, int button) {}
void ofApp::mouseReleased(int x, int y, int button) {}
void ofApp::mouseEntered(int x, int y) {}
void ofApp::mouseExited(int x, int y) {}
void ofApp::windowResized(int w, int h) {}
void ofApp::gotMessage(ofMessage msg) {}
void ofApp::dragEvent(ofDragInfo dragInfo) {}
