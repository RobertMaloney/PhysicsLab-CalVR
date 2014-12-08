#include "PhysicsLab.h"

#include <PluginMessageType.h>

#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>

// OSG:
#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Vec3d>
#include <osg/MatrixTransform>

#define NUM_SPHERES 10

using namespace std;
using namespace cvr;
using namespace osg;

CVRPLUGIN(PhysicsLab)

void setupScene( ObjectFactory* );

const double pi = 3.141593;
int frame = 0;
MatrixTransform * camNode, *boxMatrix, *seesawMatrix;
PositionAttitudeTransform * lightMat;
StateSet* lightSS;
int boxId, seesawid;
std::vector<int> sphereId;
std::vector<MatrixTransform*> sphereMatrix;
MatrixTransform* handBall;

// Constructor
PhysicsLab::PhysicsLab()
{
}

void PhysicsLab::menuCallback(MenuItem* menuItem)
{
  
}

// intialize
bool PhysicsLab::init()
{
  cerr << "PhysicsLab::PhysicsLab" << endl;

  _mainMenu = new SubMenu("PhysicsLab", "PhysicsLab");
  _mainMenu->setCallback(this);
  MenuSystem::instance()->addMenuItem(_mainMenu); 
  
  of = new ObjectFactory();
  nh = new NavHandler( of->getBulletHandler(), Vec3(0,0, 600) );
  setupScene( of );
  
  return true;
}

void setupScene( ObjectFactory * of ) {
    
    //navHandle = new NavigationHandler;
	  PluginHelper::setObjectScale(1000.0);
	  
	  camNode = new MatrixTransform;
	  Matrix cam0, cam1, cam2;
	  //cam0.makeRotate(90 * pi / 180, Vec3(1,0,0));
	  //cam1.makeRotate(-45 * pi / 180, Vec3(0,1,0));
	  cam2.makeTranslate(0,400,-800);
	  cam2 = cam0 * cam1 * cam2;
	  camNode->setMatrix(cam2);
	  PluginHelper::setObjectMatrix( cam2 );
    PluginHelper::getScene()->addChild( camNode );
    
    //camNode->addChild( of->addBox( Vec3(0,0,600), 150, false, true ) );
    
    // bounding box (invis)
    of->addHollowBox( Vec3(0,0,0), Vec3(1000,1000,1000), false, false );
    
    // open box (pit)
    camNode->addChild( of->addOpenBox( Vec3(0,0,101), Vec3(600,600,100), 20.0, false, true ) );
    
    // fill bounding box with spheres
    /*
    for( int i = 0; i < NUM_SPHERES; ++i) {
        //Vec3 sphereStart1(-500,-100,500);
        //Vec3 sphereStart3(-500,200,800);
        Vec3 sphereStart(
            ((float) (i/10%10) / 10) * 1000 - 500,
            ((float) (i%10) / 10) * 1000 - 500,
            (i/100) * 150 + 300
        );
        MatrixTransform* sphereMat = of->addSphere( sphereStart, 50, true, true );
        camNode->addChild( sphereMat );
        
        if (false) {
            sphereMat->addChild( of->addBox( Vec3(100,0,0), 50, false, true ) );
        }
    }
    */
    of->addAntiGravityField( Vec3(0,0,0), 1000.0, Vec3(0,0,-5000), true );
    //of->addAntiGravityField( Vec3(0,0,300), 300.0, Vec3(0,0,0), true );
    
    // seesaw
    //camNode->addChild( of->addSeesaw( Vec3(0,0,400), Vec3(150,500,10), Vec4(1.0,1.0,1.0,1.0), true, true ) );
    
    camNode->addChild( of->addBox( Vec3(0,0,-5000), Vec3(5000,5000,5000), Vec4(1.0,1.0,1.0,1.0), false, true ) );
    
    handBall = of->addCylinderHand( 5, 700, Vec4(1,1,1,1) );
    camNode->addChild( handBall );
    
    // Light 0
    lightSS = PluginHelper::getScene()->getOrCreateStateSet();
    camNode->addChild( of->addLight( Vec3(500,500,500), Vec4(0.8,0.2,0.2,1.0), Vec4(0.2,0.2,0.2,1.0), Vec4(0,0,0,1.0), lightSS ) );
    camNode->addChild( of->addLight( Vec3(-500,500,500), Vec4(0.2,0.8,0.2,1.0), Vec4(0.2,0.2,0.2,1.0), Vec4(0,0,0,1.0), lightSS ) );
    camNode->addChild( of->addLight( Vec3(-500,-500,500), Vec4(0.2,0.2,0.8,1.0), Vec4(0.2,0.2,0.2,1.0), Vec4(0,0,0,1.0), lightSS ) );
    camNode->addChild( of->addLight( Vec3(500,-500,500), Vec4(0.8,0.8,0.8,1.0), Vec4(0.2,0.2,0.2,1.0), Vec4(0,0,0,1.0), lightSS ) );
    
}

// this is called if the plugin is removed at runtime
PhysicsLab::~PhysicsLab()
{
    delete of;
    delete nh;
}

void PhysicsLab::preFrame()
{
	  
    frame = (frame + 1) % 720;
    static bool startSim = false;
    if (frame == 120) startSim = true;
    
    // Initialize Ball Pit over time
    Vec4 colorArray[4];
    colorArray[0] = Vec4(1.0,1.0,1.0,1.0);
    colorArray[1] = Vec4(1.0,0.0,0.0,1.0);
    colorArray[2] = Vec4(0.0,1.0,0.0,1.0);
    colorArray[3] = Vec4(0.0,0.0,1.0,1.0);
    
    static int numSpheres = 0;
    if (frame % 3 == 0 && startSim && numSpheres < NUM_SPHERES) {
      MatrixTransform* sphereMat = of->addSphere( Vec3((float) (rand() % 400 - 200), (float) (rand() % 400 - 200),500.), 50, colorArray[numSpheres%4], true, true );
      //MatrixTransform* sphereMat = of->addCylinder( Vec3((float) (rand() % 400 - 200), (float) (rand() % 400 - 200),500.), 25, 100, colorArray[numSpheres%4], true, true );
      camNode->addChild( sphereMat );
      numSpheres++;
    }
    
    // Stylus code
    Matrixd os = PluginHelper::getObjectMatrix();
    os.invert_4x4(os);
    Matrixd handMat = PluginHelper::getHandMat(0) * os;
    handMat.setTrans( handMat.getTrans() + handMat.getRotate()*Vec3(0,500,0) );
    //handBall->setMatrix(handMat*PluginHelper::getHeadMat());
    of->updateHand(handMat);
    //cout << "HandButtonMask: " << PluginHelper::getHandButtonMask(0) << endl;
    //of->updateButtonState( PluginHelper::getHandButtonMask(0) );
    of->updateButtonState( nh->getButton() );
    if (nh->getButton()) std::cout << "Stylus origin: " << handMat;
    
    if (frame % 60 == 0) {
      std::cout << "FPS: " << 1.0/PluginHelper::getLastFrameDuration() << std::endl;
    }
    
    if (startSim) of->stepSim( PluginHelper::getLastFrameDuration() );
}

bool PhysicsLab::processEvent(InteractionEvent * event) {
    static bool lightswitch = true;
    
    KeyboardInteractionEvent * kp;
    if ((kp = event->asKeyboardEvent()) != NULL) {
        nh->keyEvent( kp );
    }

    return false;
}
