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

#define NUM_SPHERES 220

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
    
    of->addAntiGravityField( Vec3(0,0,0), 1000.0, Vec3(0,0,-5000), true );
    camNode->addChild( of->addAntiGravityField( Vec3(0,0,200), 200.0, Vec3(0,0,50), true ) );
    
    // seesaw
    camNode->addChild( of->addSeesaw( Vec3(500,0,350), Vec3(75,250,5), Vec4(0.0,1.0,0.0,1.0), true, true ) );
    camNode->addChild( of->addSeesaw( Vec3(-500,0,350), Vec3(75,250,5), Vec4(1.0,0.0,0.0,1.0), true, true ) );
    
    camNode->addChild( of->addBox( Vec3(0,0,-5000), Vec3(5000,5000,5000), Vec4(1.0,1.0,1.0,1.0), false, true ) );
    
    handBall = of->addCylinderHand( 5, 4000, Vec4(1,1,1,1) );
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
    if (frame == 120) {
      startSim = true;
      //camNode->addChild(of->addBox( Vec3((float) (rand() % 400 - 200), (float) (rand() % 400 - 200),500.), Vec3(50,50,50), Vec4(1,1,1,1), true, true ));
    }
    // Initialize Ball Pit over time
    Vec4 colorArray[4];
    colorArray[0] = Vec4(1.0,1.0,1.0,1.0);
    colorArray[1] = Vec4(1.0,0.0,0.0,1.0);
    colorArray[2] = Vec4(0.0,1.0,0.0,1.0);
    colorArray[3] = Vec4(0.0,0.0,1.0,1.0);
    
    static int numSpheres = 0;
    if (frame % 3 == 0 && startSim && numSpheres < NUM_SPHERES) {
      MatrixTransform* sphereMat = of->addSphere( Vec3((float) (rand() % 400 - 200), (float) (rand() % 400 - 200),500.), 35, colorArray[numSpheres%4], true, true );
      camNode->addChild( sphereMat );
      numSpheres++;
    }
    
    if (frame % 60 == 0) {
      std::cout << "FPS: " << 1.0/PluginHelper::getLastFrameDuration() << std::endl;
    }
    Matrixd os = PluginHelper::getObjectMatrix();
    os.invert_4x4(os);
    Matrixd handMat = PluginHelper::getHandMat(0) * os;
    of->updateHand( handMat );
      
    if (startSim) of->stepSim( PluginHelper::getLastFrameDuration() );
}

bool PhysicsLab::processEvent(InteractionEvent * event) {
    static bool grabbing = false;
    
    KeyboardInteractionEvent * kp;
    TrackedButtonInteractionEvent * he;
    if ((kp = event->asKeyboardEvent()) != NULL) {
        nh->keyEvent( kp );
    } else if ((he = event->asTrackedButtonEvent()) != NULL) {
      Matrixd os = PluginHelper::getObjectMatrix();
      Matrixd handMat = PluginHelper::getHandMat(0) * PluginHelper::getObjectToWorldTransform() * os * PluginHelper::getWorldToObjectTransform();
      if (he->getHand() == 0 && he->getButton() == 0) {
        if (he->getInteraction() == BUTTON_DOWN && !grabbing)
            grabbing = of->grabObject( handMat, camNode );
        else if (he->getInteraction() == BUTTON_UP) {
            grabbing = false;
            of->releaseObject();
        }
      }
    }

    return true;
}
