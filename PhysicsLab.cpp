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

#define NUM_SPHERES 5

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

// Nav
bool goLeft = false, goRight = false, goUp = false, goDown = false;

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
  //nh = new NavHandler( of->getBulletHandler(), Vec3(0,0, 600) );
  setupScene( of );
  
  return true;
}

void setupScene( ObjectFactory * of ) {
    
    //navHandle = new NavigationHandler;
	  PluginHelper::setObjectScale(1000.0);
	  std::cout << PluginHelper::getObjectMatrix();
	  std::cout << PluginHelper::getHeadMat(0);
	  camNode = new MatrixTransform;
	  //camNode = PluginHelper::getScene();
	  Matrix cam0, cam1, cam2;
	  cam0.makeRotate(-30 * pi / 180, Vec3(1,0,0));
	  cam2.makeTranslate(0,800,-1200);
	  cam2 = cam0 * cam1 * cam2;
	  camNode->setMatrix(cam2);
    PluginHelper::getScene()->addChild( camNode );
    
    //camNode->addChild( of->addBox( Vec3(0,0,600), 150, false, true ) );
    
    // bounding box (invis)
    of->addHollowBox( Vec3(0,0,0), Vec3(2000,2000,5000), false, false );
    
    // open box (pit)
    camNode->addChild( of->addOpenBox( Vec3(0,0,101), Vec3(2000,2000,100), 50.0, false, true ) );
    
    // World Gravity, Elevator, then Funnel
    of->addAntiGravityField( Vec3(0,0,0), Vec3(10000,10000,10000), Vec3(0,0,-5000), true );
    camNode->addChild( of->addAntiGravityField( Vec3(-1850,0,500), Vec3(100,100,450), Vec3(0,0,500), true ) );
    camNode->addChild( of->addAntiGravityField( Vec3(100,0,100), Vec3(1850,100,50), Vec3(-5000,0,-5000), true) );
    
    // seesaw
    //camNode->addChild( of->addSeesaw( Vec3(500,0,350), Vec3(75,250,5), Vec4(0.0,1.0,0.0,1.0), true, true ) );
    //camNode->addChild( of->addSeesaw( Vec3(-500,0,350), Vec3(75,250,5), Vec4(1.0,0.0,0.0,1.0), true, true ) );
    
    camNode->addChild( of->addBox( Vec3(0,0,-5000), Vec3(5000,5000,5000), Quat(0,0,0,1), Vec4(1.0,1.0,1.0,1.0), false, true ) );
    
    handBall = of->addCylinderHand( 1.0f, 1000000.0f, Vec4(1,1,1,1) );
    PluginHelper::getScene()->addChild( handBall );
    handBall->setMatrix( PluginHelper::getHandMat(0) );
    
    // Invisible Sphere Walls
    of->addInvisibleWall( Vec3(0,105,2000), Vec3(2000,10,2000), COL_SPHERE );
    of->addInvisibleWall( Vec3(0,-105,2000), Vec3(2000,10,2000), COL_SPHERE );
    of->addInvisibleWall( Vec3(-2000,0,2000), Vec3(10,100,2000), COL_SPHERE );
    
    // Start Ramp
    camNode->addChild( of->addBox( Vec3(-1850, 0, 800), Vec3(100,100,5), Quat(pi / (float) 6, Vec3(0,-1,0)), Vec4(0.6,0.25,0.1,1.0), false, true ) );
    camNode->addChild( of->addBox( Vec3(-250, 0, 300), Vec3(1500,100,5), Quat(pi / (float) 36, Vec3(0,1,0)), Vec4(0.6,0.25,0.1,1.0), false, true ) );
    
    // goal bucket
    camNode->addChild( of->addOpenBox( Vec3(1850, 0, 100), Vec3(100, 100, 50), 10.0f, false, true) );
    of->addGoalZone( Vec3(1850, 0, 100), Vec3(80, 80, 40) );
    
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
    //delete nh;
}

void PhysicsLab::preFrame()
{
    // Camera Movement
    if (goLeft || goRight || goUp || goDown) {
      const float diff = 3.f;
      float xdiff = diff * PluginHelper::getLastFrameDuration() * 300.0f;
      
      Matrix cam = camNode->getMatrix();
      if (goLeft && !goRight) {
        cam.setTrans( cam.getTrans() + cam.getRotate()*Vec3(xdiff,0.f,0.f) );
      } else if (goRight && !goLeft) {
        cam.setTrans( cam.getTrans() + cam.getRotate()*Vec3(-xdiff,0.f,0.f) );
      }
      
      if (goUp && !goDown) {
        cam.setTrans( cam.getTrans() + cam.getRotate()*Vec3(0.f,-xdiff,0.f) );
      } else if (goDown && !goUp) {
        cam.setTrans( cam.getTrans() + cam.getRotate()*Vec3(0.f,xdiff,0.f) );
      }
      
      camNode->setMatrix(cam);
	  }
	  
    frame = (frame + 1) % 30000;
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
    if (frame % 300 == 0 && startSim && numSpheres < NUM_SPHERES) {
      //MatrixTransform* sphereMat = of->addSphere( Vec3((float) (rand() % 400 - 200), (float) (rand() % 400 - 200),500.), 35, colorArray[numSpheres%4], true, true );
      MatrixTransform* sphereMat = of->addSphere( Vec3(-1900,0,200), 35, colorArray[numSpheres%4], true, true );
      camNode->addChild( sphereMat );
      numSpheres++;
    }
    
    Matrixd handmat = PluginHelper::getHandMat(0);
    of->updateHand(handmat, camNode->getMatrix());
    
    if (frame % 1500 == 0) {
      std::cout << "FPS: " << 1.0/PluginHelper::getLastFrameDuration() << std::endl;
    }
    
    static bool wonGame = false;
    if (of->wonGame() ^ wonGame) {
      std::cout << "WON.\n";
      wonGame = of->wonGame();
    }
      
    if (startSim) of->stepSim( PluginHelper::getLastFrameDuration() );
}

bool PhysicsLab::processEvent(InteractionEvent * event) {
    static bool grabbing = false;
    
    KeyboardInteractionEvent * kp;
    TrackedButtonInteractionEvent * he;
    if ((kp = event->asKeyboardEvent()) != NULL) {
        if (kp->getInteraction() == KEY_DOWN) {
          switch (kp->getKey()) {
              case A:
                  goLeft = true;
                  break;
              case D:
                  goRight = true;
                  break;
              case W:
                  if (!grabbing) goUp = true;
                  else of->pushGrabbedObject();
                  break;
              case S:
                  if (!grabbing) goDown = true;
                  else of->pullGrabbedObject();
                  break;
              case SPACE:
                  break;
          }
        } else if (kp->getInteraction() == KEY_UP) {
          switch (kp->getKey()) {
              case A:
                  goLeft = false;
                  break;
              case D:
                  goRight = false;
                  break;
              case W:
                  goUp = false;
                  break;
              case S:
                  goDown = false;
                  break;
              case SPACE:
                  break;
          }
      }
    } else if ((he = event->asTrackedButtonEvent()) != NULL) {
      if (he->getHand() == 0 && he->getButton() == 0) {
          Matrixd handmat = PluginHelper::getHandMat(0);
          //handmat.preMult(camNode->getMatrix());
          //handBall->setMatrix(handmat);
          //std::cout << handmat <<std::endl;
        if (he->getInteraction() == BUTTON_DOWN && !grabbing)
            grabbing = of->grabObject( handmat, PluginHelper::getScene() );
            //grabbing = of->grabObject( handmat, camNode );
        else if (he->getInteraction() == BUTTON_UP) {
            grabbing = false;
            of->releaseObject();
        }
      }
    }

    return true;
}
