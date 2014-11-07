#include "PhysicsLab.h"

#include <PluginMessageType.h>

#include <iostream>
#include <string>
#include <cmath>

// OSG:
#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Vec3d>
#include <osg/MatrixTransform>

using namespace std;
using namespace cvr;
using namespace osg;

CVRPLUGIN(PhysicsLab)

void setupScene( BulletHandler* );

const double pi = 3.141593;
int frame = 0;
bool scenesetup = false;
MatrixTransform * camNode, *boxMatrix;
LightSource * lightsrc;
PositionAttitudeTransform * lightMat;
StateSet* lightSS;
int boxId;

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
  
  bh = new BulletHandler();
  setupScene( bh );
  
  return true;
}

void setupScene( BulletHandler * bh ) {
    
    //navHandle = new NavigationHandler;
	  PluginHelper::setObjectScale(1000.0);
	  
	  camNode = new MatrixTransform;
	  Matrix cam0, cam1, cam2;
	  cam0.makeRotate(45 * pi / 180, Vec3(0,0,1));
	  cam1.makeRotate(45 * pi / 180, Vec3(1,0,0));
	  cam2.makeTranslate(10,0,-200);
	  camNode->setMatrix(cam0 * cam1 * cam2);
    
    // Floor
    Geode * floor = new Geode;
    Geometry* floorGeometry = new Geometry;
    floor->addDrawable( floorGeometry );
    PluginHelper::getScene()->addChild( camNode );
    //camNode->addChild( floor );
    Geode * box = new Geode;
    Box * boxprim = new Box( Vec3(0,0,0), 300);
    ShapeDrawable * sd = new ShapeDrawable(boxprim);
    box->addDrawable(sd);
    boxMatrix = new MatrixTransform;
    Matrix boxm;
    boxm.makeTranslate(0,0,600);
    boxMatrix->setMatrix( boxm );
    boxId = bh->addBox( Vec3(0,0,600), 150 );
    camNode->addChild( boxMatrix );
    boxMatrix->addChild( box );
    
    Geode * floorBox = new Geode;
    Box * floorBoxPrim = new Box( Vec3(0,0,-1000), 2000);
    ShapeDrawable * floorsd = new ShapeDrawable(floorBoxPrim);
    floorBox->addDrawable(floorsd);
    floorsd->setColor(Vec4(0.7,0.7,0.7,1.0));
    camNode->addChild( floorBox );
    
    const float floorWidth = 1000.0f;
    Vec3Array* floorVerts = new Vec3Array;
    floorVerts->push_back( Vec3(-floorWidth, -floorWidth, 0.0f) );
    floorVerts->push_back( Vec3(floorWidth, -floorWidth, 0.0f) );
    floorVerts->push_back( Vec3(floorWidth, floorWidth, 0.0f) );
    floorVerts->push_back( Vec3(-floorWidth, floorWidth, 0.0f) );
    //floorVerts->push_back( Vec3(0, 0, -floorWidth) );
    //floorVerts->push_back( Vec3(floorWidth, 0, -floorWidth) );
    //floorVerts->push_back( Vec3(floorWidth, 0, floorWidth) );
    //floorVerts->push_back( Vec3(0, 0, floorWidth) );
    floorGeometry->setVertexArray( floorVerts );
    
    Vec3Array * floorNorms = new Vec3Array;
    floorNorms->push_back( Vec3(0,0,1) );
    floorGeometry->setNormalArray( floorNorms );
    
    TemplateIndexArray<unsigned int, Array::UIntArrayType, 24, 4> *normalIndexArray;
    normalIndexArray =  new TemplateIndexArray<unsigned int, Array::UIntArrayType, 24, 4>();
    normalIndexArray->push_back(0);
    normalIndexArray->push_back(0);
    normalIndexArray->push_back(0);
    normalIndexArray->push_back(0);
    floorGeometry->setNormalIndices(normalIndexArray);
    
    DrawElementsUInt* floorFace = new DrawElementsUInt(PrimitiveSet::QUADS, 0);
    floorFace->push_back(3);
    floorFace->push_back(2);
    floorFace->push_back(1);
    floorFace->push_back(0);
    floorGeometry->addPrimitiveSet(floorFace);
    
    Vec4Array* colors = new Vec4Array;
    colors->push_back( Vec4(0.0f, 1.0f, 0.0f, 1.0f) );
    colors->push_back( Vec4(0.0f, 1.0f, 0.0f, 1.0f) );
    colors->push_back( Vec4(0.0f, 1.0f, 0.0f, 1.0f) );
    colors->push_back( Vec4(0.0f, 1.0f, 0.0f, 1.0f) );
    floorGeometry->setColorArray(colors);
    floorGeometry->setColorBinding(Geometry::BIND_PER_VERTEX);
    
    // Light 0
    lightSS = PluginHelper::getScene()->getOrCreateStateSet();
    lightsrc = new LightSource;
    lightMat = new PositionAttitudeTransform();
    lightMat->addChild( lightsrc );
    Light * light0 = new Light;
    light0->setPosition( Vec4(0.0f, 0.0f, 0.0f, 1.0f) );
    light0->setLightNum(1);
    light0->setAmbient( Vec4(0.1f, 0.1f, 0.1f, 1.0f) );
    light0->setDiffuse( Vec4(0.7f, 0.1f, 0.1f, 1.0f) );
    light0->setSpecular( Vec4(0.3f, 0.1f, 0.1f, 1.0f) );
    light0->setConstantAttenuation(1.0f);
    lightsrc->setLight( light0 );
    lightsrc->setLocalStateSetModes(osg::StateAttribute::ON);
    lightsrc->setStateSetModes(*lightSS, osg::StateAttribute::ON);
    Geode * sphere = new Geode;
    Sphere * sphereprim = new Sphere( Vec3() , 50);
    ShapeDrawable * sdp = new ShapeDrawable(sphereprim);
    sphere->addDrawable(sdp);
    Material *material = new Material();
    material->setDiffuse(Material::FRONT,  Vec4(0.0, 0.0, 0.0, 1.0));
    material->setEmission(Material::FRONT, Vec4(1.0,0.1,0.1,1.0));
    sphere->getOrCreateStateSet()->setAttribute(material);
    lightMat->addChild( sphere );
    lightMat->setPosition(Vec3(0,-300,100));
    lightMat->setScale(Vec3(0.1,0.1,0.1));
    camNode->addChild(lightMat);
    /*
    osg::StateSet* brickState = floor->getOrCreateStateSet();

       osg::Program* brickProgramObject = new osg::Program;
       osg::Shader* brickFragmentObject = 
          new osg::Shader( osg::Shader::FRAGMENT );
       brickProgramObject->addShader( brickFragmentObject );
       brickFragmentObject->loadShaderSourceFromFile("glsl/brick.frag");

    brickState->setAttributeAndModes(brickProgramObject, osg::StateAttribute::ON);
    */
    
    scenesetup = true;
}

// this is called if the plugin is removed at runtime
PhysicsLab::~PhysicsLab()
{
    
}

void PhysicsLab::preFrame()
{
    static double z = 250.0;
    
    Matrix cam = camNode->getMatrix();
    Matrix change;
    change.makeRotate(0.1 * pi / 180, cam.getRotate()*Vec3(0,0,1));
    camNode->setMatrix(cam*change);
    frame = (frame + 1) % 720;
    
    if (frame % 60 == 0) {
      //std::cout << camNode->getMatrix();
      //z += 10;
    }
    if (scenesetup) {
        bh->stepSim();
        boxMatrix->setMatrix( bh->getWorldTransform( boxId ) );
        if (frame % 60 == 0) {
            std::cout << "Box linear z velocity: " << bh->getLinearVelocity( boxId ).z() << std::endl;
        }
    }
}

bool PhysicsLab::processEvent(InteractionEvent * event) {
    static bool lightswitch = true;
    
    KeyboardInteractionEvent * kp;
    if ((kp = event->asKeyboardEvent()) != NULL) {
        if (kp->getInteraction() == KEY_DOWN) {
            Matrix m = camNode->getMatrix();
            Matrix change = Matrix::identity();
            Vec3 trans(0,0,0);
            //std::cout << "KEY: " << kp->getKey() << std::endl;
            Vec3 vel = bh->getLinearVelocity( boxId );
            switch (kp->getKey()) {
                case 65361: //LEFT
                    std::cout << "Pressed LEFT.\n";
                    change.makeRotate(-15 * pi / 180, Vec3(0,0,1));
                    //trans.y() = 100;
                    bh->setLinearVelocity( boxId, Vec3(1000,vel.y(),vel.z()) );
                    bh->translate( boxId, Vec3(0,0,1) );
                    break;
                case 65362: //UP
                    std::cout << "Pressed UP.\n";
                    change.makeTranslate(m.getRotate()*Vec3(-20,0,0));
                    //std::cout << change;
                    //trans.z() = 100;
                    bh->setLinearVelocity( boxId, Vec3(vel.x(),1000,vel.z()) );
                    bh->translate( boxId, Vec3(0,0,1) );
                    break;
                case 65363: //RIGHT
                    std::cout << "Pressed RIGHT.\n";
                    change.makeRotate(15 * pi / 180, Vec3(0,0,1));
                    //trans.y() = -100;
                    bh->setLinearVelocity( boxId, Vec3(-1000,vel.y(),vel.z()) );
                    bh->translate( boxId, Vec3(0,0,1) );
                    break;
                case 65364: //DOWN
                    std::cout << "Pressed DOWN.\n";
                    change.makeTranslate(m.getRotate()*Vec3(20,0,0));
                    //std::cout << change;
                    //trans.z() = -100;
                    bh->setLinearVelocity( boxId, Vec3(vel.x(),-1000,vel.z()) );
                    bh->translate( boxId, Vec3(0,0,1) );
                    break;
                case 108: // L
                    if (!lightswitch) {
                        std::cout << "Turning on lightsrc.\n";
                        lightsrc->setLocalStateSetModes(osg::StateAttribute::ON);
                        lightsrc->setStateSetModes(*lightSS, osg::StateAttribute::ON);
                    } else {
                        std::cout << "Turning off lightsrc.\n";
                        lightsrc->setLocalStateSetModes(osg::StateAttribute::OFF);
                        lightsrc->setStateSetModes(*lightSS, osg::StateAttribute::OFF);
                    }
                    lightswitch = !lightswitch;
                    break;
                case 32: // Space
                    std::cout << "Pressed SPACE.\n";
                    bh->setLinearVelocity( boxId, Vec3(vel.x(),vel.y(),3000) );
                    bh->translate( boxId, Vec3(0,0,1) );
                    break;
            }
            //camNode->setPosition(m*change);
            lightMat->setPosition(lightMat->getPosition()+trans);
        }
    }

    return false;
}
