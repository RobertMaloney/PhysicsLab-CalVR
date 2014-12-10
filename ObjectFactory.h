#ifndef _OBJECTFACTORY_H
#define _OBJECTFACTORY_H

#include <vector>

// OSG:
#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Vec3d>
#include <osg/MatrixTransform>
#include <osg/Group>
#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>
#include <osg/StateSet>
#include <osg/LightSource>
#include <osgUtil/IntersectVisitor>
#include <osg/LineSegment>
#include <osg/PolygonMode>

// Local
#include "BulletHandler.h"

using namespace osg;

class ObjectFactory {
  public:
    ObjectFactory();
    virtual ~ObjectFactory();
    
    MatrixTransform* addBox( Vec3, Vec3, Vec4, bool, bool );
    MatrixTransform* addSeesaw( Vec3, Vec3, Vec4, bool, bool );
    MatrixTransform* addSphere( Vec3, double, Vec4, bool, bool );
    MatrixTransform* addCylinder( Vec3, double, double, Vec4, bool, bool );
    MatrixTransform* addOpenBox( Vec3, Vec3, double, bool, bool );
    MatrixTransform* addHollowBox( Vec3, Vec3, bool, bool );
    MatrixTransform* addAntiGravityField( Vec3, double, Vec3, bool );
    MatrixTransform* addPlane( Vec3, double, Vec3, bool, bool );
    PositionAttitudeTransform* addLight( Vec3, Vec4, Vec4, Vec4, StateSet* );
    MatrixTransform* addBoxHand( Vec3, Vec4 );
    MatrixTransform* addCylinderHand( double, double, Vec4 );
    void updateHand( Matrixd & );
    void updateButtonState( int );
    bool grabObject( Matrixd&, Node* );
    void releaseObject();
    
    void stepSim( double );
    BulletHandler* getBulletHandler();
    
  private:
    BulletHandler* bh;
    int numObjects;
    std::vector<MatrixTransform*> m_objects;
    std::vector<ShapeDrawable*> m_shaped;
    std::vector<int> m_physid;
    int handId;
    MatrixTransform* handMat;
    MatrixTransform* grabbedMatrix;
    ShapeDrawable* grabbedShape;
    Vec4 grabbedColor;
    Vec3 grabbedRelativePosition;
    Vec3 grabbedCurrentPosition, grabbedLastPosition;
    int grabbedId;
    
    int numLights;
};
#endif
