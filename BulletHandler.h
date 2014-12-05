#ifndef _BULLETHANDLER_H
#define _BULLETHANDLER_H

#include <vector>
#include <osg/Vec3>
#include <osg/Matrixd>
#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include "AntiGravityField.h"

class BulletHandler
{
  
  public:
    BulletHandler();
    virtual ~BulletHandler();
    int addBox( osg::Vec3, osg::Vec3, bool );
    int addSeesaw( osg::Vec3, osg::Vec3, bool );
    int addSphere( osg::Vec3, double, bool );
    int addOpenBox( osg::Vec3, osg::Vec3, double, bool );
    int addHollowBox( osg::Vec3, osg::Vec3, bool );
    void addAntiGravityField(osg::Vec3, double, osg::Vec3);
    void setLinearVelocity( int, osg::Vec3 );
    osg::Vec3 getLinearVelocity( int );
    void translate( int, osg::Vec3 );
    void setGravity( osg::Vec3 );
    
    void stepSim( double );
    btDiscreteDynamicsWorld* getDynamicsWorld();
    void getWorldTransform( int, osg::Matrixd& );
    
  private:
    btBroadphaseInterface* broadphase;
    btDefaultCollisionConfiguration* btcc;
    btCollisionDispatcher* btcd;
    btSequentialImpulseConstraintSolver* btsolver;
    btDiscreteDynamicsWorld* dynamicsWorld;
    
    std::vector<btRigidBody*> rbodies;
    int numRigidBodies;
    
    std::vector<AntiGravityField*> avfs;
    int numavfs;
};
#endif
