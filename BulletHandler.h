#ifndef _BULLETHANDLER_H
#define _BULLETHANDLER_H

#include <vector>
#include <osg/Vec3>
#include <osg/Matrixd>
#include <btBulletDynamicsCommon.h>

class BulletHandler
{
  public:
    BulletHandler();
    virtual ~BulletHandler();
    int addBox( osg::Vec3 origin, double halfwidth );
    int addSphere( osg::Vec3 origin, double radius );
    void setLinearVelocity( int, osg::Vec3 );
    osg::Vec3 getLinearVelocity( int );
    void translate( int, osg::Vec3 );
    
    void stepSim();
    osg::Matrixd getWorldTransform( int );
    
  private:
    btBroadphaseInterface* broadphase;
    btDefaultCollisionConfiguration* btcc;
    btCollisionDispatcher* btcd;
    btSequentialImpulseConstraintSolver* btsolver;
    btDiscreteDynamicsWorld* dynamicsWorld;
    
    std::vector<btRigidBody*> rbodies;
    int numRigidBodies;
};
#endif
