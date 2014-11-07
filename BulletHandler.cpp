#include "BulletHandler.h"

BulletHandler::BulletHandler() {
    // Create World
    broadphase = new btDbvtBroadphase();
    btcc = new btDefaultCollisionConfiguration();
    btcd = new btCollisionDispatcher(btcc);
    btsolver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btDiscreteDynamicsWorld(btcd, broadphase, btsolver, btcc);
    dynamicsWorld->setGravity( btVector3(0,0,-10000) );
    
    // Add Floor
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 1);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    dynamicsWorld->addRigidBody(groundRigidBody);
    
    numRigidBodies = 0;
}

BulletHandler::~BulletHandler() {
    delete dynamicsWorld;
    delete btsolver;
    delete broadphase;
    delete btcc;
    delete btcd;
}

int BulletHandler::addBox( osg::Vec3 origin, double halfwidth ) {
    btCollisionShape* boxShape = new btBoxShape( btVector3(halfwidth, halfwidth, halfwidth) );
    btDefaultMotionState* boxMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(origin.x(), origin.y(), origin.z())));
    btVector3 boxInertia(0,0,0);
    boxShape->calculateLocalInertia(btScalar(1), boxInertia);
    btRigidBody::btRigidBodyConstructionInfo boxRigidBodyCI(1, boxMotionState, boxShape, boxInertia);
    btRigidBody * boxRigidBody = new btRigidBody(boxRigidBodyCI);
    dynamicsWorld->addRigidBody(boxRigidBody);
    rbodies.push_back(boxRigidBody);
    
    return numRigidBodies++;
}

int BulletHandler::addSphere( osg::Vec3 origin, double width ) {
    return numRigidBodies++;
}

void BulletHandler::setLinearVelocity( int id, osg::Vec3 vel ) {
    rbodies[id]->setLinearVelocity( btVector3(vel.x(), vel.y(), vel.z()) );
}

void BulletHandler::translate( int id, osg::Vec3 vel ) {
    rbodies[id]->translate( btVector3(vel.x(), vel.y(), vel.z()) );
}

osg::Vec3 BulletHandler::getLinearVelocity( int id ) {
    btVector3 lv = rbodies[id]->getLinearVelocity();
    return osg::Vec3(lv.x(), lv.y(), lv.z());
}

void BulletHandler::stepSim() {
    dynamicsWorld->stepSimulation( 1 / 60.f, 10 );
}

osg::Matrixd BulletHandler::getWorldTransform( int id ) {
    btTransform trans;
    rbodies[id]->getMotionState()->getWorldTransform(trans);
    osg::Matrixd boxm;
    boxm.makeTranslate(trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z() );
    return boxm;
}
