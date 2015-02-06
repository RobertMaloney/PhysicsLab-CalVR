#include "BulletHandler.h"
#include <iostream>

int clnumavfs = 0;
std::vector<AntiGravityField*> clavfs;
int buttonState = 0, prevButtonState = -1;
AntiGravityField* hand;
btRigidBody* closest;
btVector3 initGrabPos;
btVector3 distToStylus;

CollisionType normalCollides = (CollisionType) (COL_NORMAL | COL_SPHERE);

void ghostCallback (btDynamicsWorld *world, btScalar timeStep)
{
    // AVF
    for (int i = 0; i < clnumavfs; ++i) {
        //std::cout << "Num Ghost Collisions: " << clavfs[i]->getNumOverlappingObjects() << "\n";
        for(int j = 0; j < clavfs[i]->getNumOverlappingObjects(); j++) {
            btRigidBody *pRigidBody = dynamic_cast<btRigidBody *>(clavfs[i]->getOverlappingObject(j));
            pRigidBody->setGravity( clavfs[i]->getGravity() );
        }
    }
}

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
    dynamicsWorld->setInternalTickCallback(ghostCallback,this,true);
    dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
    
    numRigidBodies = 0;
    numavfs = 0;
}

BulletHandler::~BulletHandler() {
    delete dynamicsWorld;
    delete btsolver;
    delete broadphase;
    delete btcc;
    delete btcd;
}

int BulletHandler::addBox( osg::Vec3 origin, osg::Vec3 halfLengths, osg::Quat quat, bool physEnabled ) {
    btCollisionShape* boxShape = new btBoxShape( *(btVector3*) &halfLengths );
    btDefaultMotionState* boxMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(quat.x(), quat.y(), quat.z(), quat.w()), *(btVector3*)&origin));
        
    addRigid( boxShape, boxMotionState, COL_NORMAL, normalCollides, physEnabled );
    
    return numRigidBodies++;
}

int BulletHandler::addSeesaw( osg::Vec3 origin, osg::Vec3 halflengths, bool physEnabled ) {
    btCollisionShape* boxShape = new btBoxShape( *(btVector3*) &halflengths );
    btDefaultMotionState* boxMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), *(btVector3*)&origin));
        
    btRigidBody* boxRigidBody = addRigid( boxShape, boxMotionState, COL_NORMAL, normalCollides, physEnabled );
    
    if (halflengths.x() > halflengths.y()) {
        btHingeConstraint* hinge = new btHingeConstraint(*boxRigidBody,btVector3(0,0,0),btVector3(0,1,0),true);
		    dynamicsWorld->addConstraint(hinge);
		} else {
        btHingeConstraint* hinge = new btHingeConstraint(*boxRigidBody,btVector3(0,0,0),btVector3(1,0,0),true);
		    dynamicsWorld->addConstraint(hinge);
		}
    
    return numRigidBodies++;
}

int BulletHandler::addOpenBox( osg::Vec3 origin, osg::Vec3 halflengths, double innerWidth, bool physEnabled ) {
    btCompoundShape* boxShape = new btCompoundShape();
    std::vector<btVector3> planePoints [14];
    
    // Left Plane - Outer
    planePoints[0].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[0].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[0].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[0].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    
    // Back Plane - Outer
    planePoints[1].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[1].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[1].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[1].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    
    // Right Plane - Outer
    planePoints[2].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[2].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[2].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[2].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    
    // Front Plane - Outer
    planePoints[3].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[3].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[3].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[3].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    
    // Bottom Plane - Outer
    planePoints[4].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[4].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[4].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[4].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    
    if (innerWidth > 0.0) {
    
        // Left Plane - Inner
        planePoints[5].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() - halflengths.y() + innerWidth, origin.z() - halflengths.z() + innerWidth));
        planePoints[5].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() - halflengths.y() + innerWidth, origin.z() + halflengths.z()) );
        planePoints[5].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() - halflengths.y() + innerWidth, origin.z() + halflengths.z()) );
        planePoints[5].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() - halflengths.y() + innerWidth, origin.z() - halflengths.z() + innerWidth));
    
        // Back Plane - Inner
        planePoints[6].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() - halflengths.y() + innerWidth, origin.z() - halflengths.z() + innerWidth) );
        planePoints[6].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() - halflengths.y() + innerWidth, origin.z() + halflengths.z()) );
        planePoints[6].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() + halflengths.y() - innerWidth, origin.z() + halflengths.z()) );
        planePoints[6].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() + halflengths.y() - innerWidth, origin.z() - halflengths.z() + innerWidth) );
        
        // Right Plane - Inner
        planePoints[7].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() + halflengths.y() - innerWidth, origin.z() - halflengths.z() + innerWidth) );
        planePoints[7].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() + halflengths.y() - innerWidth, origin.z() + halflengths.z()) );
        planePoints[7].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() + halflengths.y() - innerWidth, origin.z() + halflengths.z()) );
        planePoints[7].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() + halflengths.y() - innerWidth, origin.z() - halflengths.z() + innerWidth) );
        
        // Front Plane - Inner
        planePoints[8].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() - halflengths.y() + innerWidth, origin.z() - halflengths.z() + innerWidth) );
        planePoints[8].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() - halflengths.y() + innerWidth, origin.z() + halflengths.z()) );
        planePoints[8].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() + halflengths.y() - innerWidth, origin.z() + halflengths.z()) );
        planePoints[8].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() + halflengths.y() - innerWidth, origin.z() - halflengths.z() + innerWidth) );
        
        // Bottom Plane - Inner
        
        planePoints[9].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() + halflengths.y() - innerWidth, origin.z() - halflengths.z() + innerWidth) );
        planePoints[9].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() - halflengths.y() + innerWidth, origin.z() - halflengths.z() + innerWidth) );
        planePoints[9].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() - halflengths.y() + innerWidth, origin.z() - halflengths.z() + innerWidth) );
        planePoints[9].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() + halflengths.y() - innerWidth, origin.z() - halflengths.z() + innerWidth) );
        // Left Plane - Connect
        planePoints[10].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
        planePoints[10].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
        planePoints[10].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
        planePoints[10].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
    
        // Back Plane - Connect
        planePoints[11].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
        planePoints[11].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y() + innerWidth, origin.z() + halflengths.z()) );
        planePoints[11].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y() - innerWidth, origin.z() + halflengths.z()) );
        planePoints[11].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
        
        // Right Plane - Connect
        planePoints[12].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
        planePoints[12].push_back( btVector3( origin.x() - halflengths.x() + innerWidth, origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
        planePoints[12].push_back( btVector3( origin.x() + halflengths.x() - innerWidth, origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
        planePoints[12].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
        
        // Front Plane - Connect
        planePoints[13].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
        planePoints[13].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y() + innerWidth, origin.z() + halflengths.z()) );
        planePoints[13].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y() - innerWidth, origin.z() + halflengths.z()) );
        planePoints[13].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
        
        
    }
    
    btConvexHullShape * boxPlanes [((innerWidth > 0) ? 14 : 5)];
    for (int i = 0; i < ((innerWidth > 0) ? 14 : 5); ++i) {
        if (i < 14 ) {
        boxPlanes[i] = new btConvexHullShape();
        for (int j = 0; j < 4; ++j)
            boxPlanes[i]->addPoint( planePoints[i][j] );
            
        boxShape->addChildShape (btTransform::getIdentity(), boxPlanes[i]);
        }
    }
    
    btDefaultMotionState* boxMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), *(btVector3*) &origin));
        
    addRigid( boxShape, boxMotionState, COL_NORMAL, normalCollides, physEnabled );
    
    return numRigidBodies++;
}

int BulletHandler::addHollowBox( osg::Vec3 origin, osg::Vec3 halflengths, bool physEnabled ) {
    btCompoundShape* boxShape = new btCompoundShape();
    std::vector<btVector3> planePoints [6];
    
    // Left Plane
    planePoints[0].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[0].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[0].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[0].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    
    // Back Plane
    planePoints[1].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[1].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[1].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[1].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    
    // Right Plane
    planePoints[2].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[2].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[2].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[2].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    
    // Front Plane
    planePoints[3].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[3].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[3].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[3].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    
    // Top Plane
    planePoints[4].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[4].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[4].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() + halflengths.z()) );
    planePoints[4].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() + halflengths.z()) );
    
    // Bottom Plane
    planePoints[5].push_back( btVector3( origin.x() - halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[5].push_back( btVector3( origin.x() - halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[5].push_back( btVector3( origin.x() + halflengths.x(), origin.y() - halflengths.y(), origin.z() - halflengths.z()) );
    planePoints[5].push_back( btVector3( origin.x() + halflengths.x(), origin.y() + halflengths.y(), origin.z() - halflengths.z()) );
    
    btConvexHullShape * boxPlanes [6];
    for (int i = 0; i < 6; ++i) {
        boxPlanes[i] = new btConvexHullShape();
        for (int j = 0; j < 4; ++j)
            boxPlanes[i]->addPoint( planePoints[i][j] );
            
        boxShape->addChildShape (btTransform::getIdentity(), boxPlanes[i]);
    }
    
    btDefaultMotionState* boxMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(origin.x(), origin.y(), origin.z())));
        
    btRigidBody * boxRigidBody = addRigid( boxShape, boxMotionState, COL_NORMAL, normalCollides, physEnabled );
    
    return numRigidBodies++;
}

int BulletHandler::addSphere( osg::Vec3 origin, double width, bool physEnabled ) {
    btCollisionShape* sphereShape = new btSphereShape( width );
    btDefaultMotionState* sphereMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), *(btVector3*)&origin));
        
    btRigidBody * sphereRigidBody = addRigid( sphereShape, sphereMotionState, COL_SPHERE, (CollisionType) (normalCollides | COL_WALL), physEnabled );
    
    return numRigidBodies++;
}

int BulletHandler::addCylinder( osg::Vec3 origin, osg::Vec3 halfLengths, bool physEnabled ) {
    btCollisionShape* cylShape = new btCylinderShapeZ( *(btVector3*) &halfLengths );
    btDefaultMotionState* cylMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), *(btVector3*)&origin));
        
    btRigidBody * cylRigidBody = addRigid( cylShape, cylMotionState, COL_NORMAL, normalCollides, physEnabled );
    
    return numRigidBodies++;
}

void BulletHandler::addAntiGravityField(osg::Vec3 pos, osg::Vec3 halfLengths, osg::Vec3 grav) {
    btCollisionShape* ghostBox = new btBoxShape( *(btVector3*) &halfLengths );
    AntiGravityField* avf = new AntiGravityField();
    avf->setGravity( *(btVector3*) &grav );
    avf->setCollisionShape( ghostBox );
    avf->setCollisionFlags(avf->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    avf->setWorldTransform( btTransform(btQuaternion(0,0,0,1), *(btVector3*) &pos) );
    dynamicsWorld->addCollisionObject(avf);
    
    avfs.push_back(avf);
    clavfs.push_back(avf);
    numavfs++;
    clnumavfs++;
}

void BulletHandler::addInvisibleWall( osg::Vec3 origin, osg::Vec3 halfLengths, int collisionFlag ) {
    btCollisionShape* boxShape = new btBoxShape( *(btVector3*) &halfLengths );
    btDefaultMotionState* boxMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), *(btVector3*)&origin));
        
    addRigid( boxShape, boxMotionState, COL_WALL, (CollisionType) collisionFlag, false );
    rbodies.pop_back();
}

void BulletHandler::setLinearVelocity( int id, osg::Vec3 vel ) {
    rbodies[id]->setLinearVelocity( *(btVector3*) &vel );
}

void BulletHandler::translate( int id, osg::Vec3 vel ) {
    rbodies[id]->translate( *(btVector3*) &vel );
}

osg::Vec3 BulletHandler::getLinearVelocity( int id ) {
    btVector3 lv = rbodies[id]->getLinearVelocity();
    return *(osg::Vec3*) &lv;
}

void BulletHandler::stepSim( double lastFrame ) {
    dynamicsWorld->stepSimulation( lastFrame, 10, 1./80. );
}

void BulletHandler::getWorldTransform( int id, osg::Matrixd & boxm ) {
    btTransform m;
    if (id >= numRigidBodies) {
        boxm.makeIdentity();
        return;
    }
    btMotionState* ms = rbodies[id]->getMotionState();
    if (ms) {
        ms->getWorldTransform(m);
        boxm.setTrans( *(osg::Vec3*) &m.getOrigin() );
        btQuaternion btquat = rbodies[id]->getOrientation();
        osg::Quat osgquat(btquat.x(),btquat.y(),btquat.z(),btquat.w());
        boxm.setRotate( osgquat );
    } else boxm.makeIdentity();
}

void BulletHandler::setWorldTransform( int id, osg::Matrixd & boxm ) {
    btTransform m;
    if (id >= numRigidBodies) return;
    
    btMotionState* ms = rbodies[id]->getMotionState();
    if (ms) {
        osg::Vec3 t = boxm.getTrans();
        btVector3 btv( t.x(), t.y(), t.z() );
    
        btQuaternion bt90;
        bt90.setRotation( btVector3(1,0,0), 3.14159 / 2 );
        
        osg::Quat q = boxm.getRotate();
        btQuaternion btq( q.x(), q.y(), q.z(), q.w() );
        btq *= bt90;
        
        btTransform btt( btQuaternion(0,0,0,1), btv);
        //std::cout << *(osg::Vec3*) &btt.getOrigin() << "\n";
        rbodies[id]->setCenterOfMassTransform(btt);
        ms->setWorldTransform(btt);
        rbodies[id]->setGravity(btVector3(0,0,0));
        dynamicsWorld->synchronizeSingleMotionState( rbodies[id] );
    }
}

void BulletHandler::activate( int id ) {
  rbodies[id]->activate();
}

void BulletHandler::moveHand(osg::Matrixd & boxm ) {
  if (!hand) return;
  
  osg::Vec3 t = boxm.getTrans();
  btVector3 btv( t.x(), t.y(), t.z() );

  osg::Quat q = boxm.getRotate();
  btQuaternion bt90;
  bt90.setRotation( btVector3(1,0,0), 0.5 * 3.14159 / 180. );
  btQuaternion btq( q.x(), q.y(), q.z(), q.w() );
  btq *= bt90;
  btTransform btt(btq, btv);
  hand->setWorldTransform(btt);
  //hand->setInterpolationWorldTransform(btt);
  if (closest) {
    btMotionState* ms = closest->getMotionState();
    if (0) {
        closest->setCenterOfMassTransform(btt);
        ms->setWorldTransform(btt);
        closest->setLinearVelocity(btVector3(0,0,0));
        dynamicsWorld->synchronizeSingleMotionState( closest );
    }
  }
}

void BulletHandler::addHand(osg::Vec3 pos, osg::Vec3 halfLengths) {
    btCollisionShape* ghostBox = new btCylinderShapeZ( *(btVector3*) &halfLengths );
    AntiGravityField* avf = new AntiGravityField();
    avf->setGravity( btVector3(0,0,0) );
    avf->setCollisionShape( ghostBox );
    avf->setCollisionFlags(avf->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
    avf->setWorldTransform( btTransform(btQuaternion(0,0,0,1), *(btVector3*) &pos) );
    dynamicsWorld->addCollisionObject(avf);
    
    //if (hand) delete hand;
    hand = avf;
}

void BulletHandler::updateButtonState( int bs ) {
  prevButtonState = buttonState;
  buttonState = bs;
}

btDiscreteDynamicsWorld* BulletHandler::getDynamicsWorld() {
    return dynamicsWorld;
}

void BulletHandler::setGravity( osg::Vec3 g ) {
    dynamicsWorld->setGravity( *(btVector3*) &g );
}

btRigidBody* BulletHandler::addRigid( btCollisionShape* shape, btDefaultMotionState* ms, CollisionType collisionId, CollisionType collidesWith, bool physEnabled ) {
    btRigidBody * _rb;
    
    if (physEnabled) {
        btVector3 inertia(0,0,0);
        shape->calculateLocalInertia(btScalar(1), inertia);
        btRigidBody::btRigidBodyConstructionInfo _rbci(1, ms, shape, inertia);
        _rbci.m_linearSleepingThreshold = 50.0f;
        _rb = new btRigidBody(_rbci);
    } else {
        btRigidBody::btRigidBodyConstructionInfo _rbci(0, ms, shape, btVector3(0,0,0));
        _rb = new btRigidBody(_rbci);
    }
    
    dynamicsWorld->addRigidBody(_rb, collisionId, collidesWith);
    rbodies.push_back(_rb);
    
    return _rb;
}
