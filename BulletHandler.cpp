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

void tickCallback(btDynamicsWorld *world, btScalar timeStep)
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
    dynamicsWorld->setInternalTickCallback(tickCallback,this,true);
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

int BulletHandler::addOpenBox( osg::Vec3 origin, osg::Vec3 halfLengths, double innerWidth, bool physEnabled ) {
    btCollisionShape* xShape = new btBoxShape( btVector3(innerWidth/2, halfLengths.y(), halfLengths.z()) );
    btCollisionShape* yShape = new btBoxShape( btVector3(halfLengths.x(), innerWidth/2, halfLengths.z()) );
    btCollisionShape* zShape = new btBoxShape( btVector3(halfLengths.x(), halfLengths.y(), innerWidth/2) );
    
    btCompoundShape* boxShape = new btCompoundShape();
    
    boxShape->addChildShape( btTransform(btQuaternion(0,0,0,1), -btVector3(halfLengths.x() - innerWidth / 2, 0, 0)), xShape);
    boxShape->addChildShape( btTransform(btQuaternion(0,0,0,1), btVector3(halfLengths.x() - innerWidth / 2, 0, 0)), xShape);
    boxShape->addChildShape( btTransform(btQuaternion(0,0,0,1), -btVector3(0, halfLengths.y() - innerWidth / 2, 0)), yShape);
    boxShape->addChildShape( btTransform(btQuaternion(0,0,0,1), btVector3(0, halfLengths.y() - innerWidth / 2, 0)), yShape);
    boxShape->addChildShape( btTransform(btQuaternion(0,0,0,1), -btVector3(0, 0, halfLengths.z() - innerWidth / 2)), zShape);
    
    btDefaultMotionState* boxMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), *(btVector3*) &origin));
        
    addRigid( boxShape, boxMotionState, COL_NORMAL, normalCollides, physEnabled );
    
    return numRigidBodies++;
}

int BulletHandler::addHollowBox( osg::Vec3 origin, osg::Vec3 halfLengths, bool physEnabled ) {
    const float innerWidth = 1.0f;
    btCollisionShape* xShape = new btBoxShape( btVector3(innerWidth/2, halfLengths.y(), halfLengths.z()) );
    btCollisionShape* yShape = new btBoxShape( btVector3(halfLengths.x(), innerWidth/2, halfLengths.z()) );
    btCollisionShape* zShape = new btBoxShape( btVector3(halfLengths.x(), halfLengths.y(), innerWidth/2) );
    
    btCompoundShape* boxShape = new btCompoundShape();
    
    boxShape->addChildShape( btTransform(btQuaternion(0,0,0,1), -btVector3(halfLengths.x() - innerWidth / 2, 0, 0)), xShape);
    boxShape->addChildShape( btTransform(btQuaternion(0,0,0,1), btVector3(halfLengths.x() - innerWidth / 2, 0, 0)), xShape);
    boxShape->addChildShape( btTransform(btQuaternion(0,0,0,1), -btVector3(0, halfLengths.y() - innerWidth / 2, 0)), yShape);
    boxShape->addChildShape( btTransform(btQuaternion(0,0,0,1), btVector3(0, halfLengths.y() - innerWidth / 2, 0)), yShape);
    boxShape->addChildShape( btTransform(btQuaternion(0,0,0,1), -btVector3(0, 0, halfLengths.z() - innerWidth / 2)), zShape);
    boxShape->addChildShape( btTransform(btQuaternion(0,0,0,1), btVector3(0, 0, halfLengths.z() - innerWidth / 2)), zShape);
    
    btDefaultMotionState* boxMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(origin.x(), origin.y(), origin.z())));
        
    btRigidBody * boxRigidBody = addRigid( boxShape, boxMotionState, COL_NORMAL, normalCollides, physEnabled );
    
    return numRigidBodies++;
}

int BulletHandler::addCustomObject( osg::Vec3Array vert0, osg::Vec3Array vert1, osg::Vec3Array vert2, osg::Vec3 pos, bool physEnabled ) {
    if (vert0.getNumElements() != vert1.getNumElements() ||
        vert0.getNumElements() != vert2.getNumElements())
      return -1;
        
    btTriangleMesh* tri_mesh = new btTriangleMesh();
    
    // add all the triangles to the mesh
    for (int i = 0; i < vert0.getNumElements(); ++i)
      tri_mesh->addTriangle( *(btVector3*)&(vert0[i]), *(btVector3*)&(vert1[i]), *(btVector3*)&(vert2[i]), true );
      
    btBvhTriangleMeshShape* triShape = new btBvhTriangleMeshShape( tri_mesh, false );
    btDefaultMotionState* triMotionState =
        new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), *(btVector3*)&pos));
        
    btRigidBody * boxRigidBody = addRigid( triShape, triMotionState, COL_NORMAL, normalCollides, physEnabled );
    
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
        osg::Quat q = boxm.getRotate();
        btQuaternion btq( q.x(), q.y(), q.z(), q.w() );
        btTransform btt( btq, btv);
        
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
