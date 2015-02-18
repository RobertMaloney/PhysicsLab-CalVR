#include "ObjectFactory.h"

#include <iostream>
#include <string>
#include <cmath>

ObjectFactory::ObjectFactory() {
  bh = new BulletHandler();
  numObjects = 0;
  numLights = 0;
  grabbedId = -1;
  grabbedMatrix = 0;
}

ObjectFactory::~ObjectFactory() {
  delete bh;
}

MatrixTransform* ObjectFactory::addBox( Vec3 pos, Vec3 halfLengths, Quat quat, Vec4 diffuse, bool phys = true, bool render = true ) {
  MatrixTransform* mt = new MatrixTransform;
  
  if ( render ) {
    Geode * box = new Geode;
    Box * boxprim = new Box( Vec3(0,0,0), 1);
    boxprim->setHalfLengths( halfLengths );
    ShapeDrawable * sd = new ShapeDrawable(boxprim);
    sd->setColor( diffuse );
    box->addDrawable(sd);
    Matrix boxm;
    boxm.makeTranslate(pos);
    boxm.setRotate(quat);
    mt->setMatrix( boxm );
    mt->addChild( box );
  }
  
  numObjects++;
  m_objects.push_back( mt );
  m_physid.push_back( bh->addBox( pos, halfLengths, quat, phys ) );
  
  return mt;
}

MatrixTransform* ObjectFactory::addSeesaw( Vec3 pos, Vec3 halfLengths, Vec4 diffuse, bool phys, bool render ) {
  MatrixTransform* mt = new MatrixTransform;
  
  if (render) {
    Geode * seesaw = new Geode;
    Box * seesawPrim = new Box( Vec3(0,0,0), 1,1,1 );
    seesawPrim->setHalfLengths( halfLengths );
    ShapeDrawable * seesawsd = new ShapeDrawable(seesawPrim);
    seesaw->addDrawable(seesawsd);
    seesawsd->setColor(diffuse);
    Matrixd ssm;
    ssm.makeTranslate( pos );
    mt->setMatrix( ssm );
    mt->addChild( seesaw );
  }
  
  numObjects++;
  m_objects.push_back( mt );
  m_physid.push_back( bh->addSeesaw( pos, halfLengths, phys ) );
  
  return mt;
}

void ObjectFactory::addInvisibleWall( Vec3 pos, Vec3 halfLengths, int collisionFlag ) {
  bh->addInvisibleWall( pos, halfLengths, collisionFlag );
}

MatrixTransform* ObjectFactory::addSphere( Vec3 pos, double radius, Vec4 diffuse, bool phys, bool render ) {
  MatrixTransform* mt = new MatrixTransform;
  
  if (render) {
    Matrixd spherem;
    spherem.makeTranslate(pos);
    mt->setMatrix( spherem );
    
    Geode * tsphere = new Geode;
    Sphere * tsphereprim = new Sphere( Vec3(0,0,0), radius);
    ShapeDrawable * sphered = new ShapeDrawable(tsphereprim);
    tsphere->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    sphered->setColor( diffuse );
    tsphere->addDrawable(sphered);
    mt->addChild(tsphere);
  }
  
  numObjects++;
  m_objects.push_back( mt );
  m_physid.push_back( bh->addSphere( pos, radius, phys ) );
  
  return mt;
}

MatrixTransform* ObjectFactory::addCylinder( Vec3 pos, double radius, double height, Vec4 color, bool phys, bool render) {
  MatrixTransform* mt = new MatrixTransform;
  
  if (render) {
    Matrixd cylm;
    cylm.makeTranslate(pos);
    mt->setMatrix( cylm );
    
    Geode * tcyl = new Geode;
    Cylinder * tcylprim = new Cylinder( Vec3(0,0,0), radius, height);
    ShapeDrawable * cyld = new ShapeDrawable(tcylprim);
    tcyl->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    cyld->setColor( color );
    tcyl->addDrawable(cyld);
    mt->addChild(tcyl);
  }
  
  numObjects++;
  m_objects.push_back( mt );
  m_physid.push_back( bh->addCylinder( pos, Vec3(height, radius, 0), phys ) );
  
  return mt;
}

MatrixTransform* ObjectFactory::addOpenBox( Vec3 pos, Vec3 halfLengths, double innerWidth, bool phys, bool render ) {
  MatrixTransform* mt = new MatrixTransform;
  
  if (render) {
    CompositeShape * cs = new CompositeShape();
    cs->addChild( new Box(- Vec3(halfLengths.x() - innerWidth / 2, 0, 0), innerWidth, halfLengths.y() * 2, halfLengths.z() * 2) );
    cs->addChild( new Box(Vec3(halfLengths.x() - innerWidth / 2, 0, 0), innerWidth, halfLengths.y() * 2, halfLengths.z() * 2) );
    cs->addChild( new Box(- Vec3(0, halfLengths.y() - innerWidth / 2, 0), halfLengths.x() * 2, innerWidth, halfLengths.z() * 2) );
    cs->addChild( new Box(Vec3(0, halfLengths.y() - innerWidth / 2, 0), halfLengths.x() * 2, innerWidth, halfLengths.z() * 2) );
    cs->addChild( new Box(- Vec3(0, 0, halfLengths.z() - innerWidth / 2), halfLengths.x() * 2, halfLengths.y() * 2, innerWidth) );
    
    Geode * obg = new Geode;
    ShapeDrawable* sd = new ShapeDrawable(cs);
    sd->setColor( Vec4f(1,1,1,1) );
    obg->setNodeMask(~2);
    obg->addDrawable( sd );
    mt->addChild( obg );
    Matrixd obm;
    obm.makeTranslate(pos);
    mt->setMatrix( obm );
  }
  
  numObjects++;
  m_objects.push_back( mt );
  m_physid.push_back( bh->addOpenBox( pos, halfLengths, innerWidth, phys ) );
  
  return mt;
}

MatrixTransform* ObjectFactory::addHollowBox( Vec3 pos, Vec3 halfLengths, bool phys, bool render ) {
  MatrixTransform* mt = new MatrixTransform;
  
  numObjects++;
  m_objects.push_back( mt );
  m_physid.push_back( bh->addHollowBox( pos, halfLengths, phys ) );
  
  return mt;
}

MatrixTransform* ObjectFactory::addAntiGravityField( Vec3 pos, Vec3 halfLengths, Vec3 grav, bool phys ) {
  MatrixTransform* mt = new MatrixTransform;
  
  Geode * box = new Geode;
  Box * boxprim = new Box( Vec3(0,0,0), 1,1,1);
  boxprim->setHalfLengths( halfLengths );
  ShapeDrawable * sd = new ShapeDrawable(boxprim);
  box->addDrawable(sd);
  Matrix boxm;
  boxm.makeTranslate(pos);
  mt->setMatrix( boxm );
  mt->addChild( box );
  StateSet* ss = box->getOrCreateStateSet();
  PolygonMode* pg = new PolygonMode(PolygonMode::FRONT_AND_BACK, PolygonMode::LINE);
  ss->setAttributeAndModes(pg, StateAttribute::ON | StateAttribute::OVERRIDE);
  box->setNodeMask(~2);
    
  bh->addAntiGravityField( pos, halfLengths, grav );
  numObjects++;
  m_objects.push_back( mt );
  m_physid.push_back( -1 );
  
  return mt;
}

MatrixTransform* ObjectFactory::addPlane( Vec3 pos, double halfLength, Vec3 normal, bool phys, bool render ) {
    MatrixTransform* mt;
    
    Geode * floor = new Geode;
    Geometry* floorGeometry = new Geometry;
    floor->addDrawable( floorGeometry );
    
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
    //floorGeometry->setNormalIndices(normalIndexArray);
    
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
    
    return mt;
}

PositionAttitudeTransform* ObjectFactory::addLight( Vec3 pos, Vec4 diffuse, Vec4 specular, Vec4 ambient, StateSet* lightSS ) {
  LightSource* lightsrc = new LightSource;
  PositionAttitudeTransform* lightMat = new PositionAttitudeTransform();
  Light * light0 = new Light();
  
  lightMat->addChild( lightsrc );
  light0->setPosition( Vec4(0.0f, 0.0f, 0.0f, 1.0f) );
  light0->setLightNum(numLights++);
  light0->setAmbient( ambient );
  light0->setDiffuse( diffuse );
  light0->setSpecular( specular );
  light0->setConstantAttenuation(2.0f);
  
  lightsrc->setLight( light0 );
  lightsrc->setLocalStateSetModes(osg::StateAttribute::ON);
  lightsrc->setStateSetModes(*lightSS, osg::StateAttribute::ON);
  
  Geode * sphere = new Geode;
  Sphere * sphereprim = new Sphere( Vec3() , 50);
  ShapeDrawable * sdp = new ShapeDrawable(sphereprim);
  sphere->addDrawable(sdp);
  Material *material = new Material();
  material->setDiffuse(Material::FRONT,  Vec4(0.0, 0.0, 0.0, 1.0));
  material->setEmission(Material::FRONT, diffuse);
  sphere->getOrCreateStateSet()->setAttribute(material);
  
  lightMat->addChild( sphere );
  lightMat->setPosition( pos );
  lightMat->setScale(Vec3(0.1,0.1,0.1));
  
  return lightMat;
}

void ObjectFactory::stepSim( double elapsedTime ) {
    bh->stepSim( elapsedTime );
    
    int goalCount = 0;
    Matrixd m;
    for (int i = 0; i < numObjects; ++i) {
      if (m_physid[i] > -1 && m_physid[i] != grabbedId) {
        bh->getWorldTransform( m_physid[i], m );
        m_objects[i]->setMatrix( m );
        if (goalBounds != NULL && goalBounds->contains(m.getTrans())) {
          goalCount++;
        }
      }
    }
    if (goalCount > 1) m_wonGame = true;
}

BulletHandler* ObjectFactory::getBulletHandler() {
  return bh;
}

MatrixTransform* ObjectFactory::addBoxHand( Vec3 halfLengths, Vec4 color ) {
  MatrixTransform* mt = new MatrixTransform;
  
  Geode * box = new Geode;
  Box * boxprim = new Box( Vec3(0,0,0), 1);
  boxprim->setHalfLengths( halfLengths );
  ShapeDrawable * sd = new ShapeDrawable(boxprim);
  sd->setColor( color );
  box->addDrawable(sd);
  //mt->addChild( box );
  
  handId = bh->addBox( Vec3(0,0,0), halfLengths, Quat(0,0,0,1), false );
  
  numObjects++;
  m_objects.push_back( mt );
  m_physid.push_back( handId );
  
  return mt;
}

MatrixTransform* ObjectFactory::addCylinderHand( double radius, double height, Vec4 color ) {
  MatrixTransform* mt = new MatrixTransform;
  Geode * tcyl = new Geode;
  Cylinder * tcylprim = new Cylinder( Vec3(0,0,0), radius, height);
  tcylprim->setRotation(Quat(3.14f / (float) 2, Vec3f(-1,0,0)));
  ShapeDrawable * cyld = new ShapeDrawable(tcylprim);
  tcyl->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
  cyld->setColor( color );
  tcyl->addDrawable(cyld);
  tcyl->setNodeMask(~2);
  //mt->addChild(tcyl);
  
  //bh->addHand( Vec3(0,0,0), Vec3(radius, 0, height/2) );
  /*handId = bh->addCylinder( Vec3(0,0,0), Vec3(radius, 0, height/2), false );
  
  numObjects++;
  m_objects.push_back( mt );
  m_physid.push_back( handId );
  */
  handMat = mt;
  return mt;
}

void ObjectFactory::updateHand( Matrixd & m, const Matrixd & cam ) {
  if (grabbedMatrix) {
    m *= Matrixd::inverse(cam);
    //m *= Matrixd::translate(grabbedRelativePosition);
    //m.setTrans( m.getTrans() + m.getRotate() * grabbedRelativePosition );
    //std::cout << m.getTrans() << std::endl;
    Matrixd gm = grabbedMatrix->getMatrix();
    gm.setTrans( m.getTrans() + m.getRotate() * grabbedRelativePosition - grabbedOffset);
    grabbedMatrix->setMatrix( gm );
    
  }
}

void ObjectFactory::updateButtonState( int bs ) {
  bh->updateButtonState( bs );
}

bool ObjectFactory::grabObject( Matrixd & stylus, Node* root ) {
  Vec3d pointerEnd(0.f,1000000.f,0.f);
  pointerEnd = pointerEnd * stylus;
  
  osgUtil::IntersectVisitor objFinder;
  objFinder.setTraversalMask(2);
  LineSegment* pointerLine = new LineSegment();
  pointerLine->set( stylus.getTrans(), pointerEnd );
  objFinder.addLineSegment( pointerLine );
  root->accept( objFinder );
  
  osgUtil::IntersectVisitor::HitList hl = objFinder.getHitList(pointerLine);
  if (hl.empty()) return false;
  
  osgUtil::Hit closest = hl.front();
  std::string className = closest.getDrawable()->className();
  std::cout << closest.getDrawable()->className() << std::endl;
  //std::cout << "Drawable Class: " << className << "\n";
  if (className.compare("ShapeDrawable") == 0) {
    std::string shapeName =  ((ShapeDrawable*) closest.getDrawable())->getShape()->className();
    if (shapeName.compare("Box") == 0 || shapeName.compare("Sphere") == 0) {
      NodePath np = closest.getNodePath();
      grabbedMatrix = (MatrixTransform*) np[np.size()-2];
      for (int i = 0; i < numObjects; ++i) {
        if (m_objects[i] == grabbedMatrix) {
          std::cout << "Grabbing Geode.\n";
          //grabbedMatrix = m_objects[i];
          grabbedId = m_physid[i];
          break;
        }
      }
      if (grabbedId == -1) { grabbedMatrix = (MatrixTransform*) 0; return false; }
      else {
        Vec3 stylus2cam = closest.getWorldIntersectPoint() - stylus.getTrans();
        grabbedRelativePosition = Vec3(0, (closest.getWorldIntersectPoint() - stylus.getTrans()).length(), 0);
        
        // Put physics object away
        Matrixd garbage = Matrixd::translate(2000.,2000.,0.);
        bh->setWorldTransform(grabbedId, garbage);
        
        //grabbedMatrix->asGroup()->addChild( addSphere(closest.getLocalIntersectPoint(), 10.f, Vec4(0,0,0,1), false, true) );
        
        grabbedOffset = grabbedMatrix->getMatrix().getRotate() * closest.getLocalIntersectPoint();
        std::cout << "Offset: " << grabbedOffset << std::endl;
        grabbedShape = ((ShapeDrawable*) closest.getDrawable());
        grabbedColor = grabbedShape->getColor();
        grabbedShape->setColor( Vec4(grabbedColor.r() + 0.3, grabbedColor.g() + 0.3, grabbedColor.b() + 0.3, 1) );
      }
      return true;
    }
  } else if (className.compare("Geometry") == 0) {
    std::cout << "Geometry: " << closest.getDrawable()->asGeometry()->className() << std::endl;
  }
  
  return false;
}

void ObjectFactory::releaseObject() {
  //std::cout << "Releasing...\n";
  if (grabbedId != -1) {
    bh->setLinearVelocity(grabbedId, Vec3(0,0,0));
    bh->activate(grabbedId);
    grabbedShape->setColor(grabbedColor);
    Matrixd m = grabbedMatrix->getMatrix();
    bh->setWorldTransform( grabbedId, m );
  }
  grabbedMatrix = 0;
  grabbedShape = 0;
  //std::cout << "Throwing at Tangent: " << grabbedCurrentPosition-grabbedLastPosition << "\n";
  grabbedId = -1;
}

void ObjectFactory::addGoalZone( Vec3 pos, Vec3 halfLengths ) {
  goalBounds = new BoundingBoxd();
  goalBounds->set(pos - halfLengths, pos + halfLengths);
}

bool ObjectFactory::wonGame() {
  return m_wonGame;
}

void ObjectFactory::pushGrabbedObject() {
  grabbedRelativePosition.y() += 50.0f;
}

void ObjectFactory::pullGrabbedObject() {
  grabbedRelativePosition.y() -= 50.0f;
}

