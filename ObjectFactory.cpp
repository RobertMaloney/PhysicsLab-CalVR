#include "ObjectFactory.h"

#include <iostream>
#include <string>
#include <cmath>

ObjectFactory::ObjectFactory() {
  bh = new BulletHandler();
  numObjects = 0;
  numLights = 0;
}

ObjectFactory::~ObjectFactory() {
  delete bh;
}

MatrixTransform* ObjectFactory::addBox( Vec3 pos, Vec3 halfLengths, Vec4 diffuse, bool phys = true, bool render = true ) {
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
    mt->setMatrix( boxm );
    mt->addChild( box );
  }
  
  numObjects++;
  m_objects.push_back( mt );
  m_physid.push_back( bh->addBox( pos, halfLengths, phys ) );
  
  return mt;
}

MatrixTransform* ObjectFactory::addSeesaw( Vec3 pos, Vec3 halfLengths, Vec4 diffuse, bool phys, bool render ) {
  MatrixTransform* mt = new MatrixTransform;
  
  if (render) {
    Geode * seesaw = new Geode;
    Box * seesawPrim = new Box( Vec3(0,0,0), halfLengths.x(), halfLengths.y(), halfLengths.z() );
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

MatrixTransform* ObjectFactory::addSphere( Vec3 pos, double radius, Vec4 diffuse, bool phys, bool render ) {
  MatrixTransform* mt = new MatrixTransform;
  
  if (render) {
    Matrixd spherem;
    spherem.makeTranslate(pos);
    mt->setMatrix( spherem );
    
    Geode * tsphere = new Geode;
    Sphere * tsphereprim = new Sphere( Vec3(0,0,0), radius);
    ShapeDrawable * sphered = new ShapeDrawable(tsphereprim);
    sphered->setColor( diffuse );
    tsphere->addDrawable(sphered);
    mt->addChild(tsphere);
  }
  
  numObjects++;
  m_objects.push_back( mt );
  m_physid.push_back( bh->addSphere( pos, radius, phys ) );
  
  return mt;
}

MatrixTransform* ObjectFactory::addOpenBox( Vec3 pos, Vec3 halfLengths, double innerWidth, bool phys, bool render ) {
  MatrixTransform* mt = new MatrixTransform;
  
  if (render) {
    
    Vec3Array* openBoxVerts = new Vec3Array;
    // Outer
    // 0
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 + halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 + halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 + halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 + halfLengths.y(), 0 + halfLengths.z()) );
    // 4
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 + halfLengths.y(), 0 - halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 + halfLengths.y(), 0 - halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 + halfLengths.y(), 0 - halfLengths.z()) );
    // 7
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 + halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 + halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 + halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 + halfLengths.y(), 0 + halfLengths.z()) );
    // 11
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 + halfLengths.y(), 0 - halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 + halfLengths.y(), 0 - halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 + halfLengths.y(), 0 - halfLengths.z()) );
    // 14
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 - halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 - halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 - halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 - halfLengths.y(), 0 + halfLengths.z()) );
    // 18
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 - halfLengths.y(), 0 - halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 - halfLengths.y(), 0 - halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x(), 0 - halfLengths.y(), 0 - halfLengths.z()) );
    // 21
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 - halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 - halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 - halfLengths.y(), 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 - halfLengths.y(), 0 + halfLengths.z()) );
    // 25
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 - halfLengths.y(), 0 - halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 - halfLengths.y(), 0 - halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x(), 0 - halfLengths.y(), 0 - halfLengths.z()) );
    
    // Inner
    // 28
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 + halfLengths.y() - innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 + halfLengths.y() - innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 + halfLengths.y() - innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 + halfLengths.y() - innerWidth, 0 + halfLengths.z()) );
    // 32
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 + halfLengths.y() - innerWidth, 0 - halfLengths.z() + innerWidth) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 + halfLengths.y() - innerWidth, 0 - halfLengths.z() + innerWidth) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 + halfLengths.y() - innerWidth, 0 - halfLengths.z() + innerWidth) );
    // 35
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 + halfLengths.y() - innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 + halfLengths.y() - innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 + halfLengths.y() - innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 + halfLengths.y() - innerWidth, 0 + halfLengths.z()) );
    // 39
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 + halfLengths.y() - innerWidth, 0 - halfLengths.z() + innerWidth) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 + halfLengths.y() - innerWidth, 0 - halfLengths.z() + innerWidth) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 + halfLengths.y() - innerWidth, 0 - halfLengths.z() + innerWidth) );
    // 42
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 - halfLengths.y() + innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 - halfLengths.y() + innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 - halfLengths.y() + innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 - halfLengths.y() + innerWidth, 0 + halfLengths.z()) );
    // 46
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 - halfLengths.y() + innerWidth, 0 - halfLengths.z() + innerWidth) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 - halfLengths.y() + innerWidth, 0 - halfLengths.z() + innerWidth) );
    openBoxVerts->push_back( Vec3(0 + halfLengths.x() - innerWidth, 0 - halfLengths.y() + innerWidth, 0 - halfLengths.z() + innerWidth) );
    // 49
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 - halfLengths.y() + innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 - halfLengths.y() + innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 - halfLengths.y() + innerWidth, 0 + halfLengths.z()) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 - halfLengths.y() + innerWidth, 0 + halfLengths.z()) );
    // 53
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 - halfLengths.y() + innerWidth, 0 - halfLengths.z() + innerWidth) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 - halfLengths.y() + innerWidth, 0 - halfLengths.z() + innerWidth) );
    openBoxVerts->push_back( Vec3(0 - halfLengths.x() + innerWidth, 0 - halfLengths.y() + innerWidth, 0 - halfLengths.z() + innerWidth) );
    // 56 Points
    
    Geometry * openBoxShape = new Geometry();
    openBoxShape->setVertexArray( openBoxVerts );
    
    //  Left Outer
    DrawElementsUInt* openBoxInd0 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
    openBoxInd0->push_back(0); openBoxInd0->push_back(4); openBoxInd0->push_back(11); openBoxInd0->push_back(7);
    openBoxShape->addPrimitiveSet( openBoxInd0 );
    
    // Back Outer
    DrawElementsUInt* openBoxInd1 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
    openBoxInd1->push_back(8); openBoxInd1->push_back(12); openBoxInd1->push_back(18); openBoxInd1->push_back(14);
    openBoxShape->addPrimitiveSet( openBoxInd1 );
    
    // Right Outer
    DrawElementsUInt* openBoxInd2 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
    openBoxInd2->push_back(15); openBoxInd2->push_back(19); openBoxInd2->push_back(25); openBoxInd2->push_back(21);
    openBoxShape->addPrimitiveSet( openBoxInd2 );
    
    // Front Outer
    DrawElementsUInt* openBoxInd3 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
    openBoxInd3->push_back(22); openBoxInd3->push_back(26); openBoxInd3->push_back(5); openBoxInd3->push_back(1);
    openBoxShape->addPrimitiveSet( openBoxInd3 );
    
    // Bottom Outer
    DrawElementsUInt* openBoxInd4 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
    openBoxInd4->push_back(6); openBoxInd4->push_back(13); openBoxInd4->push_back(20); openBoxInd4->push_back(27);
    openBoxShape->addPrimitiveSet( openBoxInd4 );
    
    if (innerWidth > 0.0) {
      // Left Inner
      DrawElementsUInt* openBoxInd5 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
      openBoxInd5->push_back(28); openBoxInd5->push_back(32); openBoxInd5->push_back(39); openBoxInd5->push_back(35);
      openBoxShape->addPrimitiveSet( openBoxInd5 );
      
      // Back Inner
      DrawElementsUInt* openBoxInd6 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
      openBoxInd6->push_back(36); openBoxInd6->push_back(40); openBoxInd6->push_back(46); openBoxInd6->push_back(42);
      openBoxShape->addPrimitiveSet( openBoxInd6 );
      
      // Right Inner
      DrawElementsUInt* openBoxInd7 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
      openBoxInd7->push_back(43); openBoxInd7->push_back(47); openBoxInd7->push_back(53); openBoxInd7->push_back(49);
      openBoxShape->addPrimitiveSet( openBoxInd7 );
      
      // Front Inner
      DrawElementsUInt* openBoxInd8 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
      openBoxInd8->push_back(50); openBoxInd8->push_back(54); openBoxInd8->push_back(33); openBoxInd8->push_back(29);
      openBoxShape->addPrimitiveSet( openBoxInd8 );
      /*
      // Bottom Inner
      DrawElementsUInt* openBoxInd9 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
      openBoxInd9->push_back(34); openBoxInd9->push_back(41); openBoxInd9->push_back(48); openBoxInd9->push_back(55);
      openBoxShape->addPrimitiveSet( openBoxInd9 );
      */
      // Left Connect
      DrawElementsUInt* openBoxInd10 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
      openBoxInd10->push_back(2); openBoxInd10->push_back(9); openBoxInd10->push_back(37); openBoxInd10->push_back(30);
      openBoxShape->addPrimitiveSet( openBoxInd10 );
      
      // Back Connect
      DrawElementsUInt* openBoxInd11 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
      openBoxInd11->push_back(10); openBoxInd11->push_back(16); openBoxInd11->push_back(44); openBoxInd11->push_back(38);
      openBoxShape->addPrimitiveSet( openBoxInd11 );
      
      // Right Connect
      DrawElementsUInt* openBoxInd12 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
      openBoxInd12->push_back(17); openBoxInd12->push_back(23); openBoxInd12->push_back(51); openBoxInd12->push_back(45);
      openBoxShape->addPrimitiveSet( openBoxInd12 );
      
      // Front Connect
      DrawElementsUInt* openBoxInd13 = new DrawElementsUInt(PrimitiveSet::QUADS, 4);
      openBoxInd13->push_back(24); openBoxInd13->push_back(3); openBoxInd13->push_back(31); openBoxInd13->push_back(52);
      openBoxShape->addPrimitiveSet( openBoxInd13 );
    }
    
    Vec3Array * openBoxNorms = new Vec3Array;
    openBoxNorms->push_back( Vec3(1,0,0) );
    openBoxNorms->push_back( Vec3(-1,0,0) );
    openBoxNorms->push_back( Vec3(0,1,0) );
    openBoxNorms->push_back( Vec3(0,-1,0) );
    openBoxNorms->push_back( Vec3(0,0,1) );
    openBoxNorms->push_back( Vec3(0,0,-1) );
    openBoxShape->setNormalArray( openBoxNorms );
    
    TemplateIndexArray<unsigned int, Array::UIntArrayType, 56, 4> *normalIndexArray;
    normalIndexArray = new TemplateIndexArray<unsigned int, Array::UIntArrayType, 56, 4>();
    // Outer
    normalIndexArray->push_back(3);
    normalIndexArray->push_back(0);
    normalIndexArray->push_back(5);
    normalIndexArray->push_back(5);
    
    normalIndexArray->push_back(3);
    normalIndexArray->push_back(0);
    normalIndexArray->push_back(5);
    
    normalIndexArray->push_back(3);
    normalIndexArray->push_back(1);
    normalIndexArray->push_back(5);
    normalIndexArray->push_back(5);
    
    normalIndexArray->push_back(3);
    normalIndexArray->push_back(1);
    normalIndexArray->push_back(5);
    
    normalIndexArray->push_back(1);
    normalIndexArray->push_back(2);
    normalIndexArray->push_back(5);
    normalIndexArray->push_back(5);
    
    normalIndexArray->push_back(1);
    normalIndexArray->push_back(2);
    normalIndexArray->push_back(5);
    
    normalIndexArray->push_back(2);
    normalIndexArray->push_back(0);
    normalIndexArray->push_back(5);
    normalIndexArray->push_back(5);
    
    normalIndexArray->push_back(2);
    normalIndexArray->push_back(0);
    normalIndexArray->push_back(5);
    
    if (innerWidth > 0.0) {
      // Inner
      normalIndexArray->push_back(3);
      normalIndexArray->push_back(0);
      normalIndexArray->push_back(5);
      normalIndexArray->push_back(5);
      
      normalIndexArray->push_back(3);
      normalIndexArray->push_back(0);
      normalIndexArray->push_back(5);
      
      normalIndexArray->push_back(3);
      normalIndexArray->push_back(1);
      normalIndexArray->push_back(5);
      normalIndexArray->push_back(5);
      
      normalIndexArray->push_back(3);
      normalIndexArray->push_back(1);
      normalIndexArray->push_back(5);
      
      normalIndexArray->push_back(1);
      normalIndexArray->push_back(2);
      normalIndexArray->push_back(5);
      normalIndexArray->push_back(5);
      
      normalIndexArray->push_back(1);
      normalIndexArray->push_back(2);
      normalIndexArray->push_back(5);
      
      normalIndexArray->push_back(2);
      normalIndexArray->push_back(0);
      normalIndexArray->push_back(5);
      normalIndexArray->push_back(5);
      
      normalIndexArray->push_back(2);
      normalIndexArray->push_back(0);
      normalIndexArray->push_back(5);
    }
    
    openBoxShape->setNormalIndices(normalIndexArray);
    openBoxShape->setNormalBinding( Geometry::BIND_PER_VERTEX );
    
    osg::Vec4Array* colors = new osg::Vec4Array;
    for (int i = 0; i < 56; ++i)
        colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );
    openBoxShape->setColorArray(colors);
    openBoxShape->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    
    Geode * obg = new Geode;
    obg->addDrawable( openBoxShape );
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

MatrixTransform* ObjectFactory::addAntiGravityField( Vec3 pos, double halfLength, Vec3 grav, bool phys ) {
  MatrixTransform* mt = new MatrixTransform;
  
  bh->addAntiGravityField( pos, halfLength, grav );
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
    
    Matrixd m;
    for (int i = 0; i < numObjects; ++i) {
      if (m_physid[i] > -1) {
        bh->getWorldTransform( m_physid[i], m );
        m_objects[i]->setMatrix( m );
      }
    }
}

BulletHandler* ObjectFactory::getBulletHandler() {
  return bh;
}

