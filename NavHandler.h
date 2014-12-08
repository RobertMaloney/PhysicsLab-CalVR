#ifndef _NAVHANDLER_H
#define _NAVHANDLER_H

// OSG:
#include <osg/Vec3>
#include <osg/Matrixd>

// CVR:
#include <cvrKernel/InteractionEvent.h>

#include "BulletHandler.h"

#define MAX_VELOCITY 30

using namespace osg;
using namespace cvr;

enum KEYS {
  W = 119,
  A = 97,
  S = 115,
  D = 100,
  SPACE = 32
};

class NavHandler {
  public:
    NavHandler( BulletHandler *, Vec3 );
    virtual ~NavHandler( );
    
    void keyEvent( KeyboardInteractionEvent * );
    Matrixd getMovement();
    int getButton();
    
  private:
    bool goLeft, goRight, goUp, goDown, jumping;
    int person;
    BulletHandler * bh;
};
#endif
