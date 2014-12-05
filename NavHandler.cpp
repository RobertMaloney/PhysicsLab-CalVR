#include "NavHandler.h"
#include <stdio.h>

NavHandler::NavHandler( BulletHandler * bfact, Vec3 start ) {
  goLeft = goRight = goUp = goDown = jumping = false;
  
  bh = bfact;
  person = bh->addBox( start, Vec3(50,50,200), true);
}

NavHandler::~NavHandler() {}
    
void NavHandler::keyEvent( KeyboardInteractionEvent * kp ) {
  
  if (kp->getInteraction() == KEY_DOWN) {
    switch (kp->getKey()) {
        case A:
            goLeft = true;
            break;
        case W:
            goUp = true;
            break;
        case D:
            goRight = true;
            break;
        case S:
            goDown = true;
            break;
        case SPACE:
            jumping = true;
            break;
    }
  } else if (kp->getInteraction() == KEY_UP) {
    switch (kp->getKey()) {
        case A:
            goLeft = false;
            break;
        case W:
            goUp = false;
            break;
        case D:
            goRight = false;
            break;
        case S:
            goDown = false;
            break;
        case SPACE:
            jumping = false;
            break;
    }
  }
}

Matrixd NavHandler::getMovement() {
  static bool inAir = false;
  Vec3 x_vec(MAX_VELOCITY,0,0);
  Vec3 y_vec(0,MAX_VELOCITY,0);
  
  Vec3 mov;
  if (goLeft)  mov -= x_vec;
  if (goRight) mov += x_vec;
  if (goUp)    mov += y_vec;
  if (goDown)  mov -= y_vec;
  if (jumping && !inAir) {
    std::cout << "Jumping\n";
    inAir = true;
    //bh->setLinearVelocity( person, Vec3(0,0,MAX_VELOCITY) );
  } else if (!jumping && inAir) {
    std::cout << "Can Jump\n";
    inAir = false;
  }
  
  bh->translate( person, mov );
  Matrixd m;
  bh->getWorldTransform( person, m );
  return (m);
}
