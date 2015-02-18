#ifndef _PHYSICSLAB_H
#define _PHYSICSLAB_H

// CVR
#include <cvrKernel/CVRPlugin.h>
#include <cvrKernel/PluginHelper.h>
#include <cvrKernel/SceneManager.h>
#include <cvrKernel/ComController.h>
#include <cvrKernel/Navigation.h>
#include <cvrMenu/MenuSystem.h>
#include <cvrMenu/SubMenu.h>
#include <cvrMenu/MenuButton.h>
#include <cvrKernel/InteractionEvent.h>

// OSG
#include <osg/Group>
#include <osg/Vec3>
#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Material>
#include <osg/StateSet>

// Local
#include "ObjectFactory.h"

enum KEYS {
  W = 119,
  A = 97,
  S = 115,
  D = 100,
  SPACE = 32
};

class PhysicsLab : public cvr::CVRPlugin, public cvr::MenuCallback
{
  protected:
    cvr::SubMenu * _mainMenu, * _loadMenu;
    cvr::MenuButton * _removeButton;

  public:
    PhysicsLab();
    virtual ~PhysicsLab();
    void menuCallback(cvr::MenuItem*);
    bool init();
    void preFrame();
    bool processEvent(cvr::InteractionEvent*);
    
  private:
    ObjectFactory * of;
};

#endif
