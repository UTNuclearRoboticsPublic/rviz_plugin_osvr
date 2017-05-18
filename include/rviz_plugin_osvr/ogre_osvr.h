#pragma once
#include <string>
#include <QObject>
#include <QWidget>

#include "OGRE/OgreSceneManager.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreCompositorManager.h"
#include "OGRE/OgreCompositorInstance.h"
#include "OGRE/OgreCompositionTargetPass.h"
#include "OGRE/OgreCompositionPass.h"

#include <osvr/ClientKit/ClientKit.h>
#include <osvr/ClientKit/Display.h>

namespace Ogre
{
	class SceneManager;
	class RenderWindow;
	class Camera;
	class SceneNode;
	class Viewport;
	class CompositorInstance;
}

namespace 
{
	const float g_defaultNearClip = 0.01f;
	const float g_defaultFarClip = 10000.0f;
	const float g_defaultIPD = 0.064f;
	const Ogre::ColourValue g_defaultViewportColour(97 / 255.0f, 97 / 255.0f, 200 / 255.0f);
	const float g_defaultProjectionCentreOffset = 0.14529906f;
	const float g_defaultDistortion[4] = {1.0f, 0.22f, 0.24f, 0.0f};
	const float g_defaultChromAb[4] = {0.996, -0.004, 1.014, 0.0f};
}
namespace rviz_plugin_osvr
{

class OsvrClient 
{
public:
  OsvrClient(void); 
  ~OsvrClient(void);
  void onInitialize();
  bool setupOgre(Ogre::SceneManager *sm, Ogre::RenderWindow *win, Ogre::SceneNode *parent);
  void update();

protected:
  Ogre::RenderWindow* window_;
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* camera_node_;
  Ogre::Camera* cameras_[2];
  Ogre::Viewport* viewports_[2];
  Ogre::CompositorInstance* compositors_[2];

  osvr::clientkit::ClientContext osvr_ctx_;
  osvr::clientkit::DisplayConfig osvr_disp_conf_;
};


}
