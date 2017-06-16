#pragma once
#include <string>
#include <array>
#include <vector>

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

#include "rviz_plugin_osvr/distortion.h"

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
	const float g_defaultProjectionCenterOffset = 0.058;
	const float g_defaultDistortion[4] = {1.0f, -1.74f, 5.15f, -1.27f};
	const float g_defaultChromAb[4] = {0.996, -0.004, 1.014, 0.0f};
}

namespace rviz_plugin_osvr
{

    /// Holds a list of mappings from physical-display normalized
	/// coordinates to canonical-display normalized coordinates.
	/// There are an arbitrary number of points describing the mapping.
	/// Each point has a from and to vector.

	typedef std::vector<          //!< Vector of mappings, one per point
		std::array<               //!< 2-array of from, to coordinates
		std::array<double, 2> //!< 2-array of unit coordinates (x,y)
		,
		2> > MonoPointDistortionMeshDescription;

	typedef std::vector< //!< One mapping per eye
		MonoPointDistortionMeshDescription> MonoPointDistortionMeshDescriptions;


	class OsvrClient 
	{
		public:
			OsvrClient(void); 
			~OsvrClient(void);
			void onInitialize();
			bool setupOgre(Ogre::SceneManager *sm, Ogre::RenderWindow *win, Ogre::SceneNode *parent);
			void update(float wall_dt, float ros_dt);

		protected:
			Ogre::RenderWindow* window_;
			Ogre::SceneManager* scene_manager_;
			Ogre::SceneNode* camera_node_;
			Ogre::Camera* cameras_[2];
			Ogre::Viewport* viewports_[2];
			Ogre::TexturePtr textures_[2];
			Ogre::MaterialPtr materials_[2];

			Ogre::SceneManager* external_scene_manager_;
			Ogre::Viewport* external_viewport_;
			Ogre::Camera* external_camera_;

			osvr::clientkit::ClientContext osvr_ctx_;
			osvr::clientkit::DisplayConfig osvr_disp_conf_;

		private:
			Distortion distortion_;
	};



}
