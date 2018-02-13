#pragma once
#include <string>
#include <array>
#include <vector>

#include <QObject>
#include <QWidget>

#include "OGRE/OgreSceneManager.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreVector3.h"
#include "OGRE/OgreQuaternion.h"

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
}

namespace 
{
	const float g_defaultNearClip = 0.01f;
	const float g_defaultFarClip = 10000.0f;
	//IPD Gender  Mean     Min      Max
	//    Female  0.0617   0.051    0.0745
	//    Male    0.064    0.053    0.077

	const float g_defaultIPD = 0.064f; //average male IPD
	const Ogre::ColourValue g_defaultViewportColour(97 / 255.0f, 97 / 255.0f, 200 / 255.0f);
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
			bool connectToServer();
			void setupDistortion();
			bool setupOgre(Ogre::SceneManager *sm, Ogre::RenderWindow *win, Ogre::SceneNode *parent);
			void update();
			bool getPose(Ogre::Vector3& pos, Ogre::Quaternion& ori);

			inline void setPosOffset(const Ogre::Vector3& offset){pos_offset_ = offset;}
			inline Ogre::Vector3 getPosOffset() {return pos_offset_;}
			inline void setPosScale(const Ogre::Vector3& scale){pos_scale_ = scale;}
			void useTracker(bool use_tracker){use_tracker_ = use_tracker;}
      void resetOrientation();

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


			osvr::clientkit::ClientContext* osvr_ctx_;
			osvr::clientkit::DisplayConfig* osvr_disp_conf_;

		private:
			Distortion distortion_;
			Ogre::Vector3 pos_offset_;
			Ogre::Vector3 pos_scale_;
			Ogre::Quaternion ori_offset_;
			bool use_tracker_;

	};



}
