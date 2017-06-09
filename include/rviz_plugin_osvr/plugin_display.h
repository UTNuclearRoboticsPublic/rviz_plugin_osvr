#pragma once

#include <QObject>
#include <rviz/display.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <rviz_plugin_osvr/ogre_osvr.h>
#include "osvr/ClientKit/ClientKit.h"
//#include <QWidget.h>

namespace rviz{
	class RenderWidget;
}

namespace rviz_plugin_osvr
{
	class PluginDisplay : public rviz::Display, public Ogre::RenderTargetListener
	{
		Q_OBJECT
		public:
			PluginDisplay(void);
			virtual ~PluginDisplay(void);

			//Override methods from rviz::Display
			virtual void onInitialize();
			virtual void update(float wall_dt, float ros_dt);
			
			//Override methods from Ogre::RenderTargetListener
			virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt);
			virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt);

			// PluginDisplay public methods
			void reset();

		protected:
			//Override methods from rviz::Display
			virtual void onEnable();
			virtual void onDisable();
			
			void updateCamera(float wall_dt, float ros_dt);

		protected Q_SLOTS:
			void onFullScreenChanged();

		private:
		
			rviz::BoolProperty *fullscreen_property_;
		
			rviz::RenderWidget *render_widget_;
			osvr::clientkit::ClientContext *osvr_context_;
			Ogre::SceneNode *scene_node_;

			#ifndef Q_MOC_RUN
				OsvrClient *osvr_client_;
			#endif
			
			
	};

}
