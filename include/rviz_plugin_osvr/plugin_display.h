#pragma once

#include <QObject>
#include <rviz/display.h>
#include <OGRE/OgreRenderTargetListener.h>

#include <osvr/ClientKit/ClientKit.h>
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
			virtual void onEnable();
			virtual void onDisable();
			
			//Override methods from Ogre::RenderTargetListener
			virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt);
			virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt);

			// PluginDisplay public methods
			void updateCamera();
			void reset();

		private:
			rviz::RenderWidget *render_widget_;
			osvr::clientkit::ClientContext *osvr_context_;
			
	};

}
