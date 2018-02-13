#pragma once

#include <QObject>
#include <OGRE/OgreRenderTargetListener.h>

#include <rviz/display.h>
#include <rviz_plugin_osvr/osvr_client.h>

#include <tf/transform_broadcaster.h>

namespace rviz{
	class RenderWidget;
	class BoolProperty;
	class EnumProperty;
	class TfFrameProperty;
	class VectorProperty;
	class StringProperty;
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
			void onPubTfChanged();
			void onFollowCamChanged();
			void onPosOffsetChanged();
			void onPosScaleChanged();
			void onUseTrackerChanged();
			void onResetOrientationChanged();
			

		private:
		
			rviz::BoolProperty *fullscreen_property_;
			rviz::EnumProperty *fullscreen_name_property_;
			rviz::TfFrameProperty *tf_frame_property_;
			rviz::VectorProperty *pos_offset_property_;
			rviz::VectorProperty *pos_scale_property_;
			rviz::BoolProperty *pub_tf_property_;
			rviz::StringProperty *pub_tf_frame_property_;
			rviz::BoolProperty *follow_cam_property_;
			rviz::BoolProperty *use_tracker_property_;
			rviz::BoolProperty *reset_orientation_property_;
			
			tf::TransformBroadcaster tf_pub_;			
		
			rviz::RenderWidget *render_widget_;
			Ogre::SceneNode *scene_node_;

			#ifndef Q_MOC_RUN
				OsvrClient *osvr_client_;
			#endif
			
			
	};

}
