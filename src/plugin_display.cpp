//#include <ros/package.h>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderWindow.h>

#include <rviz_plugin_osvr/plugin_display.h>
#include <rviz/window_manager_interface.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/frame_manager.h>

#include <osvr/ClientKit/Context.h>
#include <osvr/ClientKit/Parameters.h>
#include <osvr/ClientKit/ServerAutoStartC.h>

#include <QWidget>

namespace rviz_plugin_osvr{

	PluginDisplay::PluginDisplay()
	{
		osvrClientAttemptServerAutoStart();
		osvr_context_ = new osvr::clientkit::ClientContext("com.osvr.rviz_plugin_osvr");

	}

	PluginDisplay::~PluginDisplay()
	{
		delete osvr_context_;
		osvrClientReleaseAutoStartedServer();
	}


	void PluginDisplay::onInitialize()
	{
		ROS_INFO("onInitialize");
		render_widget_ = new rviz::RenderWidget(rviz::RenderSystem::get());
		render_widget_->setVisible(false);
		render_widget_->setWindowTitle("OSVR View");

		render_widget_->setParent(context_->getWindowManager()->getParentWindow());
		render_widget_->setWindowFlags(
				Qt::Window | 
				Qt::CustomizeWindowHint | 
				Qt::WindowTitleHint | 
				Qt::WindowMaximizeButtonHint);
		Ogre::RenderWindow *window = render_widget_->getRenderWindow();
		window->setVisible(true);
		window->setAutoUpdated(true);
		window->addListener(this);

		scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

		update(0,0);
		render_widget_->setVisible(true);

		
	}



	void PluginDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
	{
		ROS_INFO("preRender callback");
		updateCamera();
	}


	void PluginDisplay::onEnable()
	{
		ROS_INFO("PluginDisplay enabled");
		std::string disp_desc = osvr_context_->getStringParameter("/display");
		ROS_INFO_STREAM("disp desc " << disp_desc);
	}


	void PluginDisplay::onDisable()
	{
		ROS_INFO("PluginDisplay disabled");
	}

	void PluginDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
	{
		ROS_INFO("postRender callback");
		Ogre::RenderWindow *window = render_widget_->getRenderWindow();
		window->swapBuffers();
	}


	void PluginDisplay::update(float wall_dt, float ros_dt)
	{
		ROS_INFO("PluginDisplay update");
		updateCamera();
		Ogre::RenderWindow *window = render_widget_->getRenderWindow();
		window->update(false);
	}

	void PluginDisplay::updateCamera()
	{
		ROS_INFO("PluginDisplay updateCamera");

		const Ogre::Camera *cam = context_->getViewManager()->getCurrent()->getCamera();
		Ogre::Vector3 pos = cam->getDerivedPosition();
		Ogre::Quaternion ori = cam->getDerivedOrientation();
		scene_node_->setPosition(pos+Ogre::Vector3(10,0,10));
		scene_node_->setOrientation(ori);

	}

	void PluginDisplay::reset()
	{
		rviz::Display::reset();
		onDisable();
		onEnable();
	}
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_osvr::PluginDisplay, rviz::Display)
