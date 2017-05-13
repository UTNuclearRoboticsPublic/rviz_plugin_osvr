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
#include <QApplication>
#include <QDesktopWidget>

namespace rviz_plugin_osvr{

	PluginDisplay::PluginDisplay() : osvr_client_(0), osvr_context_(0), render_widget_(0), scene_node_(0)
	{
		Ogre::MaterialManager::getSingleton().setVerbose(true);
		std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
		Ogre::ResourceGroupManager* rm = Ogre::ResourceGroupManager::getSingletonPtr();
		rm->addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME);
		rm->initialiseResourceGroup(ROS_PACKAGE_NAME);
		osvrClientAttemptServerAutoStart();
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

		
	}



	void PluginDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
	{
		ROS_INFO("preRender callback");
		updateCamera();
	}


	void PluginDisplay::onEnable()
	{
		ROS_INFO("PluginDisplay enabled");

		if(!osvr_client_)
		{
			Ogre::RenderWindow *window = render_widget_->getRenderWindow();
			osvr_client_ = new OsvrClient();
			osvr_client_->setupOgre(scene_manager_, window, scene_node_);
		}
		
		//if (!osvr_context_)
		//{
		//	osvr_context_ = new osvr::clientkit::ClientContext("com.osvr.rviz_plugin_osvr");
		//	std::string disp_desc = osvr_context_->getStringParameter("/display");
		//	ROS_INFO_STREAM("disp desc " << disp_desc);
		//}

		int x_res = 1280;
		int y_res = 800;
		if (osvr_client_)
		{ 
			int primary_screen = QApplication::desktop()->primaryScreen();
			QRect screen_res = QApplication::desktop()->screenGeometry( primary_screen  );
			render_widget_->setGeometry( screen_res.x(), screen_res.y(), x_res, y_res  );
			render_widget_->showNormal();   
		}
	}


	void PluginDisplay::onDisable()
	{
		ROS_INFO("PluginDisplay disabled");
		if(osvr_client_)
		{
			delete osvr_client_;
			osvr_client_=0;
		}


		if(osvr_context_)
		{
			delete osvr_context_;
			osvr_context_=0;
		}
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
		scene_node_->setPosition(pos+Ogre::Vector3(0,0,0));
		scene_node_->setOrientation(ori);
		if(osvr_client_)
		{
			osvr_client_->update();
		}
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
