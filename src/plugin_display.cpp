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

#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>

//#include <osvr/ClientKit/Context.h>
//#include <osvr/ClientKit/Display.h>
//#include <osvr/ClientKit/DisplayConfig.h>
//#include <osvr/ClientKit/Parameters.h>
#include <osvr/ClientKit/ServerAutoStartC.h>

#include <QWidget>
#include <QWindow>
#include <QScreen>
//#include <QRect>
#include <QApplication>
#include <QGuiApplication>
#include <QDesktopWidget>

namespace rviz_plugin_osvr{

PluginDisplay::PluginDisplay() : osvr_client_(0), render_widget_(0), scene_node_(0), 
		fullscreen_property_(0)
{
//	Ogre::MaterialManager::getSingleton().setVerbose(true);
}

PluginDisplay::~PluginDisplay()
{
	ROS_INFO("PluginDisplay::~PluginDisplay()");
	delete osvr_client_;
	osvr_client_=0;
	ROS_INFO("removed osvr osvr client");

	ROS_INFO("removed resource path from ogre resource manager");
	if(scene_node_)
	{
		delete scene_node_;
		scene_node_=0;
		ROS_INFO("deleted scene node");
	}
	if (render_widget_)
	{
		delete render_widget_;
		render_widget_=0;
		ROS_INFO("deleted render widget");
	}


	ROS_INFO("PluginDisplay::~PluginDisplay() ended");
}


void PluginDisplay::onInitialize()
{
	ROS_INFO("PluginDisplay::onInitialize");

	// initialize all the plugin properties in rviz
	fullscreen_property_ = new rviz::BoolProperty("Full Screen", false,
		"If checked, will render fullscreen. Otherwise, shows a window.",
		this, SLOT(onFullScreenChanged()));

	fullscreen_name_property_ = new rviz::EnumProperty("Screen name", "Select screen",
		"The name of a screen where osvr context appears",
		this, SLOT(onFullScreenChanged()));
	for(const auto& screen : QGuiApplication::screens())
	{
		fullscreen_name_property_->addOption(screen->name());
	}

	tf_frame_property_ = new rviz::TfFrameProperty("Target Frame", "<Fixed Frame>", 
			"Tf frame that VR camera follows", this, context_->getFrameManager(), true);

	offset_property_ = new rviz::VectorProperty("Offset", Ogre::Vector3(0,0,0),
		   "Additional offset of the VR camera from the followed RViz camera or target frame.", this);


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
	window->setAutoUpdated(false);
	window->addListener(this);

	scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

	
}


void PluginDisplay::onEnable()
{
	
	ROS_INFO("Enabling %s", ROS_PACKAGE_NAME);
	if(!render_widget_ || !scene_node_)
	{
		ROS_ERROR("Enabling plugin failed, because render_widget_ or scene_node_ is NULL");
		return;
	}

	Ogre::RenderWindow *window = render_widget_->getRenderWindow();
	if(window)
	{
		window->setVisible(true);
	}
	else
	{
		ROS_ERROR("Enabling plugin failed, because getRenderWindow() returned NULL");
		return;
	}

	if(!osvr_client_)
	{
		// Start osvr server, create our osvr client, and try to connect.
		// The default server config is expected at ~/.config/osvr/osvr_server_config.json
		//
		// osvrClientAttemptServerAutoStart(); 
		//
		// Cannot use that, due to linux support for osvrStartProcess()
		// in inc/osvr/Util/ProcessUtils.h is not implemented yet.
		
		osvr_client_ = new OsvrClient();
		osvr_client_->setupDistortion();
		osvr_client_->setupOgre(scene_manager_, window, scene_node_);
	}

}



void PluginDisplay::onDisable()
{
	
	ROS_INFO("Disabling %s", ROS_PACKAGE_NAME);

	Ogre::RenderWindow *window = render_widget_->getRenderWindow();
	if(window)
	{
		window->setVisible(false);
	}

	if(osvr_client_)
	{
		// osvrClientReleaseAutoStartedServer();
		// not yet supported in linux
		
		delete osvr_client_;
		osvr_client_=0;
	}
}


void PluginDisplay::onFullScreenChanged()
{

	QScreen* screen = QGuiApplication::primaryScreen();
	int default_width = 1280;
	int default_height = 800;
	for(const auto& scr : QGuiApplication::screens())
	{
		// Identify user selected screen name from the list
		if(scr->name() == fullscreen_name_property_->getString())
		{
			render_widget_->setGeometry(
					scr->availableGeometry().x(),
					scr->availableGeometry().y(),
					default_width,
					default_height);
			ROS_INFO("%d %d",scr->availableGeometry().x(), scr->availableGeometry().y());
			//render_widget_->saveGeometry();
			screen = scr;
			break;
		}
	}

	if (fullscreen_property_->getBool())
	{
		ROS_INFO_STREAM("Going fullscreen on "<<screen->name().toUtf8().constData());
		render_widget_->showFullScreen();
	}
	else
	{
		ROS_INFO_STREAM("Going windowed mode (" << default_width << " x " << default_height << ") on " << 
				screen->name().toUtf8().constData());
		render_widget_->showNormal();
	}
}



void PluginDisplay::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
}


void PluginDisplay::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
	//ROS_INFO("postRender callback");
	Ogre::RenderWindow *window = render_widget_->getRenderWindow();
	window->swapBuffers();
}


void PluginDisplay::update(float wall_dt, float ros_dt)
{
	//ROS_INFO("PluginDisplay update");
	updateCamera(wall_dt, ros_dt);
	render_widget_->getRenderWindow()->update(false);
}

void PluginDisplay::updateCamera(float wall_dt, float ros_dt)
{
	//Synchronize rotation and position of the scene in rviz window.
	const Ogre::Camera *cam = context_->getViewManager()->getCurrent()->getCamera();
	scene_node_->setPosition(cam->getDerivedPosition());
	scene_node_->setOrientation(cam->getDerivedOrientation());
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

} //rviz_plugin_osvr namespace


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_osvr::PluginDisplay, rviz::Display)
