//#include <ros/package.h>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
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
#include <rviz/properties/string_property.h>

//#include <osvr/ClientKit/ServerAutoStartC.h>

#include <QWidget>
#include <QWindow>
#include <QScreen>
#include <QApplication>
#include <QGuiApplication>
#include <QDesktopWidget>

namespace rviz_plugin_osvr{

PluginDisplay::PluginDisplay() : osvr_client_(0), 
	render_widget_(0), 
	scene_node_(0), 
	fullscreen_property_(0), 
	fullscreen_name_property_(0), 
	tf_frame_property_(0), 
	pos_offset_property_(0),
	pos_scale_property_(0),
	pub_tf_property_(0),
	pub_tf_frame_property_(0),
	use_tracker_property_(0),
	reset_orientation_property_(0)
	{}

PluginDisplay::~PluginDisplay()
{
	
	ROS_INFO("PluginDisplay::~PluginDisplay()");
	if(osvr_client_)
	{
		delete osvr_client_;
		osvr_client_=0;
		ROS_INFO("removed osvr osvr client");
	}

	if(scene_node_)
	{
		scene_manager_->getRootSceneNode()->removeChild(scene_node_);
		scene_manager_->destroySceneNode(scene_node_);
		scene_node_=0;
		ROS_INFO("deleted scene node");
	}
	if (render_widget_)
	{
		delete render_widget_;
		render_widget_=0;
		ROS_INFO("deleted render widget");
	}

	if (fullscreen_property_) delete fullscreen_property_;
	if (fullscreen_name_property_) delete fullscreen_name_property_;
	if (tf_frame_property_) delete tf_frame_property_;
	if (pos_offset_property_) delete pos_offset_property_;
	if (pos_scale_property_) delete pos_scale_property_;
	if (pub_tf_property_) delete pub_tf_property_;
	if (pub_tf_frame_property_) delete pub_tf_frame_property_;
	if (use_tracker_property_) delete use_tracker_property_;
	if (reset_orientation_property_) delete reset_orientation_property_;

	ROS_INFO("PluginDisplay::~PluginDisplay() ended");
}


void PluginDisplay::onInitialize()
{
	ROS_INFO("PluginDisplay::onInitialize");
	
	// *************
	// Initialize all the plugin properties for rviz
	// *************
	fullscreen_property_ = new rviz::BoolProperty("Fullscreen", false,
		"If checked, will render fullscreen on the selected screen. Otherwise, shows a window.",
		this, SLOT(onFullScreenChanged()));

	fullscreen_name_property_ = new rviz::EnumProperty("Screen name", "Select screen",
		"The name of a screen where osvr window is displayed.",
		this, SLOT(onFullScreenChanged()));
	for(const auto& screen : QGuiApplication::screens())
	{
		fullscreen_name_property_->addOption(screen->name());
	}

	follow_cam_property_ = new rviz::BoolProperty("Follow RViz camera", true,
			"If checked, will follow the pose of RViz main camerai.",
			this, SLOT(onFollowCamChanged()));

	tf_frame_property_ = new rviz::TfFrameProperty("Target frame", "<Fixed Frame>", 
			"Tf frame that VR camera follows", this, context_->getFrameManager(), true);

	pos_offset_property_ = new rviz::VectorProperty("Offset", Ogre::Vector3(0,0,0),
		   "This offset is added to the raw reading obtained from the OSVR IR tracking device."
		   " The offset is added before scaling.", 
		   this, SLOT(onPosOffsetChanged()));

	pos_scale_property_ = new rviz::VectorProperty("Scale", Ogre::Vector3(1,1,1),
		   "With this property you can set the gain of the position tracking.", 
		   this, SLOT(onPosScaleChanged()));

	pub_tf_property_ = new rviz::BoolProperty("Publish tf", true,
		"If checked, will publish the pose of OSVR as a tf frame.",
		this, SLOT(onPubTfChanged()));

	pub_tf_frame_property_ = new rviz::StringProperty("OSVR tf frame", "/rviz_plugin_osvr/head",
		"Name of the published tf frame.", this);

	use_tracker_property_ = new rviz::BoolProperty("Use tracker", true,
		"If checked, will update head position based on OSVR IR tracker data",
		this, SLOT(onUseTrackerChanged()));

	reset_orientation_property_ = new rviz::BoolProperty("Reset orientation", false,
		"If checked, will set current OSVR orientation as inversed offset, so that current orientation will become zero.",
		this, SLOT(onResetOrientationChanged()));


	// *************
	// Initialize osvr window, widget and scenenode
	// *************
	render_widget_ = new rviz::RenderWidget(rviz::RenderSystem::get());
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
		osvr_client_->useTracker(use_tracker_property_->getBool());
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
			screen = scr;
			break;
		}
	}

	if (fullscreen_property_->getBool())
	{
		ROS_INFO_STREAM("OSVR is going fullscreen on "<<screen->name().toUtf8().constData());
		render_widget_->showFullScreen();
	}
	else
	{
		ROS_INFO_STREAM("OSVR is going window mode (" << default_width << " x " << default_height << ") on " << 
				screen->name().toUtf8().constData());
		render_widget_->showNormal();
	}
}


void PluginDisplay::onFollowCamChanged()
{
	  tf_frame_property_->setHidden(follow_cam_property_->getBool());
}


void PluginDisplay::onPubTfChanged()
{
	  pub_tf_frame_property_->setHidden(!pub_tf_property_->getBool());
}


void PluginDisplay::onPosOffsetChanged()
{
	if(osvr_client_)
	{
		//from rviz space to opengl space
		osvr_client_->setPosOffset(Ogre::Vector3(
				-pos_offset_property_->getVector().y,
				 pos_offset_property_->getVector().z,
				-pos_offset_property_->getVector().x
				));
	}
}


void PluginDisplay::onPosScaleChanged()
{
	if(osvr_client_)
	{
		osvr_client_->setPosScale(Ogre::Vector3(
				pos_scale_property_->getVector().y,
				pos_scale_property_->getVector().z,
				pos_scale_property_->getVector().x
				));
	}
}

void PluginDisplay::onUseTrackerChanged()
{
	if(osvr_client_)
	{
		osvr_client_->useTracker(use_tracker_property_->getBool());
	}
}

void PluginDisplay::onResetOrientationChanged()
{
  // make this property act as a button
  reset_orientation_property_->setBool(false);
  if(osvr_client_)
	{
		osvr_client_->resetOrientation();
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
	// VR head pose in OpenGL coordinates
	Ogre::Vector3 pos;
	Ogre::Quaternion ori;
	
	if(follow_cam_property_->getBool())
	{
		//Synchronize rotation and position with the one in rviz window.
		const Ogre::Camera *cam = context_->getViewManager()->getCurrent()->getCamera();
		pos = cam->getDerivedPosition();
		ori = cam->getDerivedOrientation();
	}
	else
	{
		// get reference frame pose
		context_->getFrameManager()->getTransform(tf_frame_property_->getStdString(),
			                                                  ros::Time(), pos, ori);
	    Ogre::Quaternion r; // Rotate from RViz coordinates to OpenGL coordinates
	    r.FromAngleAxis(Ogre::Radian(M_PI*0.5), Ogre::Vector3::UNIT_X);
	    ori = ori*r;
	    r.FromAngleAxis(Ogre::Radian(-M_PI*0.5), Ogre::Vector3::UNIT_Y);
	    ori = ori*r;	
	}

	scene_node_->setPosition(pos);
	scene_node_->setOrientation(ori);

	if(!osvr_client_)
	{
		return;
	}

	osvr_client_->update();

	// publish tf if selected
	if(pub_tf_property_->getBool())
	{
		tf::StampedTransform pose;
		pose.frame_id_ = context_->getFixedFrame().toStdString();
		pose.child_frame_id_ = pub_tf_frame_property_->getStdString();
		pose.stamp_ = ros::Time::now();

		Ogre::Vector3 head_pos;
		Ogre::Quaternion head_ori;
		if(osvr_client_->getPose(head_pos, head_ori))
		{
			ori = ori*head_ori;
			
			pos.x -= head_pos.z;
			pos.y -= head_pos.x;
			pos.z += head_pos.y;

			Ogre::Quaternion r; //Transform from OpenGL space to RViz space
			r.FromAngleAxis(Ogre::Radian(M_PI*0.5), Ogre::Vector3::UNIT_Y);
			ori = ori * r;
			r.FromAngleAxis(Ogre::Radian(-M_PI*0.5), Ogre::Vector3::UNIT_X);
			ori = ori * r;
			
			pose.setRotation(tf::Quaternion(ori.x, ori.y, ori.z, ori.w));
			pose.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
			tf_pub_.sendTransform(pose);
		}
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
