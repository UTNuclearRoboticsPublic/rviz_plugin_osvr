#include <rviz_plugin_osvr/rviz_plugin_osvr.h>
#include "rviz/display.h"
#include <ros/console.h>

#include <osvr/ClientKit/ClientKit.h>

namespace rviz_plugin_osvr
{
	PlugOSVR::PlugOSVR()
	{
		ROS_INFO("PlugOSVR created");
	}

	PlugOSVR::~PlugOSVR(void)
	{
		ROS_INFO("PlugOSVR destroyed");
	//  shutDownOgre();
	//  shutDownOculus();
	}
	
	void PlugOSVR::onInitialize()
	{
		ROS_INFO("Initializing RViz Plugin for OSVR");

		osvr::clientkit::ClientContext context("com.rviz.plugOSVR");
		ROS_INFO("Initializing OSVR library");

		
	}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_osvr::PlugOSVR, rviz::Display)

