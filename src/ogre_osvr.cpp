#include <boost/operators.hpp>
#include <rviz_plugin_osvr/ogre_osvr.h>
#include "rviz/display.h"
#include <rviz/display_context.h>
#include <ros/console.h>

#include "OGRE/OgreSceneManager.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreCompositorManager.h"
#include "OGRE/OgreCompositorInstance.h"
#include "OGRE/OgreCompositionTargetPass.h"
#include "OGRE/OgreCompositionPass.h"

#include <osvr/ClientKit/ClientKit.h>
#include <osvr/ClientKit/Display.h>

#include <Ogre.h>
#include "rviz_plugin_osvr/ogre_osvr.h"

namespace rviz_plugin_osvr
{
	OsvrClient::OsvrClient() : window_(0), scene_manager_(0), camera_node_(0)
	{
		for(int i=0;i<2;i++)
		{
			cameras_[i]=0;
			viewports_[i]=0;
			compositors_[i]=0;
		}
		ROS_INFO("OsvrClient created");
	}

	OsvrClient::~OsvrClient(void)
	{
		ROS_INFO("OsvrClient destroyed");
	}
	
	void OsvrClient::onInitialize()
	{
		ROS_INFO("Initializing RViz Plugin for OSVR");
		//osvr::clientkit::ClientContext context("com.rviz.plugOSVR");
		//ROS_INFO("OSVR context created");

		//osvr::clientkit::DisplayConfig display;
		//do {
		//	ROS_INFO("OSVR: Waiting for display config");
		//	context.update();
		//	display = osvr::clientkit::DisplayConfig(context);
		//} while (!display.valid());

		
//		ROS_INFO("OSVR: Display config is valid");
//		ROS_INFO("Showing osvr plugin window...");
	}

	bool OsvrClient::setupOgre(Ogre::SceneManager *sm, Ogre::RenderWindow *win, Ogre::SceneNode *parent)
	{
		ROS_INFO("Setting up OGRE...");
		window_ = win;
		scene_manager_ = sm;
		if(parent)
		{
			camera_node_ = parent->createChildSceneNode("StereoCameraNode");
		}
		else
		{
			sm->getRootSceneNode()->createChildSceneNode("StereoCameraNode");
		}
		

		cameras_[0] = sm->createCamera("CameraLeft");
		cameras_[1] = sm->createCamera("CameraRight");

		Ogre::MaterialPtr matLeft = Ogre::MaterialManager::getSingleton().getByName("Ogre/Compositor/Osvr");
		ROS_INFO("matLeft isNull: %x\n",matLeft.isNull());


		Ogre::MaterialPtr matRight = matLeft->clone("Ogre/Compositor/Osvr/Right");
		  
		Ogre::GpuProgramParametersSharedPtr pParamsLeft =
			matLeft->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
		Ogre::GpuProgramParametersSharedPtr pParamsRight =
			matRight->getTechnique(0)->getPass(0)->getFragmentProgramParameters();

		//warp describes the transformation from the rectangular image to warped image, which suits for the head mounted display
		Ogre::Vector4 hmdwarp = Ogre::Vector4(g_defaultDistortion[0], g_defaultDistortion[1], g_defaultDistortion[2],
				                            g_defaultDistortion[3]);
		pParamsLeft->setNamedConstant("HmdWarpParam", hmdwarp);
		pParamsRight->setNamedConstant("HmdWarpParam", hmdwarp);

//		Ogre::Vector4 hmdchrom = Ogre::Vector4(g_defaultChromAb);
//		pParamsLeft->setNamedConstant("ChromAbParam", hmdchrom);
//		pParamsRight->setNamedConstant("ChromAbParam", hmdchrom);

		pParamsLeft->setNamedConstant("LensCenter", 0.5f );
		pParamsRight->setNamedConstant("LensCenter", 0.5f );

		Ogre::CompositorPtr comp = Ogre::CompositorManager::getSingleton().getByName("OsvrRight");
		comp->getTechnique(0)->getOutputTargetPass()->getPass(0)->setMaterialName("Ogre/Compositor/Osvr/Right");

		for (int i = 0; i < 2; ++i)
		{
			camera_node_->attachObject(cameras_[i]);
			//if (m_stereoConfig)
			//{
			//	// Setup cameras.
			//	m_cameras[i]->setNearClipDistance(m_stereoConfig->GetEyeToScreenDistance());
			//	m_cameras[i]->setFarClipDistance(g_defaultFarClip);
			//	m_cameras[i]->setPosition((i * 2 - 1) * m_stereoConfig->GetIPD() * 0.5f, 0, 0);
			//	m_cameras[i]->setAspectRatio(m_stereoConfig->GetAspect());
			//	m_cameras[i]->setFOVy(Ogre::Radian(m_stereoConfig->GetYFOVRadians()));
			//		                                   
			//}
			//else
			//{
				cameras_[i]->setNearClipDistance(g_defaultNearClip);
				cameras_[i]->setFarClipDistance(g_defaultFarClip);
				cameras_[i]->setPosition((i * 2 - 1) * g_defaultIPD * 0.5f, 0, 0);
				cameras_[i]->setFOVy(Ogre::Radian(1.2));
			//}

			viewports_[i] = win->addViewport(cameras_[i], i, 0.5f * i, 0, 0.5f, 1.0f);
			viewports_[i]->setBackgroundColour(g_defaultViewportColour);
			compositors_[i] = Ogre::CompositorManager::getSingleton().addCompositor(viewports_[i],
															   i == 0 ? "OsvrLeft" : "OsvrRight");
			compositors_[i]->setEnabled(true);
								
		}


		return true;
	}

	void OsvrClient::update()
	{
	    //const Ogre::Camera *cam = context_->getViewManager()->getCurrent()->getCamera();
	    //pos = cam->getDerivedPosition();
	    //ori = cam->getDerivedOrientation();
		//camera_node_->setOrientation(getOrientation());
		ROS_INFO("Children: %d",camera_node_->numChildren());

	}

}


