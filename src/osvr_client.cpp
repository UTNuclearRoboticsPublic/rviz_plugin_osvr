#include <boost/operators.hpp>
#include <rviz_plugin_osvr/osvr_client.h>
#include "rviz/display.h"
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_system.h>
#include <ros/console.h>
#include <ros/package.h>

#include "OGRE/OgreMatrix4.h"
#include "OGRE/OgreSceneManager.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreCompositorManager.h"
#include "OGRE/OgreCompositorInstance.h"
#include "OGRE/OgreCompositionTargetPass.h"
#include "OGRE/OgreCompositionPass.h"
#include "OGRE/OgreHardwarePixelBuffer.h"
#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreTextureUnitState.h"
#include "OGRE/OgreMaterialManager.h"
#include "OGRE/OgreTextureManager.h"

#include <osvr/ClientKit/ClientKit.h>
#include <osvr/ClientKit/Display.h>
#include <osvr/Util/Pose3C.h>


namespace rviz_plugin_osvr
{
	OsvrClient::OsvrClient() :
		window_(0), 
		scene_manager_(0), 
		camera_node_(0),
		osvr_ctx_(0), 
		osvr_disp_conf_(0),
		pos_offset_(Ogre::Vector3(0,0,0)),
		pos_scale_(Ogre::Vector3(1,1,1)),
		use_tracker_(false)
	{


		// Initialize resource group
		Ogre::ResourceGroupManager* res_mgr = Ogre::ResourceGroupManager::getSingletonPtr();
		res_mgr->createResourceGroup(ROS_PACKAGE_NAME);
		res_mgr->initialiseResourceGroup(ROS_PACKAGE_NAME);

		for(int i=0;i<2;i++)
		{
			cameras_[i] = 0;
			viewports_[i] = 0;
		}

		external_camera_ = 0;
		external_viewport_ = 0;
		ROS_INFO("OsvrClient created");
	}

	
	OsvrClient::~OsvrClient(void)
	{
		if(window_)
		{
			window_->removeViewport(0);
		}
		
		for(int i=0;i<2;i++)
		{
			materials_[i]->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
			textures_[i]->getBuffer()->getRenderTarget()->removeViewport(0);
			viewports_[i]=0;
			Ogre::MaterialManager::getSingleton().remove(materials_[i]->getName());
			materials_[i].setNull();
			Ogre::TextureManager::getSingleton().remove(textures_[i]->getName());
			textures_[i].setNull();
		}

		if(scene_manager_)
		{
			if(cameras_[0])
			{
				scene_manager_->destroyCamera(cameras_[0]);
				cameras_[0]=0;
			}
			if(cameras_[1])
			{
				scene_manager_->destroyCamera(cameras_[1]);
				cameras_[1]=0;
			}
			if(camera_node_)
			{	
				scene_manager_->destroySceneNode(camera_node_);
				camera_node_=0;
			}
				viewports_[0]=0;
		}

		if(osvr_disp_conf_)
		{
			delete osvr_disp_conf_;
			osvr_disp_conf_=0;
		}	

		if(osvr_ctx_) 
		{
			delete osvr_ctx_;
			osvr_ctx_=0;
		}

		// Destroy resourcegroup with its resources
		Ogre::ResourceGroupManager* res_mgr = Ogre::ResourceGroupManager::getSingletonPtr();
		res_mgr->destroyResourceGroup(ROS_PACKAGE_NAME);
		
		ROS_INFO("OsvrClient destroyed");
	}


	bool OsvrClient::connectToServer()
	{
		if (!osvr_ctx_)
		{
			osvr_ctx_ = new osvr::clientkit::ClientContext("com.rviz.plugOSVR");
		}

		if(!osvr_ctx_ || !osvr_ctx_->checkStatus())
		{
			ROS_WARN_DELAYED_THROTTLE(1,"osvr_client::connect_to_server: Context not valid");
			return false;
		}

		if(!osvr_disp_conf_)
		{
			osvr_disp_conf_ = new osvr::clientkit::DisplayConfig(*osvr_ctx_);
		}

		if(!osvr_disp_conf_ || !osvr_disp_conf_->valid())
		{
			ROS_WARN_DELAYED_THROTTLE(1,"OsvrClient::connectToServer: DisplayConfig not valid");
			return false;
		}

		return true;
	}
	

	void OsvrClient::setupDistortion()
	{
		ROS_INFO("Setting up osvr distortion");
		DistortionNames dist_names = distortion_.getDatasetNames();
		ROS_INFO("Available distortions:");
		for(auto& dist_name : dist_names)
		{
			ROS_INFO_STREAM("--->>>"<<dist_name);
		}

		distortion_.parse(dist_names[2]); //TODO: fixed choice
		if (distortion_.computeDistortionMeshes())
		{
			ROS_INFO("onInitialize(): Distortion mesh generated.");
		}
		else
		{
			ROS_INFO("onInitialize(): Failed to generate distortion mesh.");
		}
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
			camera_node_ = sm->getRootSceneNode()->createChildSceneNode("StereoCameraNode");
		}
		
		// Create external scenemanager, ortho camera, and viewport for distortion mesh.  
		external_scene_manager_ = rviz::RenderSystem::get()->root()->createSceneManager(Ogre::ST_GENERIC);
		external_scene_manager_ ->setAmbientLight(Ogre::ColourValue(1, 1, 1));
		
		external_camera_ = external_scene_manager_ -> createCamera("OsvrCameraExternal");
		external_camera_ -> setFarClipDistance(50);
		external_camera_ -> setNearClipDistance(0.001);
		external_camera_ -> setProjectionType(Ogre::PT_ORTHOGRAPHIC);
		external_camera_ -> setOrthoWindow(2,1);
		//180 rotation for HDK2
		external_camera_ ->setOrientation(Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_Z));

		external_viewport_ = win->addViewport(external_camera_);
		external_viewport_ -> setBackgroundColour(g_defaultViewportColour);
		external_viewport_ -> setOverlaysEnabled(true);

		
		// Prepare cameras and viewports for each eye
		for (int i = 0; i < 2; ++i)
		{
			// Setup cameras
			cameras_[i] = sm->createCamera(i==0 ? "OsvrCameraLeft" : "OsvrCameraRight");
			camera_node_->attachObject(cameras_[i]);
			cameras_[i]->setNearClipDistance(g_defaultNearClip);
			cameras_[i]->setFarClipDistance(g_defaultFarClip);
			cameras_[i]->setPosition((i * 2 - 1) * g_defaultIPD * 0.5f, 0, 0);
			cameras_[i]->setAspectRatio(1200.0/2160.0*2); 
			cameras_[i]->setFOVy(Ogre::Degree(92));


			// Create Textures and materials for viewports
			textures_[i] = Ogre::TextureManager::getSingleton().createManual(
					i==0 ? "OsvrTextureLeft" : "OsvrTextureRight",
					ROS_PACKAGE_NAME,
					Ogre::TEX_TYPE_2D,
					1182,
					1461,
					0,
					Ogre::PF_R8G8B8,
					Ogre::TU_RENDERTARGET);
			
			materials_[i] = Ogre::MaterialManager::getSingleton().create(
					i==0 ? "OsvrMaterialLeft" : "OsvrMaterialRight",
					ROS_PACKAGE_NAME);
			Ogre::Pass* pass = materials_[i]->getTechnique(0)->getPass(0);
			Ogre::TextureUnitState* unit_state = pass->createTextureUnitState();
			unit_state->setTexture(textures_[i]);
			unit_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_BORDER);


			// Create viewport for each stereo camera.
			viewports_[i] = textures_[i]->getBuffer()->getRenderTarget()->addViewport(cameras_[i]);
			viewports_[i]->setBackgroundColour(g_defaultViewportColour);
			viewports_[i]->setClearEveryFrame(true);
			viewports_[i]->setOverlaysEnabled(false);
			viewports_[i]->setShadowsEnabled(true);
		}

		// Create external scene node for holding distortion meshes. 
		Ogre::SceneNode* meshNode = external_scene_manager_ -> getRootSceneNode() -> createChildSceneNode();
		const DistortionMeshes dist_meshes = distortion_.getMeshes();

		unsigned int eyeIdx = 0;
		for(auto& dist_mesh : dist_meshes) // Loop over meshes, one per each eye.
		{
			if (eyeIdx > 1)
				break; // Hard limit for maximum of two meshes/eyes.
			
			Ogre::ManualObject *manObj = external_scene_manager_->createManualObject(
					(eyeIdx==0) ? "OsvrObjectLeft" : "OsvrObjectRight");
			manObj->begin((eyeIdx==0) ? "OsvrMaterialLeft" : "OsvrMaterialRight",
				   	Ogre::RenderOperation::OT_TRIANGLE_LIST);

			//set vertices
			for(auto& vert : dist_mesh.vertices)
			{
				// place vertices along space X:[0..2], Y:[0..1]
				manObj->position(vert.pos[0]+eyeIdx, vert.pos[1], 0.0);

				// assign uv coordinates, transform from 
				// bottom-left convention (osvr) to top-left convention (ogre3d)
				manObj->textureCoord(vert.tex[0], 1-vert.tex[1]);
			}

			// set indices (grouped by three representing triangles)
			for(auto& idx : dist_mesh.indices)
			{
				manObj->index(idx);
			}


			manObj->end();
			meshNode->attachObject(manObj);
			eyeIdx++;
		}

		// Move mesh to viewport center (in range X:[-1..1], Y:[-0.5..0.5]) and
		// push it towards negative z-direction (away from external camera).
		meshNode->setPosition(-1,-0.5,-1);

		return true;
	}

	void OsvrClient::update()
	{
		if(osvr_ctx_)
		{
			osvr_ctx_->update();
		}
		
		if (!connectToServer())
		{
			return; // The server is probably not fully running yet
		}


		Ogre::Vector3 pos;
		Ogre::Quaternion ori;
		if(getPose(pos, ori))
		{
			camera_node_->setOrientation(ori);
			camera_node_->setPosition(pos);
		}
	}


	bool OsvrClient::getPose(Ogre::Vector3& pos, Ogre::Quaternion& ori)	
	{
		if (!connectToServer())
		{
			return false; // The server is probably not fully running yet
		}

		if (osvr_disp_conf_->getNumViewers()>0)
		{
			osvr::clientkit::Viewer viewer = osvr_disp_conf_->getViewer(0);
			OSVR_Pose3 pose;
			if(!viewer.getPose(pose))
				return false;

			
			// if tracker is enabled, get its position.
			if(use_tracker_)
			{
				pos.x = (Ogre::Real)pose.translation.data[0];
				pos.y =	(Ogre::Real)pose.translation.data[1];
				pos.z =	(Ogre::Real)pose.translation.data[2];
			}
			else
			{
				pos.x = 0;
				pos.y = 0;
				pos.z = 0;
			}
			pos+=pos_offset_;
			pos*=pos_scale_;

			ori.w = (Ogre::Real)osvrQuatGetW(&pose.rotation);
			ori.x = (Ogre::Real)osvrQuatGetX(&pose.rotation);
			ori.y = (Ogre::Real)osvrQuatGetY(&pose.rotation);
			ori.z = (Ogre::Real)osvrQuatGetZ(&pose.rotation);
			ori.normalise();
      ori = ori_offset_ * ori;
    }
		return true;
	}

  void OsvrClient::resetOrientation()
  {
    Ogre::Vector3 pos;
    Ogre::Quaternion ori;
    getPose(pos,ori);
    ori_offset_ = (ori_offset_.Inverse() * ori).Inverse();
  }
} // namespace rviz_plugin_osvr
