#include <boost/operators.hpp>
#include <rviz_plugin_osvr/ogre_osvr.h>
#include "rviz/display.h"
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_system.h>
#include <ros/console.h>

#include "OGRE/OgreMatrix4.h"
#include "OGRE/OgreSceneManager.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreCompositorManager.h"
#include "OGRE/OgreCompositorInstance.h"
#include "OGRE/OgreCompositionTargetPass.h"
#include "OGRE/OgreCompositionPass.h"
#include "OGRE/OgreHardwarePixelBuffer.h"
#include "OGRE/OgreManualObject.h"

#include <osvr/ClientKit/ClientKit.h>
#include <osvr/ClientKit/Display.h>
#include <osvr/Util/Pose3C.h>


#include "rviz_plugin_osvr/ogre_osvr.h"

namespace rviz_plugin_osvr
{
	OsvrClient::OsvrClient() : window_(0), scene_manager_(0), camera_node_(0),
			osvr_ctx_("com.rviz.plugOSVR"), osvr_disp_conf_(osvr_ctx_)
	{
		for(int i=0;i<2;i++)
		{
			cameras_[i] = 0;
			external_camera_ = 0;
			viewports_[i] = 0;
			external_viewport_ = 0;
		}
		ROS_INFO("OsvrClient created");
	}

	OsvrClient::~OsvrClient(void)
	{
		ROS_INFO("OsvrClient destroyed");

		if(scene_manager_)
		{
			if(camera_node_)
			{	
				scene_manager_->destroySceneNode(camera_node_);
				camera_node_=0;
			}
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
		}
		if(window_)
		{
			for(int i=0;i<2;i++)
			{
				Ogre::CompositorManager::getSingletonPtr()->removeCompositorChain(viewports_[i]);
			}

			window_->removeAllViewports();
			for(int i=0;i<2;i++)
			{
				viewports_[i]=0;
				//compositors_[i]=0;
			}
			//TODO: remove materials and textures
		}
	}
	
	void OsvrClient::onInitialize()
	{
		ROS_INFO("Initializing RViz Plugin for OSVR");
		DistortionNames dist_names = distortion_.getDatasetNames();
		ROS_INFO("Available distortions:");
		for(auto& dist_name : dist_names)
		{
			ROS_INFO_STREAM("/t"<<dist_name);
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
		
		// Create external scenemanager and viewport for distortion mesh.  
		external_scene_manager_ = rviz::RenderSystem::get()->root()->createSceneManager(Ogre::ST_GENERIC);
		external_scene_manager_ ->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
		
		external_camera_ = external_scene_manager_ -> createCamera("OsvrCameraExternal");
		external_camera_ -> setFarClipDistance(50);
		external_camera_ -> setNearClipDistance(0.001);
		external_camera_ -> setProjectionType(Ogre::PT_ORTHOGRAPHIC);
		external_camera_ -> setOrthoWindow(2,1);

		external_viewport_ = win->addViewport(external_camera_);
		external_viewport_ -> setBackgroundColour(g_defaultViewportColour);
		external_viewport_ -> setOverlaysEnabled(true);

		

		for (int i = 0; i < 2; ++i)
		{
			// Setup cameras
			cameras_[i] = sm->createCamera(i==0 ? "OsvrCameraLeft" : "OsvrCameraRight");
			camera_node_->attachObject(cameras_[i]);
			cameras_[i]->setNearClipDistance(g_defaultNearClip);
			cameras_[i]->setFarClipDistance(g_defaultFarClip);
			cameras_[i]->setPosition((i * 2 - 1) * g_defaultIPD * 0.5f, 0, 0);
			cameras_[i]->setFOVy(Ogre::Degree(90));


			// Create Textures and materials for viewports
			textures_[i] = Ogre::TextureManager::getSingleton().createManual(
					i==0 ? "OsvrTextureLeft" : "OsvrTextureRight",
					"rviz_plugin_osvr",
					Ogre::TEX_TYPE_2D,
					1182,
					1461,
					0,
					Ogre::PF_R8G8B8,
					Ogre::TU_RENDERTARGET);
			
			materials_[i] = Ogre::MaterialManager::getSingleton().create(
					i==0 ? "OsvrMaterialLeft" : "OsvrMaterialRight",
					"rviz_plugin_osvr");
			Ogre::Pass* pass = materials_[i]->getTechnique(0)->getPass(0);
			pass->createTextureUnitState();
			pass->getTextureUnitState(0)->setTexture(textures_[i]);

			// Create viewport for each camera
			viewports_[i] = textures_[i]->getBuffer()->getRenderTarget()->addViewport(cameras_[i]);
			viewports_[i]->setBackgroundColour(g_defaultViewportColour);
			viewports_[i]->setClearEveryFrame(true);
			viewports_[i]->setOverlaysEnabled(false);
			viewports_[i]->setShadowsEnabled(true);
		}

		Ogre::SceneNode* meshNode = external_scene_manager_ -> getRootSceneNode() -> createChildSceneNode();
		const DistortionMeshes dist_meshes = distortion_.getMeshes();


		unsigned int eyeIdx = 0;
		for(auto& dist_mesh : dist_meshes) // One mesh per eye
		{
			if (eyeIdx > 1)
				break; //limited to two meshes currently
			
			Ogre::ManualObject *manObj = external_scene_manager_->createManualObject(
					(eyeIdx==0) ? "OsvrObjectLeft" : "OsvrObjectRight");
			manObj->begin((eyeIdx==0) ? "OsvrMaterialLeft" : "OsvrMaterialRight",
				   	Ogre::RenderOperation::OT_LINE_STRIP);
			manObj->colour(1,1,1);
			
			for(auto& vert : dist_mesh.vertices)
			{
				manObj->position(vert.pos[0] - eyeIdx, vert.pos[1], 0.0);
				manObj->textureCoord(vert.tex[0], vert.tex[1]);
			}

			manObj->end();
			meshNode->attachObject(manObj);
			eyeIdx++;
		}

		// Move mesh vertically to the center and towards negative z-direction.
		meshNode->setPosition(0,-5,-1);

		//pass->setFragmentProgram("OsvrFragmentProgram");
		//pass->setVertexProgram("OsvrVertexProgram");

		//Ogre::GpuProgramParametersSharedPtr shader_params = pass->getFragmentProgramParameters();
		//shader_params->setNamedConstant("LensCenter", 0.5f + g_defaultProjectionCenterOffset / 
		//		2.0f * (i==0) ? 1 : -1);
		//pass->setFragmentProgram("OsvrFragmentProgram");
		//pass->setVertexProgram("OsvrVertexProgram");

		//Ogre::GpuProgramParametersSharedPtr shader_params = pass->getFragmentProgramParameters();
		//shader_params->setNamedConstant("LensCenter", 0.5f + g_defaultProjectionCenterOffset / 
		//		2.0f * (i==0) ? 1 : -1);

		//extObjRight->begin("OsvrMaterialRight", Ogre::RenderOperation::OT_TRIANGLE_LIST);



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
			

//		cameras_[0] = sm->createCamera("CameraLeft");
//		cameras_[1] = sm->createCamera("CameraRight");

//		Ogre::MaterialPtr matLeft = Ogre::MaterialManager::getSingleton().getByName("Ogre/Compositor/Osvr");
//		Ogre::MaterialPtr matRight = matLeft->clone("Ogre/Compositor/Osvr/Right");

//		Ogre::Vector4 hmdchrom = Ogre::Vector4(g_defaultChromAb);
//		pParamsLeft->setNamedConstant("ChromAbParam", hmdchrom);
//		pParamsRight->setNamedConstant("ChromAbParam", hmdchrom);

//		Ogre::CompositorPtr comp = Ogre::CompositorManager::getSingleton().getByName("OsvrRight");
//		comp->getTechnique(0)->getOutputTargetPass()->getPass(0)->setMaterialName("Ogre/Compositor/Osvr/Right");
			//compositors_[i] = Ogre::CompositorManager::getSingleton().addCompositor(viewports_[i],
			//												   i == 0 ? "OsvrLeft" : "OsvrRight");
			//compositors_[i]->setEnabled(true);
			//

//		Ogre::GpuProgramParametersSharedPtr pParamsRight =
//			matRight->getTechnique(0)->getPass(0)->getFragmentProgramParameters();

		//warp describes the transformation from the rectangular image to warped image, which suits for the head mounted display
//		pParamsRight->setNamedConstant("HmdWarpParam", hmdwarp);
//		pParamsRight->setNamedConstant("LensCenter", 0.5f - g_defaultProjectionCenterOffset / 2.0f );
//
		return true;
	}

	void OsvrClient::update(float wall_dt, float ros_dt)
	{
	    //const Ogre::Camera *cam = context_->getViewManager()->getCurrent()->getCamera();
	    //pos = cam->getDerivedPosition();
	    //ori = cam->getDerivedOrientation();
		//camera_node_->setOrientation(getOrientation());
		
		if (osvr_ctx_.checkStatus())
		{
			osvr_ctx_.update();
		}
		if (!osvr_disp_conf_.valid()) return;



		int eye_idx=0;
		osvr_disp_conf_.forEachEye([&](osvr::clientkit::Eye eye)
		{
			if (eye_idx < 2)  
			{
				eye.forEachSurface([&](osvr::clientkit::Surface surface)
				{
					try
					{
					ROS_INFO("COP: %f", 
							surface.getRadialDistortion().centerOfProjection.data[0]);
					}
					catch(std::runtime_error e ){}
				});


				double osvr_view_mat[OSVR_MATRIX_SIZE];
				if(eye.getViewMatrix(OSVR_MATRIX_COLMAJOR|OSVR_MATRIX_COLVECTORS,osvr_view_mat))
				{
					Ogre::Matrix4 ogre_view_mat;

					for(int i=0;i<4;i++)
					for (int j=0;j<4;j++)
					ogre_view_mat[j][i] = osvr_view_mat[i*4+j];
				//	cameras_[eye_idx]->setCustomViewMatrix(true,ogre_view_mat.inverse());

				//	Ogre::Matrix4 ident = Ogre::Matrix4::IDENTITY;
				//	ident.setTrans(Ogre::Vector3(0,0,-2));

					OSVR_Pose3 pose;
					eye.getPose(pose);
					Ogre::Quaternion ori;
					ori.w = (Ogre::Real)osvrQuatGetW(&pose.rotation);
					ori.x = -(Ogre::Real)osvrQuatGetX(&pose.rotation);
					ori.y = -(Ogre::Real)osvrQuatGetY(&pose.rotation);
					ori.z = (Ogre::Real)osvrQuatGetZ(&pose.rotation);
					ori.normalise();

//					ROS_INFO("rot: %f %f %f %f",ori.x, ori.y, ori.z, ori.w);
					
					Ogre::Vector3 pos(
							(Ogre::Real)pose.translation.data[0],
							(Ogre::Real)pose.translation.data[1],
							(Ogre::Real)pose.translation.data[2]);


					//Ogre::Quaternion oneEightyTurnY(Ogre::Degree(180),Ogre::Vector3::UNIT_Y);
					//Ogre::Quaternion oneEightyTurnZ(Ogre::Degree(180),Ogre::Vector3::UNIT_Z);

					cameras_[eye_idx]->setOrientation(ori);
					//cameras_[eye_idx]->setOrientation(oneEightyTurnZ*oneEightyTurnY*ori);
//					Ogre::Quaternion curOri = cameras_[eye_idx]->getOrientation();
//					cameras_[eye_idx]->setOrientation(newOri*curOri);
//					cameras_[eye_idx]->setPosition(pos);
//					ROS_INFO_STREAM("POS: "<<pos);
				}
			}
			eye_idx++;
		});
	}

}


