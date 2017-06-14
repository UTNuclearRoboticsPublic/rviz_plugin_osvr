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

#include <osvr/ClientKit/ClientKit.h>
#include <osvr/ClientKit/Display.h>
#include <osvr/Util/Pose3C.h>

#include <rviz_plugin_osvr/osvr_display_config_built_in_osvr_hdks.h>
#include <json/reader.h>
#include <json/value.h>

#include "rviz_plugin_osvr/ogre_osvr.h"

namespace rviz_plugin_osvr
{
	OsvrClient::OsvrClient() : window_(0), scene_manager_(0), camera_node_(0),
			osvr_ctx_("com.rviz.plugOSVR"), osvr_disp_conf_(osvr_ctx_)
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
				compositors_[i]=0;
			}
		}
	}
	
	void OsvrClient::onInitialize()
	{
		ROS_INFO("Initializing RViz Plugin for OSVR");
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
			camera_node = sm->getRootSceneNode()->createChildSceneNode("StereoCameraNode");
		}
		

		cameras_[0] = sm->createCamera("CameraLeft");
		cameras_[1] = sm->createCamera("CameraRight");

		Ogre::MaterialPtr matLeft = Ogre::MaterialManager::getSingleton().getByName("Ogre/Compositor/Osvr");
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
		pParamsLeft->setNamedConstant("LensCenter", 0.5f + g_defaultProjectionCenterOffset / 2.0f );
		pParamsRight->setNamedConstant("LensCenter", 0.5f - g_defaultProjectionCenterOffset / 2.0f );

		Ogre::CompositorPtr comp = Ogre::CompositorManager::getSingleton().getByName("OsvrRight");
		comp->getTechnique(0)->getOutputTargetPass()->getPass(0)->setMaterialName("Ogre/Compositor/Osvr/Right");

		//TODO limited to 2 cameras for testing purpose, should be extended to whatever number of cameras and viewports is declared from OSVR
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

		external_scene_manager_ = rviz::RenderSystem::get()->root()->createSceneManager(Ogre::ST_GENERIC);
		external_scene_manager_ ->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

		parseDistortionMeshes();


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

	
	MonoPointDistortionMeshDescriptions OsvrClient::parseDistortionMeshes()
	{

		ROS_INFO("Loading Distortion Mesh ...");

		MonoPointDistortionMeshDescriptions mesh; /// Mesh, to be returned.
		auto withError = [&] // Return empty mesh in case of errors
		{
			ROS_INFO("FAILED, returning empty mesh!");
			mesh.clear();
			return mesh;
		};

		struct BuiltInKeysAndData {
			const char* key;
			const char* dataString;
		};

		static const std::initializer_list<BuiltInKeysAndData>
			BUILT_IN_MONO_POINT_SAMPLES = {
				{"OSVR_HDK_13_V1", osvr_display_config_built_in_osvr_hdk13_v1},
				{"OSVR_HDK_13_V2", osvr_display_config_built_in_osvr_hdk13_v2},
				{"OSVR_HDK_20_V1", osvr_display_config_built_in_osvr_hdk20_v1}};
		
		std::string builtInString;
		bool found = false;
		const std::string builtInKey = "OSVR_HDK_20_V1"; //TODO: make this user selectable
		for (auto& knownEntry : BUILT_IN_MONO_POINT_SAMPLES) 
		{
			if (builtInKey == knownEntry.key)
			{
				builtInString = knownEntry.dataString;
				found = true;
				break;
			}
		}

		if (!found)
		{
			ROS_WARN_STREAM("Built in distortion " << builtInKey << " not found!");
			return withError();
		}

		Json::Reader reader;
		Json::Value builtInData;
		if (!reader.parse(builtInString, builtInData, false))
		{
			ROS_WARN_STREAM("JSON parsing error: " << reader.getFormattedErrorMessages());
			return withError();
		}

		auto const& distortion = builtInData["display"]["hmd"]["distortion"];
		if (distortion.isNull()) 
		{
			ROS_WARN("Distortion data not found from built in distortion string.");
			return withError();
		}


		ROS_INFO("Processing JSON data into mono point samples description structure.");
		Json::Value const& eyeArray = distortion["mono_point_samples"];
		if (eyeArray.isNull() || eyeArray.empty()) {
				ROS_WARN_STREAM("Couldn't find non-empty distortion mono "
						"pointdistortion in data from " << builtInKey);
			return withError();

		}

		for (auto& pointArray : eyeArray) {
			MonoPointDistortionMeshDescription eye;
			if (pointArray.empty()) {
				ROS_WARN_STREAM("Empty distortion mono point distortion list for eye "
						"in data from " << builtInKey);
				return withError();

			}
			using Double2 = std::array<double, 2>;
			using Double2x2 = std::array<Double2, 2>;
			for (auto& elt : pointArray) {
				Double2x2 point;
				if ((elt.size() != 2) || (elt[0].size() != 2) ||
						(elt[1].size() != 2)) {
					ROS_WARN_STREAM("Empty distortion mono point distortion list for eye "
							"in data from " << builtInKey);
					return withError();
				}

				Double2 in, out;
				in[0] = (elt[0][0].asDouble());
				in[1] = (elt[0][1].asDouble());
				out[0] = (elt[1][0].asDouble());
				out[1] = (elt[1][1].asDouble());
				point[0] = (in);
				point[1] = (out);
				eye.push_back(point);

			}
			mesh.push_back(eye);

		}
		ROS_INFO_STREAM("Initial processing complete. Loaded mono point samples data with "
				<< mesh[0].size() << " and " << mesh[1].size()
				<< " samples per eye, respectively.");
		return mesh;

	}

}


