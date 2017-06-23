#pragma once

//#include <cstddef>      // for size_t
#include <string>      
#include <vector>      
#include <array>      
#include <cmath>      


#include <rviz_plugin_osvr/osvr_display_config_built_in_osvr_hdks.h>

namespace rviz_plugin_osvr {

	struct BuiltInKeysAndData {
		const char* key;
		const char* data_string;
	};

	static const std::initializer_list<BuiltInKeysAndData>
		OSVR_DISTORTION_DATASETS = {
			{"OSVR_HDK_13_V1", osvr_display_config_built_in_osvr_hdk13_v1},
			{"OSVR_HDK_13_V2", osvr_display_config_built_in_osvr_hdk13_v2},
			{"OSVR_HDK_20_V1", osvr_display_config_built_in_osvr_hdk20_v1}};

	using Point2D = std::array<double, 2>;	

	struct DistortionVertex
	{
		Point2D pos; // XY coordinates
		Point2D tex; // UV coordinates; Currently, RGB distortion currently not supported
	};

	struct DistortionMesh
	{
		std::vector<DistortionVertex> vertices;
		std::vector<unsigned int> indices;
	};


	using DistortionMeshes = std::vector<DistortionMesh>; // one mesh for each eye/viewport
	
	// Array of points that describe the distortion mapping between XY and UV coordinates
	using DistortionPointMap = std::vector<DistortionVertex>;
	using DistortionPointMaps = std::vector<DistortionPointMap>; // one map for each eye/viewport
	using DistortionNames = std::vector<std::string>;

	class Distortion
	{
		public:
			Distortion();
			bool parse(const std::string& dataset_name);
			bool computeDistortionMeshes();
			inline const DistortionNames getDatasetNames(){return names_;};
			inline const DistortionMeshes getMeshes(){return meshes_;};
			inline const DistortionPointMaps getDistortionPointMaps(){return maps_;};
			
			static bool computeInterpolatedDistortionVertex(
					DistortionVertex& dist_vert,
					const DistortionPointMap& distortion_map, 
					double overfill_factor);
			
			static DistortionPointMap getNearestPoints(
					const DistortionPointMap& distortion_map, 
					const Point2D& pos);
			
			static bool interpolate(
					DistortionVertex& dv_interp,
					const DistortionVertex& dv1,
					const DistortionVertex& dv2,
					const DistortionVertex& dv3);

			static double getDistanceBetweenPoints(const Point2D& p1, const Point2D& p2);
			static bool nearlyCollinear(const Point2D& p1, const Point2D& p2,const Point2D& p3);

		private:
			DistortionPointMaps maps_;
			DistortionMeshes meshes_;
			DistortionNames names_; // Names of available datasets

			unsigned int desired_triangles_;	
			double overfill_factor_;
	};

} // namespace rviz_plugin_osvr
