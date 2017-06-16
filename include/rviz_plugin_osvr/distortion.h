#pragma once

//#include <cstddef>      // for size_t
#include <string>      
#include <vector>      
#include <array>      


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

	struct DistortionVertex
	{
		std::array<double, 2> pos; // XY coordinates
		std::array<double, 2> tex; // UV coordinates; Currently, RGB distortion currently not supported
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


		private:
			DistortionPointMaps maps_;
			DistortionMeshes meshes_;
			DistortionNames names_; // Names of available datasets

			unsigned int desired_triangles_;	
	};

} // namespace rviz_plugin_osvr
