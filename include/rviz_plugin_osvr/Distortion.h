#pragma once

#include <cstddef>      // for size_t
#include <string>      
#include <vector>      
#include <array>      

namespace rviz_plugin_osvr {

//	using Double2 = std::array<double, 2>;
//	using Double2x2 = std::array<double, 2>;

	struct DistortionVertex
	{
		std::array<double, 2> pos; // XY coordinates
		std::array<double, 2> tex; // UV coordinates
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
			DistortionNames getAvailableDistortions();
			DistortionMeshes ComputeDistortionMeshes();


		private:
			DistortionPointMaps maps_;
			DistortionMeshes meshes_;
	};

} // namespace rviz_plugin_osvr
