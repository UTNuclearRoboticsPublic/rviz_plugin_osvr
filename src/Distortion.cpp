#include "ComputeDistortionMesh.h"
#include "UnstructuredMeshInterpolator.h"
#include "DistortionCorrectTextureCoordinate.h"

#include <iostream>
#include <cmath>

namespace rviz_plugin_osvr {

	Distortion::Distortion()
	{

	}

	std::vector<std::string> Distortion::getAvailableDistortions()
	{
		return ;
	}

	bool Distortion::parseFromJson(const char * jsonStr)
	{
		ROS_INFO("Parsing distortion mesh ...");

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

    DistortionMesh ComputeDistortionMesh(size_t eye, DistortionParameters distort, float overfillFactor) {
		DistortionMesh ret;

		// Figure out how many quads we should use in each dimension.  The
		// minimum is 1.  We have an even number in each.  There are two
		// triangles per quad.
		int quadsPerSide =
			static_cast<int>(std::sqrt(distort.m_desiredTriangles / 2));
		if (quadsPerSide < 1) {
			quadsPerSide = 1;
		}

		// Figure out how large each quad will be.  Recall that we're
		// covering a range of 2 (from -1 to 1) in each dimension, so the
		// quads will all be square in texture space.
		float quadSide = 2.0f / quadsPerSide;
		float quadTexSide = 1.0f / quadsPerSide;

		// Compute distorted texture coordinates and use those for each
		// vertex, with appropriate spatial location and texture
		// coordinates.

		auto const numVertsPerSide = quadsPerSide + 1;
		auto const numVertices = numVertsPerSide*numVertsPerSide;
		ret.vertices.reserve(numVertices);

		// Generate a grid of vertices with distorted texture coordinates
		for (int x = 0; x < numVertsPerSide; x++) {
			float xPos = -1 + x * quadSide;
			float xTex = x * quadTexSide;

			for (int y = 0; y < numVertsPerSide; y++) {
				float yPos = -1 + y * quadSide;
				float yTex = y * quadTexSide;

				Float2 pos = { xPos, yPos };
				Float2 tex = { xTex, yTex };

				ret.vertices.emplace_back(pos,
						DistortionCorrectTextureCoordinate(eye, tex, distort, 0, overfillFactor, interpolators),
						DistortionCorrectTextureCoordinate(eye, tex, distort, 1, overfillFactor, interpolators),
						DistortionCorrectTextureCoordinate(eye, tex, distort, 2, overfillFactor, interpolators));
			}
		}

		// Generate a pair of triangles for each quad, wound
		// counter-clockwise from the mesh grid

		// total of quadsPerSide * quadsPerSide * 6 vertices added: reserve
		// that space to avoid excess copying during mesh generation.
		ret.indices.reserve(quadsPerSide * quadsPerSide * 6);
		for (int x = 0; x < quadsPerSide; x++) {
			for (int y = 0; y < quadsPerSide; y++) {
				// Grid generated above is in column-major order
				int indexLL = x*numVertsPerSide + y;
				int indexHL = indexLL + numVertsPerSide;
				int indexHH = indexLL + numVertsPerSide + 1;
				int indexLH = indexLL + 1;

				// Triangle 1
				ret.indices.emplace_back(indexLL);
				ret.indices.emplace_back(indexHL);
				ret.indices.emplace_back(indexHH);

				// Triangle 2
				ret.indices.emplace_back(indexLL);
				ret.indices.emplace_back(indexHH);
				ret.indices.emplace_back(indexLH);
                  }
              }
          } break;
        case RADIAL: {
              std::cerr
                  << "ComputeDistortionMesh: Radial mesh type "
                  << "not yet implemented" << std::endl;

              // @todo Scale the aspect ratio of the rings around the center of
              // projection so that they will be round in the visible display.
          } break;
        default:
              std::cerr << "ComputeDistortionMesh: Unsupported "
                  "mesh type: "
                  << type << std::endl;
        }

        return ret;
    }

} // end namespace renderkit
} // end namespace osvr

