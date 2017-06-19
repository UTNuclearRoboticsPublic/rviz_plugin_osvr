#include "rviz_plugin_osvr/distortion.h"

#include <iostream>
#include <cmath>
#include <string>
#include <ros/console.h>

#include <json/reader.h>
#include <json/value.h>

#include <OGRE/OgreVector2.h>
#include <OGRE/Plane.h>

namespace rviz_plugin_osvr {

Distortion::Distortion() : desired_triangles_(8), overfill_factor_(1.0)
{

}


bool Distortion::parse(const std::string& name)
{
	ROS_INFO_STREAM("Parsing distortion " << name);
	
	std::string data_string;
	bool found = false;
	for (auto& dataset: OSVR_DISTORTION_DATASETS )
	{
		if (name == dataset.key)
		{
			data_string = dataset.data_string;
			found = true;
			break;
		}
	}

	if (!found)
	{
		ROS_WARN_STREAM("Built in distortion \"" << name << "\" not found!");
		return false;
	}

	Json::Reader reader;
	Json::Value data_json;
	if (!reader.parse(data_string, data_json, false))
	{
		ROS_WARN_STREAM("JSON parsing error: " << reader.getFormattedErrorMessages());
		return false;
	}

	auto const& distortion = data_json["display"]["hmd"]["distortion"];
	if (distortion.isNull()) 
	{
		ROS_WARN("Distortion data not found from built in distortion string.");
		return false;
	}


	ROS_INFO("Processing JSON data into mono point samples description structure.");
	Json::Value const& eye_array = distortion["mono_point_samples"];
	if (eye_array.isNull() || eye_array.empty()) {
			ROS_WARN_STREAM("Couldn't find non-empty distortion mono "
					"pointdistortion in data from " << name);
		return false;

	}

	for (auto& point_array : eye_array) {
		DistortionPointMap dist_map;
		if (point_array.empty()) {
			ROS_WARN_STREAM("Empty distortion mono point distortion list for eye "
					"in data from " << name);
			return false;

		}

		for (auto& elt : point_array) {
			if ((elt.size() != 2) || (elt[0].size() != 2) ||
					(elt[1].size() != 2)) {
				ROS_WARN_STREAM("Empty distortion mono point distortion list for eye "
						"in data from " << name);
				return false;
			}

			DistortionVertex v;
			v.pos[0] = (elt[0][0].asDouble());
			v.pos[1] = (elt[0][1].asDouble());
			v.tex[0] = (elt[1][0].asDouble());
			v.tex[1] = (elt[1][1].asDouble());

			dist_map.push_back(v);

		}
		maps_.push_back(dist_map);
		ROS_INFO_STREAM("Loaded distortion map " << maps_.size()-1 << " with "
				<< dist_map.size() << " points in it.");
	}
	
	return true;
}


bool Distortion::computeDistortionMeshes() {


	// Figure out how many quads we should use in each dimension.  The
	// minimum is 1.  We have an even number in each.  There are two
	// triangles per quad.
	int quads_per_side =
		static_cast<int>(std::sqrt(desired_triangles_ / 2));
	if (quads_per_side < 1) {
		quads_per_side = 1;
	}

	// Figure out how large each quad will be.  Recall that we're
	// covering a range of 2 (from -1 to 1) in each dimension, so the
	// quads will all be square in texture space.
	float quad_side = 2.0f / quads_per_side;
	float quad_tex_side = 1.0f / quads_per_side;

	// Compute distorted texture coordinates and use those for each
	// vertex, with appropriate spatial location and texture
	// coordinates.

	auto const num_verts_per_side = quads_per_side + 1;
	auto const num_vertices = num_verts_per_side*num_verts_per_side;


	// go through the distortion maps and create mesh for each map using interpolation
	for (auto& dist_map : maps_)
	{
		DistortionMesh mesh;
		mesh.vertices.reserve(num_vertices);
		// Generate a grid of vertices with distorted texture coordinates
		for (int x = 0; x < num_verts_per_side; x++) 
		{
			float x_pos = -1 + x * quad_side;
//			float x_tex = x * quad_tex_side;

			for (int y = 0; y < num_verts_per_side; y++) 
			{
				float y_pos = -1 + y * quad_side;
//				float y_tex = y * quad_tex_side;

//				DistortionVertex v;
				Point2D pos{x_pos, y_pos};
//				v.tex = {x_tex, y_tex};
				mesh.vertices.emplace_back(computeInterpolatedDistortionVertex(dist_map, pos, overfill_factor_));
			}
		}

		// Generate a pair of triangles for each quad, wound
		// counter-clockwise from the mesh grid

		// total of quadsPerSide * quadsPerSide * 6 vertices added: reserve
		// that space to avoid excess copying during mesh generation.
		mesh.indices.reserve(quads_per_side * quads_per_side * 6);
		for (int x = 0; x < quads_per_side; x++) {
			for (int y = 0; y < quads_per_side; y++) {
				// Grid generated above is in column-major order
				int index_ll = x*num_verts_per_side + y;
				int index_hl = index_ll + num_verts_per_side;
				int index_hh = index_ll + num_verts_per_side + 1;
				int index_lh = index_ll + 1;

				// Triangle 1
				mesh.indices.emplace_back(index_ll);
				mesh.indices.emplace_back(index_hl);
				mesh.indices.emplace_back(index_hh);

				// Triangle 2
				mesh.indices.emplace_back(index_ll);
				mesh.indices.emplace_back(index_hh);
				mesh.indices.emplace_back(index_lh);
			}
		}

	} // loop over maps for each eye 

	return true;
} // end computeMeshes


DistortionVertex Distortion::computeInterpolatedDistortionVertex(const DistortionPointMap& dist_map, const Point2D& pos, double overfill_factor)
{

	Point2D modified_position{{
			(pos[0] - 0.5) * overfill_factor + 0.5,   // x
			(pos[0] - 0.5) * overfill_factor + 0.5}}; // y

	//get nearest points
DistortionPointMap nearest_points = getNearestPoints(dist_map, modified_position)

	DistortionVertex v; 
	v.pos = {0,0};
	v.tex = {0,0};
	return v; 
}

DistortionPointMap Distortion::getNearestPoints(const DistortionPointMap& distortion_map, const Point2D& pos)
{

	DistortionPointMap ret; //fill this vector with maximum of 3 nearest points
	typedef std::multimap<double, size_t> PointDistanceIndexMap;
	PointDistanceIndexMap distance_map;
	for (const auto& v : distortion_map)
	{
		distance_map.insert(std::make_pair(getDistanceBetweenPoints(v.pos,pos), v-v.begin()));
	}

	PointDistanceIndexMap::const_iterator it = distance_map.begin();
	size_t first = it->second;
	it++;
	size_t second = it->second;
	it++;
	size_t third = first;
	while (it != distance_map.end())
	{
		if(!nearlyCollinear(
					distortion_map[first].pos,
					distortion_map[second].pos,
					distortion_map[it->second].pos))
		{
			third = it->second;
			break;
		}
		it++;
	}

	ret.push_back(distortion_map[first]);
	ret.push_back(distortion_map[second]);
	if(first!=third)
	{
		ret.push_back(distortion_map[third]);
	}

}


double Distortion::getDistanceBetweenPoints(const Point2D& p1, const Point2D& p2)
{
	return std::sqrt((p1[0]-p2[0]) * (p1[0]-p2[0]) + (p1[1]-p2[1]) * (p1[1]-p2[1]));
}

bool nearlyCollinear(const Point2D& p1, const Point2D& p2,const Point2D& p3)
{
	Ogre::Vector2 v1(p2[0]-p1[0], p2[1]-p1[1]);
	Ogre::Vector2 v2(p3[0]-p1[0], p3[1]-p1[1]);
	if(v1.length()*v2.length()==0) // If either vector is zero length, they are collinear
	{
		return true;
	}

	v1.normalise();
	v2.normalise();

	return (fabs(v1.crossProduct(v2)) > 0.8)
}

DistortionVertex Distortion::interpolate(const DistortionVertex& dv1, const DistortionVertex& dv2,const DistortionVertex& dv3, const Point2D& pos)
{

	//Create a plane using the given three points
	Ogre::Plane(Ogre::Vector3 )

}


} // end namespace rviz_plugion_osvr

