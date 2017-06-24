#include "rviz_plugin_osvr/distortion.h"

#include <iostream>
#include <cmath>
#include <string>
#include <ros/console.h>

#include <json/reader.h>
#include <json/value.h>

#include <OGRE/OgreVector2.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgrePlane.h>

namespace rviz_plugin_osvr {

Distortion::Distortion() : desired_triangles_(800), overfill_factor_(1)
{
	//initialize dataset names
	names_.clear();
	for (auto& dataset: OSVR_DISTORTION_DATASETS )
	{
		names_.emplace_back(dataset.key);
	}
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
	double const quad_side = 1.0f / quads_per_side;
//	float quad_tex_side = 1.0f / quads_per_side;

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
			float x_pos = 0 + x * quad_side;
			//float x_pos = -1 + x * quad_side;
//			float x_tex = x * quad_tex_side;

			for (int y = 0; y < num_verts_per_side; y++) 
			{
				float y_pos = 0 + y * quad_side;
				//float y_pos = -1 + y * quad_side;
//				float y_tex = y * quad_tex_side;

				DistortionVertex dv;
				dv.pos = {{x_pos, y_pos}};
				dv.tex = {{0, 0}}; // to be interpolated
				if(computeInterpolatedDistortionVertex(dv, dist_map, overfill_factor_))
				{
					mesh.vertices.push_back(dv);
				}
				else
				{
					// interpolation error, mesh is not consistent any more,
					// stop and get out of here
					return false;
				}
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

		meshes_.push_back(mesh);

	} // loop over maps for each eye 

	return true;
} // end computeMeshes


bool Distortion::computeInterpolatedDistortionVertex(DistortionVertex& dv, const DistortionPointMap& dist_map, double overfill_factor)
{
	// scale input point from overfill space to normalized space 
		dv.pos[0] = (dv.pos[0] - 0.5) * overfill_factor + 0.5; // x
		dv.pos[1] = (dv.pos[1] - 0.5) * overfill_factor + 0.5; // y

	// get nearest points
	DistortionPointMap nearest_points = getNearestPoints(dist_map, dv.pos);
	if(nearest_points.size() != 3)
		return false;

//	dv.tex[0] = (nearest_points[0].tex[0]+nearest_points[1].tex[0]+nearest_points[1].tex[0])/3;
//	dv.tex[1] = (nearest_points[0].tex[1]+nearest_points[1].tex[1]+nearest_points[1].tex[1])/3;
	if(!interpolate(dv, nearest_points[0], nearest_points[1],nearest_points[2]))
		return false;

	//scale back from normalized space to overfill space
	dv.pos[0] = (dv.pos[0] - 0.5) / overfill_factor + 0.5;   // x
	dv.pos[1] = (dv.pos[1] - 0.5) / overfill_factor + 0.5;   // y

	return true;
}

DistortionPointMap Distortion::getNearestPoints(const DistortionPointMap& distortion_map, const Point2D& pos)
{

	DistortionPointMap ret; //fill this vector with maximum of 3 nearest points
	typedef std::multimap<double, size_t> PointDistanceIndexMap;
	PointDistanceIndexMap distance_map;
	for (size_t i=0; i<distortion_map.size(); i++)
	{
		distance_map.insert(std::make_pair(getDistanceBetweenPoints(pos, distortion_map[i].pos), i));
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
//		ROS_WARN("nearlyCollinear set detected");
//		ROS_INFO("p1: %.2f, %.2f", distortion_map[first].pos[0], distortion_map[first].pos[1]);
//		ROS_INFO("p2: %.2f, %.2f", distortion_map[second].pos[0], distortion_map[second].pos[1]);
//		ROS_INFO("p3: %.2f, %.2f", distortion_map[it->second].pos[0], distortion_map[it->second].pos[1]);

		it++;
	}

	ret.push_back(distortion_map[first]);
	ret.push_back(distortion_map[second]);
	if(first!=third)
	{
		ret.push_back(distortion_map[third]);
	}
	return ret;

}


double Distortion::getDistanceBetweenPoints(const Point2D& p1, const Point2D& p2)
{
	return std::sqrt((p1[0]-p2[0]) * (p1[0]-p2[0]) + (p1[1]-p2[1]) * (p1[1]-p2[1]));
}

bool Distortion::nearlyCollinear(const Point2D& p1, const Point2D& p2,const Point2D& p3)
{
	Ogre::Vector2 v1(p2[0]-p1[0], p2[1]-p1[1]);
	Ogre::Vector2 v2(p3[0]-p1[0], p3[1]-p1[1]);
	if(v1.length()*v2.length()==0) // If either vector is zero length, they are collinear
	{
		return true;
	}

	v1.normalise();
	v2.normalise();

	return fabs(v1.dotProduct(v2)) > 0.75;
}

bool Distortion::interpolate(DistortionVertex& dv_interp, const DistortionVertex& dv1, 
		const DistortionVertex& dv2,const DistortionVertex& dv3)
{
//prepare 3D points, with X, Y and U coordinates.
	Ogre::Vector3 p1u(dv1.pos[0], dv1.pos[1], dv1.tex[0]);
	Ogre::Vector3 p2u(dv2.pos[0], dv2.pos[1], dv2.tex[0]);
	Ogre::Vector3 p3u(dv3.pos[0], dv3.pos[1], dv3.tex[0]);

//prepare 3D points, with X, Y and V coordinates.
	Ogre::Vector3 p1v(dv1.pos[0], dv1.pos[1], dv1.tex[1]);
	Ogre::Vector3 p2v(dv2.pos[0], dv2.pos[1], dv2.tex[1]);
	Ogre::Vector3 p3v(dv3.pos[0], dv3.pos[1], dv3.tex[1]);
	
	//Create two planes for each pointset
	Ogre::Plane pu(p1u, p2u, p3u);
	Ogre::Plane pv(p1v, p2v, p3v);

	double u_norm_length = pu.normalise();
	double v_norm_length = pv.normalise();

	if(u_norm_length*v_norm_length == 0)
	{
		return false;
	}

	//ROS_INFO_STREAM("P1: "<<p1v);
	//ROS_INFO_STREAM("P2: "<<p2v);
	//ROS_INFO_STREAM("P3: "<<p3v);
	//ROS_INFO("X,Y IN: %.2f, %.2f", dv_interp.pos[0], dv_interp.pos[1]);
	//ROS_INFO_STREAM("u ABC:"<<pu.normal);
	//ROS_INFO_STREAM("v ABC:"<<pv.normal);

	// Find u and v coordinates for fixed x and y on previously defined planes.
	//Z = -(AX + BY + D)/C;
	dv_interp.tex[0] = -(pu.normal.x*dv_interp.pos[0] + 
						 pu.normal.y*dv_interp.pos[1] + pu.d) / pu.normal.z; //u

	dv_interp.tex[1] = -(pv.normal.x*dv_interp.pos[0] + 
						 pv.normal.y*dv_interp.pos[1] + pv.d) / pv.normal.z; //v

	//ROS_INFO("TEX: %.2f, %.2f", dv_interp.tex[0], dv_interp.tex[1]);
	return  true;
}


} // end namespace rviz_plugion_osvr

