#include <cob_3d_mapping_geometry_map_v2/visualization/vis_obj.h>

using namespace cob_3d_geometry_map::Visualization;


void Object::_serialize(cob_3d_visualization::RvizMarker &scene)
{
	scene.color(color()(0), color()(1), color()(2), color()(3));
	scene.ns(name_);
}

void Sphere::serialize(Marker &stream)
{
	cob_3d_visualization::RvizMarker scene;
	_serialize(scene);
	scene.sphere(pos_, r_);
}

void Mesh::serialize(Marker &stream)
{
	cob_3d_visualization::RvizMarker scene;
	_serialize(scene);
	for(size_t i=0; i<indices_.size(); i++)
		scene.addTriangle(
			verts_[indices_[i].ind[0]],
			verts_[indices_[i].ind[1]],
			verts_[indices_[i].ind[2]]
		);
}
