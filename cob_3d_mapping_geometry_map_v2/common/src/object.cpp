#include <cob_3d_mapping_geometry_map_v2/types/object.h>
#include <cob_3d_mapping_geometry_map_v2/visualization/vis_obj.h>

using namespace cob_3d_geometry_map;


Object::TID Object::generate_id()
{
	static TID running_id = 0;
	return ++running_id;
}

	
void Object3D::visualization(std::vector<boost::shared_ptr<Visualization::Object> > &objs)
{
	objs.push_back( Visualization::Object::Ptr( new Visualization::Sphere(pose_.translation(), 0.1f) ) );
}
