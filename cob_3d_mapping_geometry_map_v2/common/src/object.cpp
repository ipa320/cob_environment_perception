#include <cob_3d_mapping_geometry_map_v2/types/object.h>
#include <cob_3d_mapping_geometry_map_v2/types/classifier.h>
#include <cob_3d_mapping_geometry_map_v2/visualization/vis_obj.h>

using namespace cob_3d_geometry_map;


Object::TID Object::generate_id()
{
	static TID running_id = 0;
	return ++running_id;
}
	
bool Object::has_class(const int id) const {
	return classes_.find(id)!=classes_.end();
}

void Object::add_class(const boost::shared_ptr<Class> &cl) {
	assert(!has_class(cl->class_id()));
	classes_[cl->class_id()] = cl;
}

std::string Object::vis_name(const std::string &prefix) const {
	std::string r=prefix;
	for(ClassList::const_iterator it=classes_.begin(); it!=classes_.end(); it++)
		r+=","+it->second->name();
	return r;
}
	
void Object3D::visualization(std::vector<boost::shared_ptr<Visualization::Object> > &objs)
{
	Parent::visualization(objs);
	
	objs.push_back( Visualization::Object::Ptr( new Visualization::Sphere(vis_name("Object3D"), cast(pose_.loc_), 0.1f) ) );
}

void ObjectVolume::visualization(std::vector<boost::shared_ptr<Visualization::Object> > &objs)
{
	Parent::visualization(objs);
	
	objs.push_back( Visualization::Object::Ptr( new Visualization::Box(vis_name("ObjectVolume"), cast(pose_), bb_) ) );
}

