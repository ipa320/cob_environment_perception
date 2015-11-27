#include <cob_3d_mapping_geometry_map_v2/types/object.h>
#include <cob_3d_mapping_geometry_map_v2/types/classifier.h>
#include <cob_3d_mapping_geometry_map_v2/visualization/vis_obj.h>
#include <cmath>

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

void Object::rem_class(const int id) {
	ClassList::iterator it = classes_.find(id);
	if(it!=classes_.end())
		classes_.erase(it);
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

bool ObjectVolume::contains(const ObjectVolume &o) const {
	const Eigen::Affine3f Ta = cast(pose_), Tb = cast(o.pose_);
    const Eigen::Affine3f Tb_in_a = Ta.inverse()*Tb;
    
    for(int i=0; i<8; i++)
		if(!bb_.contains( Tb_in_a*o.bb_.corner( (TBB::CornerType)i ) ))
			return false;
		
	return true;
}

inline double proj(const Eigen::Vector3f &pt, const Eigen::Vector3f &off, const Eigen::Vector3f &n) {
	return (pt-off).dot(n)/n.squaredNorm();
}

bool ObjectVolume::_overlaps(const ObjectVolume &o) const {
	const Eigen::Affine3f Ta = cast(pose_), Tb = cast(o.pose_);
    const Eigen::Affine3f Tb_in_a = Ta.inverse()*Tb;
    
    for(int j=0; j<3; j++) {
		Eigen::Vector3f n(0,0,0);
		n(j) = (bb_.max()-bb_.min())(j);
		
		bool i_sm = false, i_bg = false;
		for(int i=0; (!i_sm || !i_bg) && i<8; i++) {
			const double p = proj(Tb_in_a*o.bb_.corner( (TBB::CornerType)i ), bb_.min(), n);
			i_sm |= p<=1;
			i_bg |= p>=0;
		}
			
		if(!i_sm || !i_bg)
			return false;
	}
	
	return true;
}

bool ObjectVolume::overlaps(const ObjectVolume &o) const {
	//3+3 axes check
	if(!_overlaps(o) || !o._overlaps(*this))
		return false;
    
    //9 calculated axes check
	const Eigen::Affine3f Ta = cast(pose_), Tb = cast(o.pose_);
    for(int j=0; j<3; j++) {
		Eigen::Vector3f n1(0,0,0);
		n1(j) = 1;
		n1 = Ta.rotation()*n1;
		
		for(int k=0; k<3; k++) {
			Eigen::Vector3f n2(0,0,0);
			n2(k) = 1;
			n2 = Tb.rotation()*n2;
			
			const Eigen::Vector3f n = n1.cross(n2);
			
			double i1,i2, a1,a2;
			for(int i=0; i<8; i++) {
				const double p1 = (Ta*  bb_.corner( (TBB::CornerType)i )).dot(n);
				const double p2 = (Tb*o.bb_.corner( (TBB::CornerType)i )).dot(n);
				if(i) {
					i1 = std::min(i1, p1);
					i2 = std::min(i2, p2);
					a1 = std::max(a1, p1);
					a2 = std::max(a2, p2);
				}
				else {
					i1=a1=p1;
					i2=a2=p2;
				}				
			}
			
			if( i2>a1 || a2<i1 )
				return false;
		}
	}
	
	return true;
}
