#pragma once

#include "object.h"
#include <unsupported/Eigen/BVH>
#include <angelscript.h>
#include <cob_3d_mapping_msgs/PlaneScene.h>

namespace cob_3d_geometry_map {
	
class Context {
	typedef float Scalar;
	enum {Dim=3};
	
	typedef Eigen::KdBVH<Scalar, Dim, Eigen::AlignedBox<Scalar, Dim> > BVH_Volume;
	typedef Eigen::KdBVH<Scalar, Dim, Eigen::Vector3f > BVH_Point;
	
	std::vector<Object::Ptr> objs_;
	BVH_Volume search_volume_;
	BVH_Point  search_point_;
	
	asIScriptEngine* script_engine_;
	
public:
	typedef boost::shared_ptr<Context> Ptr;
	
	Context() {}
	
	void add_scene(const Ptr &this_ctxt, const cob_3d_mapping_msgs::PlaneScene &);

	void add(const Object::Ptr &);
};

class GlobalContext {
	asIScriptEngine* script_engine_;

	std::vector<Context::Ptr> locals_;
	Context::Ptr global_;
	
public:

	GlobalContext();
	
	void add_scene(const cob_3d_mapping_msgs::PlaneScene &);
};

}
