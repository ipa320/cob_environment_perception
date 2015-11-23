#pragma once

#include "object.h"
#include "plane.h"
#include "classifier.h"
#include <unsupported/Eigen/BVH>
#include <angelscript.h>
#include <cob_3d_mapping_msgs/PlaneScene.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace cob_3d_geometry_map {
	
class Context {
public:
	typedef float Scalar;
	enum {Dim=3};
	
	template<class TObj, class TPos>
	struct SearchObject {
		TObj obj_;
		TPos pos_;
		
		SearchObject() {}
		SearchObject(const TObj &obj, const TPos &pos) : obj_(obj), pos_(pos) {}
	};
	
	typedef Eigen::KdBVH<Scalar, Dim, SearchObject<Object::Ptr, Eigen::AlignedBox<Scalar, Dim> > > BVH_Volume;
	typedef Eigen::KdBVH<Scalar, Dim, SearchObject<Object::Ptr, Eigen::Vector3f> > BVH_Point;

private:	
	std::vector<Object::Ptr> objs_;
	BVH_Volume search_volume_;
	BVH_Point  search_point_;
	
	asIScriptEngine* script_engine_;
	
public:
	typedef boost::shared_ptr<Context> Ptr;
	
	Context() {}
	void build();
	
	void add_scene(const Ptr &this_ctxt, const cob_3d_mapping_msgs::PlaneScene &);

	void add(const Object::Ptr &);
	void visualize(std::vector<boost::shared_ptr<Visualization::Object> > &vis_objs);
	
	void classify(const Context::Ptr &ctxt, const Classifier::Ptr &classifier);
	void optimize(const Context::Ptr &ctxt);
	
	void volume_search_in_volume(const Eigen::AlignedBox<Scalar, Dim> &bb, std::vector<Object::Ptr> &result);
	void volume_search_in_points(const Eigen::AlignedBox<Scalar, Dim> &bb, std::vector<Object::Ptr> &result);
	
	void intersects(const Intersection_Volume &search_param, std::vector<Object::Ptr> &result);
	
	inline std::vector<Object::Ptr>::iterator begin() {return objs_.begin();}
	inline std::vector<Object::Ptr>::iterator end() {return objs_.end();}
	inline void erase(Object::Ptr ptr) {
		for(size_t i=0; i<objs_.size(); i++)
			if(objs_[i]==ptr) {
				objs_.erase(objs_.begin()+i);
				--i;
			}
	}
};

class Context2D {
public:
    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
	typedef boost::geometry::model::box<point_t> box;
	
	struct ProjectedPolygon {
		typedef boost::shared_ptr<ProjectedPolygon> Ptr;
		
		Object::Ptr obj_;
		Plane_Polygon::Ptr poly_;
		
		static box get_box(const Plane_Polygon::Ptr& poly) {
			box b;
			boost::geometry::envelope(*poly, b);
			return b;
		}
		
		box get_box() const {
			return get_box(poly_);
		}
	};
	
	typedef std::pair<box, ProjectedPolygon::Ptr> value;
	
	typedef boost::shared_ptr<Context2D> Ptr;
	
private:
	struct ObjInfo {	//for default values
		double area;
		
		ObjInfo() : area(0) {}
	};
	
	typedef boost::geometry::index::rtree< value, boost::geometry::index::quadratic<16> > RTREE;
	typedef std::map<Object::Ptr, ObjInfo> TAreaMap;
	
	RTREE rtree_;
	TAreaMap area_;
	Context::Ptr scene_;
	
	void insert(Object::Ptr obj, const Eigen::Matrix3f &proj);
	void insert(Object::Ptr obj, const Plane_Polygon &p);
	void insert(Object::Ptr obj, const std::vector<Plane_Polygon> &ps);
	void _insert(Object::Ptr obj, const std::vector<Plane_Polygon> &ps);
	
public:
	Context2D(const Context::Ptr &ctxt, const Eigen::Matrix3f &proj, const Eigen::Matrix3f &proj_inv, const double far_clipping=4.);
	
	void merge(const Context::Ptr &ctxt, const Eigen::Matrix3f &proj, const Eigen::Matrix3f &proj_inv, const double far_clipping=4.);
	
	void save_as_svg(const std::string &fn) const;
};

class GlobalContext {
	asIScriptEngine* script_engine_;

	Context::Ptr scene_;
	Context2D::Ptr scene_2d_;
	
	Eigen::Matrix3d proj_;
	
	typedef std::vector<Classifier::Ptr> ClassifierSet;
	
	ClassifierSet classifiers_;
	
	void classify(const Context::Ptr &ctxt);
public:

	GlobalContext();
	
	void add_scene(const cob_3d_mapping_msgs::PlaneScene &);
	
	void visualize_markers();
	
	bool registerClassifier(Classifier *c);
	
	inline void set_projection(const Eigen::Matrix3d &proj) {
		proj_ = proj;
	}
};

}
