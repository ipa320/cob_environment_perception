#pragma once

#include "object.h"
#include "plane.h"
#include "classifier.h"

namespace Eigen {
     namespace internal {
		 template<class T>
         Eigen::AlignedBox<float,3> bounding_box(const T &bb) { return bb.pos_; }
     }
}

#include <unsupported/Eigen/BVH>
#include <cob_3d_mapping_msgs/PlaneScene.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>

//! interfaces of cob_3d_geometry_map (v2)
namespace cob_3d_geometry_map {
	
class Context2D;

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
	
	Context2D *ctxt2d_;
	
public:
	typedef boost::shared_ptr<Context> Ptr;
	typedef boost::shared_ptr<const Context> ConstPtr;
	
	Context() : ctxt2d_(NULL) {}
	void build();
	
	void set_context_2d(Context2D *ctxt2d) {ctxt2d_=ctxt2d;}
	
	void add_scene(const Ptr &this_ctxt, const cob_3d_mapping_msgs::PlaneScene &, const std::vector<Image::Ptr> &);

	void add(const Object::Ptr &);
	void visualize(std::vector<boost::shared_ptr<Visualization::Object> > &vis_objs);
	
	void classify(const Context::Ptr &ctxt, const Classifier::Ptr &classifier, const bool single_shot);
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
	
	bool fitModel(const Object &model, double &score_coverage, double &score_matching);
};

class TransformationEstimator {
public:

	virtual bool register_scene(const Context::ConstPtr &new_scene, const Context::ConstPtr &map, nuklei::kernel::se3 &tf_out) = 0;
};


class Projector_Viewport : public Projector {
	Eigen::Matrix4f T_;
public:
	Projector_Viewport() {}
	Projector_Viewport(const Eigen::Matrix4f &T) : T_(T) {
	}
	
	void setT(const Eigen::Matrix4f &T) {T_ = T;}
	
	virtual Vector2 operator()(const nuklei_wmf::Vector3<double> &pt3) const {
		Eigen::Vector4f v;
		v.head<3>() = cast(pt3);
		v(3) = 1;
		v = T_*v;
		v/= v(2);
		return v.head<2>();
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
	
	RTREE rtree_;
	Context::Ptr scene_;
	Projector_Viewport projector_;
	nuklei::kernel::se3 tf_;
	
	void insert(Object::Ptr obj);
	void insert(Object::Ptr obj, const Plane_Polygon &p);
	void insert(Object::Ptr obj, const std::vector<Plane_Polygon> &ps);
	void _insert(Object::Ptr obj, const std::vector<Plane_Polygon> &ps);
	
public:
	Context2D(const Context::Ptr &ctxt, const Eigen::Matrix3f &proj, const nuklei::kernel::se3 &tf, const double far_clipping=4.);
	
	void merge(const Context::Ptr &ctxt, const bool merge_enabled=true);
	
	void save_as_svg(const std::string &fn) const;
	
	bool fitModel(const Object &model, double &score_coverage, double &score_matching);
};

class GlobalContext {
	Context::Ptr scene_;
	Context2D::Ptr scene_2d_;
	
	Eigen::Matrix3d proj_;
	
	bool merge_enabled_;
	
	typedef std::vector<Classifier::Ptr> ClassifierSet;
	
	ClassifierSet classifiers_;
	
	void classify(const Context::Ptr &ctxt, const bool single_shot);
public:
	typedef boost::shared_ptr<GlobalContext> Ptr;

	GlobalContext();
	
	void add_scene(const cob_3d_mapping_msgs::PlaneScene &, TransformationEstimator * const tf_est, const sensor_msgs::ImageConstPtr& color_img=sensor_msgs::ImageConstPtr(), const sensor_msgs::ImageConstPtr& depth_img=sensor_msgs::ImageConstPtr());
	
	void visualize_markers();
	
	bool registerClassifier(Classifier *c);
	
	bool &merge_enabled() {return merge_enabled_;}
	
	inline void set_projection(const Eigen::Matrix3d &proj) {
		proj_ = proj;
	}
};

}
