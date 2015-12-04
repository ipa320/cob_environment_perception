#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <map>
#include <nuklei/Kernel.h>
#include <nuklei/LinearAlgebra.h>

class asIScriptEngine;


namespace cob_3d_geometry_map {
	
class Context;
class Class;
namespace Visualization {class Object;}


template<class T>
inline nuklei_wmf::Vector3<double> cast(const Eigen::Matrix<T,3,1> &v) {
	return nuklei_wmf::Vector3<double>(v(0), v(1), v(2));
}

inline Eigen::Vector3f cast(const nuklei_wmf::Vector3<double> &v) {
	return Eigen::Vector3f(v.X(), v.Y(), v.Z());
}

template<class T>
inline nuklei_wmf::Quaternion<double> cast(const Eigen::Quaternion<T> &v) {
	return nuklei_wmf::Quaternion<double>(v.w(), v.x(), v.y(), v.z());
}

inline Eigen::Quaternionf cast(const nuklei_wmf::Quaternion<double> &v) {
	return Eigen::Quaternionf(v.W(), v.X(), v.Y(), v.Z());
}

template<class T>
inline nuklei::kernel::se3 cast(const Eigen::Transform<T,3,Eigen::Affine> &v) {
	nuklei::kernel::se3 k;
	k.loc_ = cast((Eigen::Matrix<T,3,1>)v.translation());
	k.ori_ = cast((Eigen::Quaternion<T>)v.rotation());
	return k;
}

inline Eigen::Affine3f cast(const nuklei::kernel::se3 &v) {
	return Eigen::Translation3f(cast(v.loc_))*cast(v.ori_);
}

typedef Eigen::Matrix<float, 2,1> Vector2;
	
class Projector {
public:
	virtual Vector2 operator()(const nuklei_wmf::Vector3<double> &pt3) const = 0;
};

	
class Object {
public:
	typedef int TID;
	typedef boost::shared_ptr<Object> Ptr;
	typedef boost::shared_ptr<Context> ContextPtr;
	typedef std::map<int, boost::shared_ptr<Class> > ClassList;
	
	static TID generate_id();
	
private:
	TID id_;
	ClassList classes_;
	
protected:
	ContextPtr ctxt_;
	
public:

	Object(const ContextPtr &ctxt): id_(generate_id()), ctxt_(ctxt)
	{
	}
	
	inline TID id() const {return id_;}
	
	virtual std::string type() {return "Object";}
	
	virtual void visualization(std::vector<boost::shared_ptr<Visualization::Object> > &) {}
	
	std::string vis_name(const std::string &prefix) const;
	
	bool has_class(const int id) const;
	void add_class(const boost::shared_ptr<Class> &cl);
	void rem_class(const int id);
	
	virtual bool can_merge_fast(const Object &o) const {return false;}
	virtual bool can_merge(const Object &o) const {return false;}
	virtual bool merge(const Object &o, const double relation) {return true;}
	virtual bool in_front_of(const Object &o) const {return false;}
};

class Object3D : public Object {
protected:
	nuklei::kernel::se3 pose_;
	
public:
	typedef Object Parent;
	typedef boost::shared_ptr<Object3D> Ptr;
	
	Object3D(const ContextPtr &ctxt) : Object(ctxt)
	{}

	//<getter for pose
	const nuklei::kernel::se3 &pose() const {return pose_;}
	nuklei::kernel::se3 &pose() {return pose_;}
	
	virtual std::string type() {return "Object3D";}
	
	virtual void visualization(std::vector<boost::shared_ptr<Visualization::Object> > &);
};

class ObjectVolume : public Object3D {
public:
	typedef Eigen::AlignedBox<float,3> TBB;
	
protected:
	
	//bounding box in pose frame
	TBB bb_;
	
	bool _overlaps(const ObjectVolume &o) const;
public:
	typedef Object3D Parent;
	typedef boost::shared_ptr<ObjectVolume> Ptr;
	
	ObjectVolume(const ContextPtr &ctxt) : Object3D(ctxt)
	{}

	TBB &_bb() {return bb_;}
	
	//<getter for pose
	const TBB &bb_in_pose() const {return bb_;}
	TBB bb_in_context() const {
		TBB bb;
		bb.setEmpty();		
		for(int i=0; i<8; i++)
			bb.extend(cast(pose_)*bb_.corner( (TBB::CornerType)i ));
		return bb;
	}
	
	virtual std::string type() {return "ObjectVolume";}
	
	virtual void visualization(std::vector<boost::shared_ptr<Visualization::Object> > &);
	
	
	bool overlaps(const ObjectVolume &o) const;
	bool contains(const ObjectVolume &o) const;
	
	virtual bool in_front_of(const Object &o) const {
		const ObjectVolume *vol = dynamic_cast<const ObjectVolume*>(&o);
		if(vol) return in_front_of(*vol);
		
		return false;
	}
	
	virtual bool in_front_of(const ObjectVolume &o) const {
		return (cast(pose_).inverse()*cast(o.pose_)*o.bb_.center())(2)>=-(pose_.loc_h_+o.pose_.loc_h_);
	}
	
	inline Eigen::Vector3f local2global(const Eigen::Vector3f &pt3_local) const {
		return cast(pose_)*pt3_local;
	}
};

class Projector_Volume : public Projector {
public:
	enum ESide {FRONT, BACK, LEFT, RIGHT, TOP, BOTTOM};
	
private:
	const ObjectVolume* vol_;
	ESide side_;
	
	nuklei::kernel::se3 _pose() const {
		nuklei::kernel::se3 T;
		
		switch(side_) {
			case FRONT:
				T.ori_ = nuklei_wmf::Quaternion<double>::IDENTITY;
				T.loc_ = cast(vol_->bb_in_pose().min());
			break;
			case BACK:
				T.ori_ = nuklei_wmf::Quaternion<double>::IDENTITY;
				T.loc_ = cast(vol_->bb_in_pose().max());
			break;
			
			case LEFT:
				T.ori_.FromAxisAngle(nuklei_wmf::Vector3<double>::UNIT_Y, M_PI_2);
				T.loc_ = cast(vol_->bb_in_pose().min());
			break;
			case RIGHT:
				T.ori_.FromAxisAngle(nuklei_wmf::Vector3<double>::UNIT_Y, M_PI_2);
				T.loc_ = cast(vol_->bb_in_pose().max());
			break;
			
			case BOTTOM:
				T.ori_.FromAxisAngle(nuklei_wmf::Vector3<double>::UNIT_X, -M_PI_2);
				T.loc_ = cast(vol_->bb_in_pose().min());
			break;
			case TOP:
				T.ori_.FromAxisAngle(nuklei_wmf::Vector3<double>::UNIT_X, -M_PI_2);
				T.loc_ = cast(vol_->bb_in_pose().max());
			break;
		}
		
		T.ori_ = vol_->pose().ori_*T.ori_;
		T.loc_ = nuklei::la::transform(vol_->pose().loc_, vol_->pose().ori_, T.loc_);
		
		return T;
	}
	
public:
	Projector_Volume(const ObjectVolume* v, const ESide side) : vol_(v), side_(side) {
	}
	
	virtual Vector2 operator()(const nuklei_wmf::Vector3<double> &pt3) const {
		const nuklei::kernel::se3 pose_inv = _pose().inverseTransformation();
		nuklei_wmf::Vector3<double> t = nuklei::la::transform(pose_inv.loc_, pose_inv.ori_, pt3);
		return Vector2(t.X(), t.Y());
	}
	
	virtual nuklei_wmf::Vector3<double> operator()(const Vector2 &pt2) const {
		nuklei_wmf::Vector3<double> pt3(pt2(0), pt2(1), 0);
		const nuklei::kernel::se3 pose = _pose();
		return nuklei::la::transform(pose.loc_, pose.ori_, pt3);
	}
};

}
