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
};

class Object3D : public Object {
protected:
	nuklei::kernel::se3 pose_;
	
public:
	typedef Object Parent;
	
	Object3D(const ContextPtr &ctxt) : Object(ctxt)
	{}

	//<getter for pose
	const nuklei::kernel::se3 &pose() const {return pose_;}
	
	virtual std::string type() {return "Object3D";}
	
	virtual void visualization(std::vector<boost::shared_ptr<Visualization::Object> > &);
};

class ObjectVolume : public Object3D {
protected:
	typedef Eigen::AlignedBox<float,3> TBB;
	
	//bounding box
	TBB bb_;
public:
	typedef Object3D Parent;
	
	ObjectVolume(const ContextPtr &ctxt) : Object3D(ctxt)
	{}

	//<getter for pose
	const TBB &bb() const {return bb_;}
	
	virtual std::string type() {return "ObjectVolume";}
	
	virtual void visualization(std::vector<boost::shared_ptr<Visualization::Object> > &);
};

}
