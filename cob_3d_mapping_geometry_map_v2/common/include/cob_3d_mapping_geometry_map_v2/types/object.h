#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

class asIScriptEngine;


namespace cob_3d_geometry_map {
	
class Context;
namespace Visualization {class Object;}
	
class Object {
public:
	typedef int TID;
	typedef boost::shared_ptr<Object> Ptr;
	typedef boost::shared_ptr<Context> ContextPtr;
	
	static TID generate_id();
	
private:
	TID id_;
	
protected:
	ContextPtr ctxt_;
	
public:

	Object(const ContextPtr &ctxt): id_(generate_id()), ctxt_(ctxt)
	{
	}
	
	inline TID id() const {return id_;}
	
	virtual std::string type() {return "Object";}
	
	virtual void visualization(std::vector<boost::shared_ptr<Visualization::Object> > &) {}
};

class Object3D : public Object {
protected:
	Eigen::Affine3f pose_;
	
public:
	Object3D(const ContextPtr &ctxt) : Object(ctxt), pose_(Eigen::Translation3f(0,0,0))
	{}

	//<getter for pose
	const Eigen::Affine3f &pose() const {return pose_;}
	
	virtual std::string type() {return "Object3D";}
	
	virtual void visualization(std::vector<boost::shared_ptr<Visualization::Object> > &);
};

class ObjectVolume : public Object3D {
protected:
	typedef Eigen::AlignedBox<float,3> TBB;
	
	//bounding box
	TBB bb_;
public:
	ObjectVolume(const ContextPtr &ctxt) : Object3D(ctxt)
	{}

	//<getter for pose
	const TBB &pose() const {return bb_;}
	
	virtual std::string type() {return "ObjectVolume";}
};

}
