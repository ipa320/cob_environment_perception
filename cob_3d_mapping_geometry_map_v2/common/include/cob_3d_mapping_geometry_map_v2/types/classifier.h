#pragma once

#include "object.h"


namespace cob_3d_geometry_map {
	
class Context;

class Class {
public:
	typedef boost::shared_ptr<Class> Ptr;
	
	virtual int class_id() const = 0;
	virtual std::string name() const = 0;
};
	
class Classifier {
protected:
	typedef boost::shared_ptr<Context> ContextPtr;
	
public:
	typedef boost::shared_ptr<Classifier> Ptr;

	virtual int class_id() const = 0;
	virtual std::string name() const = 0;
	virtual Class::Ptr classifiy(Object::Ptr obj, ContextPtr ctxt) = 0;
};

class Class_Simple : public Class {
protected:
	Classifier *classifier_;
	
public:
	typedef boost::shared_ptr<Class_Simple> Ptr;
	
	Class_Simple(Classifier *c) :
		classifier_(c)
	{}
	
	virtual int class_id() const {return classifier_->class_id();}
	virtual std::string name() const {return classifier_->name();}
};

	namespace DefaultClassifier {
		enum {CLASSIFIER_FLOOR=1};
		
		class Classifier_Floor : public Classifier {
			nuklei::kernel::r3xs2 floor_params_;
			
		public:
			Classifier_Floor(const Eigen::Vector3f &normal, const Eigen::Vector3f &offset, const float bandwith_off, const float bandwith_normal) {
				assert(bandwith_off>=0);
				assert(bandwith_normal>=0 && bandwith_normal<=M_PI);
				
				floor_params_.loc_ = cast(offset);
				floor_params_.dir_ = cast(normal.normalized());
				floor_params_.loc_h_ = bandwith_off;
				floor_params_.dir_h_ = bandwith_normal;
			}

			virtual int class_id() const {return CLASSIFIER_FLOOR;}
			virtual std::string name() const {return "floor";}
			
			virtual Class::Ptr classifiy(Object::Ptr obj, ContextPtr ctxt);
		};
		
	}

}
