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
	virtual Class::Ptr classifiy(Object::Ptr obj, ContextPtr ctxt, const bool single_shot) = 0;
	
	virtual void start(ContextPtr ctxt) {}
	virtual void end(ContextPtr ctxt) {}
	virtual void visualize(ContextPtr ctxt) {}
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
		enum {CLASSIFIER_FLOOR=1, CLASSIFIER_WALL=2};
		
		class Classifier_Floor : public Classifier, public Projector {
			nuklei::kernel::r3xs2 floor_params_;
			std::vector<double> rays_;
			
			virtual Vector2 operator()(const nuklei_wmf::Vector3<double> &pt3) const;
			
		public:
			Classifier_Floor(const Eigen::Vector3f &normal, const Eigen::Vector3f &offset, const float bandwith_off, const float bandwith_normal, const size_t num_rays) {
				assert(bandwith_off>=0);
				assert(bandwith_normal>=0 && bandwith_normal<=M_PI);
				
				rays_.resize(num_rays, std::numeric_limits<double>::infinity());
				
				floor_params_.loc_ = cast(offset);
				floor_params_.dir_ = cast(normal.normalized());
				floor_params_.loc_h_ = bandwith_off;
				floor_params_.dir_h_ = bandwith_normal;
			}
			
			virtual void start(ContextPtr ctxt) {
				rays_.assign(rays_.size(), std::numeric_limits<double>::infinity());
			}
			
			virtual void end(ContextPtr ctxt) {
				//sqrt only once per ray :)
				for(size_t j=0; j<rays_.size(); j++)
					rays_[j] = std::sqrt(rays_[j]);
			}

			virtual int class_id() const {return CLASSIFIER_FLOOR;}
			virtual std::string name() const {return "floor";}
			
			virtual Class::Ptr classifiy(Object::Ptr obj, ContextPtr ctxt, const bool single_shot);
			
			const std::vector<double> &get_rays() const {return rays_;}
			inline float max_range() const {return 5.f;}
		};
	}
	
	
	namespace CustomClassifier {
		enum {CLASSIFIER_CARTON=-100};
		
		class Classifier_Carton : public Classifier {
			ObjectVolume interest_volume_;
			
		public:
			Classifier_Carton() :
				interest_volume_(ContextPtr())
			{
			}

			virtual int class_id() const {return CLASSIFIER_CARTON;}
			virtual std::string name() const {return "carton";}
			
			virtual Class::Ptr classifiy(Object::Ptr obj, ContextPtr ctxt, const bool single_shot);
		};
		
	}

}
