#pragma once

#include "object.h"
#include <boost/lexical_cast.hpp>

namespace cob_3d_geometry_map {
	
class Context;
class CmpSmallestAxis;

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

	virtual bool once() const {return false;}
	virtual int class_id() const = 0;
	virtual std::string name() const = 0;
	virtual Class::Ptr classifiy(Object::Ptr obj, ContextPtr ctxt, const bool single_shot) = 0;
	
	virtual void start(ContextPtr ctxt) {}
	virtual void end(ContextPtr ctxt) {}
	virtual void visualize(ContextPtr ctxt, std::vector<boost::shared_ptr<Visualization::Object> > &objs) {}
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

	class Plane;

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
			virtual void visualize(ContextPtr ctxt, std::vector<boost::shared_ptr<Visualization::Object> > &objs);
			
			const std::vector<double> &get_rays() const {return rays_;}
			inline float max_range() const {return 5.f;}
		};
	}
	
	
	namespace CustomClassifier {
		enum {CLASSIFIER_CARTON_SIDE=-100, CLASSIFIER_CARTON_FRONT=-101};
		
		class Classifier_Carton : public Classifier {
			struct Classification {
				Plane* plane_;
				double width_;
				
				Classification(Plane *p, double w=-1) : plane_(p), width_(w) {}
			};
			
			//parameters
			const int id_;
			ObjectVolume interest_volume_;
			std::vector<double> widths_;
			Classifier_Carton *classifier_front_;
			
			std::vector<Classification> classified_planes_;
			
			Class::Ptr classifiy_front(Plane *plane, ContextPtr ctxt, const bool single_shot);
			Class::Ptr classifiy_side(Plane *plane, ContextPtr ctxt, const bool single_shot);
	
			ObjectVolume generate_shelf() const {
				ObjectVolume shelf = interest_volume_;
				Eigen::Vector3f vi=interest_volume_.bb_in_pose().min(), va=interest_volume_.bb_in_pose().max();
				va(2) = vi(2)+0.04; //4cm
				shelf._bb() = ObjectVolume::TBB(vi, va);
				return shelf;
			}
	
			ObjectVolume generate_floor_shelf() const {
				ObjectVolume shelf = interest_volume_;
				Eigen::Vector3f vi=interest_volume_.bb_in_pose().min(), va=interest_volume_.bb_in_pose().max();
				vi(1) = va(1)-0.03; //3cm
				shelf._bb() = ObjectVolume::TBB(vi, va);
				return shelf;
			}
			
			ObjectVolume generate_over_shelf() const {
				ObjectVolume over_shelf = interest_volume_;
				over_shelf._bb().min()(0) -= 0.15; //20cm
				over_shelf._bb().min()(1) -= 0.1; //cm
				over_shelf._bb().min()(2) -= 0.1; //cm
				over_shelf._bb().max()(0) += 0.15; //20cm
				over_shelf._bb().max()(1) += 0.15; //cm
				over_shelf._bb().max()(2) += 0.15; //cm
				return over_shelf;
			}
			
			bool find_extrema(const Projector &projector, const CmpSmallestAxis &cmp, Vector2 &ex1, Vector2 &ex2) const;
			
			Eigen::Vector3f meanNormal() const;
			void extend(ObjectVolume &vol) const;
			std::vector<ObjectVolume> get_cartons(const ObjectVolume &volume) const;
			
		public:
			Classifier_Carton(const int id, const Eigen::Affine3f &pose, const Eigen::Vector3f &sizes, const std::vector<double> &widths, const double bandwith_orientation=0.2) :
				id_(CLASSIFIER_CARTON_FRONT - id*1000), interest_volume_(ContextPtr()), widths_(widths), classifier_front_(NULL)
			{
				interest_volume_.pose() = cast(pose);
				interest_volume_.pose().ori_h_ = bandwith_orientation;
				interest_volume_.pose().loc_h_ = sizes.maxCoeff();
				
				interest_volume_._bb().min().fill(0);
				interest_volume_._bb().max() = sizes;
				
				std::sort(widths_.begin(), widths_.end());
				assert(widths_.front()<=widths_.back());
				
				//opposite direction
				for(size_t i=0; i<widths.size(); i++)
					widths_.push_back(-widths_[i]);
			}
			
			Classifier_Carton(Classifier_Carton *cl_front) : 
				id_(cl_front->id_+1), interest_volume_(cl_front->interest_volume_), widths_(cl_front->widths_), classifier_front_(cl_front)
			{
			}
			
			virtual void start(ContextPtr ctxt) {
				std::cout<<name()<<": "<<classified_planes_.size()<<std::endl;
				classified_planes_.clear();
			}
			
			virtual void end(ContextPtr ctxt) {
			}

			virtual int class_id() const {return id_;}
			inline int carton_id() const {return (CLASSIFIER_CARTON_SIDE-id_)/1000;}
			virtual std::string name() const {return (classifier_front_?"carton_side":"carton_front")+boost::lexical_cast<std::string>( carton_id() );}
			
			virtual Class::Ptr classifiy(Object::Ptr obj, ContextPtr ctxt, const bool single_shot);
			virtual void visualize(ContextPtr ctxt, std::vector<boost::shared_ptr<Visualization::Object> > &objs);
			
			std::vector<ObjectVolume> get_cartons() const;
		};
		
	}

}
