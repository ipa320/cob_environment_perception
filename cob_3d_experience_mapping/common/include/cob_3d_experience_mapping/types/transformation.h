#pragma once

#include <Eigen/Core>
#include <boost/math/distributions/normal.hpp>

namespace cob_3d_experience_mapping {
	
	template<class TTransformation>
	class ActionSearchResult {
		typename TTransformation::TPtr trans_;
		bool found_;
	public:
	
		ActionSearchResult(typename TTransformation::TPtr trans): found_(true), trans_(trans)
		{}
		
		ActionSearchResult(const bool found): found_(found)
		{}
		
		static ActionSearchResult None() {return ActionSearchResult(false);}
		static ActionSearchResult Reached() {return ActionSearchResult(true);}
		
		bool found() {return found_;}
		bool reached() {return found_ && !trans_;}
	};
	
	template<typename _TType, int NUM_TRANS, int NUM_ROT>
	class TransformationLink {
	public:
		typedef _TType TType;
		typedef Eigen::Matrix<TType, NUM_TRANS+NUM_ROT, 1> TLink;
		typedef Eigen::Matrix<TType, 2, 1> TDist;
		typedef boost::shared_ptr<TransformationLink> TPtr;
		
	protected:
		TLink link_;
		TLink deviation_;

		template<int TInd>
		inline static void helper_hystersis(TDist &d, const TDist &thr) {
			if(d(TInd)<thr(TInd)) d(TInd) = 1-d(TInd)/thr(TInd);
			else d(TInd) = 0;
		}
		
	public:
		
		TransformationLink() : link_(TLink::Zero()), deviation_(TLink::Zero())
		{}
		
		
		inline const TLink &get_data() const {return link_;}
		
		inline TLink &deviation() {return deviation_;}
		inline TLink deviation() const {return deviation_;}

		TransformationLink scale(const TDist &thr) const {
			TLink tmp = link_;
			TDist d = distance(TransformationLink());
			for(int i=0; d(0) && i<NUM_TRANS; i++)
				tmp(i) *= thr(0)/d(0);
			for(int i=NUM_TRANS; d(1) && i<NUM_TRANS+NUM_ROT; i++)
				tmp(i) *= thr(1)/d(1);
			return TransformationLink(tmp);
		}

		TDist distance(const TransformationLink &o) const {
			TLink tmp = link_-o.link_;
			TDist r;
			
			r(0) = tmp.template head<NUM_TRANS>().norm();
			r(1) = tmp.template tail<NUM_ROT>  ().norm();
			
			return r;
		}
		
		void integrate(const TransformationLink &movement) {
			link_ += movement.link_;
			//_DEVIATION += movement._DEVIATION;
		}
		
		inline Eigen::Matrix<TType, NUM_TRANS, 1> translation() const {return link_.template head<NUM_TRANS>();}
		inline Eigen::Matrix<TType, NUM_ROT, 1> rotation() const {return link_.template tail<NUM_ROT>();}
		
		inline TType dist() const {
			return link_.norm();
		}
		
		TType dist(const TDist &thr) const {
			TDist r;
			
			r(0) = link_.template head<NUM_TRANS>().norm()/thr(0);
			r(1) = link_.template tail<NUM_ROT>  ().norm()/thr(1);
			
			return r.norm();
		}
		
		inline TType dist(const TLink &thr) const {
			return link_.cwiseProduct(thr.cwiseInverse()).norm();
		}
		
		/*TType dist_uncertain(const TDist &thr) const {
			return dist(thr) + _DEVIATION;
		}*/
		
		inline static Eigen::Matrix<TType, NUM_ROT, 1> dist_rad(Eigen::Matrix<TType, NUM_ROT, 1> v) {
			for(int i=0; i<NUM_ROT; i++) {
				while(v(i)<0) v(i)+=2*M_PI;
				while(v(i)>2*M_PI) v(i)-=2*M_PI;
				if(v(i)>M_PI) v(i)=2*M_PI-v(i);
			}
			return v;
		}
		
		/*TType transition_factor(const TransformationLink &o, const TDist &thr) const {
			TDist r;
			TLink tmp;
			
			if(link_.squaredNorm())
				tmp = std::sqrt(o.link_.squaredNorm()/link_.squaredNorm())*link_ + o.link_;
			else
				tmp = o.link_;
			
			r(0) = tmp.template head<NUM_TRANS>().norm()/thr(0);
			r(1) = tmp.template tail<NUM_ROT>  ().norm()/thr(1);
			
			return o.dist(thr)-r.norm();
		}*/
		
		void transition_factor(const TransformationLink &o, const TLink &thr, TType &sim, TType &dev, TLink &er) const {
			TType rel;
			transition_factor(o, thr, sim, dev, er, rel);
		}
		void transition_factor(const TransformationLink &o, const TLink &thr, TType &sim, TType &dev, TLink &er, TType &rel) const {
			TLink tmp1 =   link_.cwiseProduct(thr.cwiseInverse());
			TLink tmp2 = o.link_.cwiseProduct(thr.cwiseInverse());
			
			if(tmp1.squaredNorm()) {
				sim =  tmp1.dot( -tmp2 )/tmp1.squaredNorm();
				dev = std::sqrt( std::max((TType)0, tmp2.squaredNorm() - sim*tmp1.dot( -tmp2 )) );
				
				if(sim>1) sim = 1-sim;
				sim = std::max((TType)0, sim);
				rel = sim*tmp1.norm()/tmp2.norm();
				
				er = (tmp2+sim*tmp1).cwiseAbs().cwiseProduct(thr);
			}
			else {
				rel = dev = sim = 0;
				er.fill(0);
			}
			
			DBG_PRINTF("tr f %f %f %f\t\t\t%f %f  \t\t   %f %f\n", sim, dev, rel, link_(0), link_(2), er(0), er(2));
			DBG_PRINTF("links  %f %f\n", o.link_(0), o.link_(2));
			
			//assert(er.norm() <= 2*o.link_.norm()+0.00001f);
		}
		void transition_factor2(const TransformationLink &o, const TLink &thr, const TType relation_factor, TType &sim, TType &dev, TType &rel) const {
			TLink tmp1 =   link_.cwiseProduct(thr.cwiseInverse());
			TLink tmp2 = o.link_.cwiseProduct(thr.cwiseInverse());
			TLink dev1 = deviation_.cwiseProduct(thr.cwiseInverse());
			
			if(tmp1.squaredNorm()) {
				sim =  tmp1.dot( -tmp2 )/tmp1.squaredNorm();
				dev = std::sqrt( std::max((TType)0, tmp2.squaredNorm() - sim*tmp1.dot( -tmp2 )) );
				
				if(sim>1) sim = 1-sim;
				sim = std::max((TType)0, sim);
				rel = sim*tmp1.norm()/tmp2.norm();
				
				TLink er = (tmp2+sim*tmp1).cwiseAbs();
				
				DBG_PRINTF("error1 %f (allowed %f %f)   \t%f %f\n", dev, (dev1*tmp2.norm()/tmp1.norm())(0), (dev1*tmp2.norm()/tmp1.norm())(2), er(0), er(2));
				
				er-= dev1*tmp2.norm()/tmp1.norm();
				//er-= tmp2.cwiseAbs()*0.1f;
				er = er.cwiseMax(TLink::Zero());
				
				//dev = relation_factor*(std::pow(1/relation_factor, er.norm()/dev1.norm())-1);
				//dev = relation_factor*(std::exp(er.norm()/dev1.norm())-1);
				//dev = std::max((TType)0, er.sum()-relation_factor*tmp2.norm());
				//dev = er.sum();
				dev = er.sum()/tmp1.norm();
				//dev = relation_factor*er.sum()/tmp2.norm();
				
				//allowed dev. for odom:
				const TType allowed = dev1.norm()*tmp2.norm()/tmp1.norm();
				
				DBG_PRINTF("error2 %f (allowed %f)   \t%f %f\n", dev, allowed, er(0), er(2));
				
				//if(dev<allowed) dev = 0;
				//else dev = relation_factor*(dev-allowed); //dev = (relation_factor/deviation())*(dev-allowed);
				
				if(dev!=dev) dev = tmp1.norm();
			}
			else
				rel = dev = sim = 0;
			DBG_PRINTF("dev %f %f\n", dev1(0), dev1(2));
			DBG_PRINTF("link %f %f\n", tmp1(0), tmp1(2));
			DBG_PRINTF("odom %f %f\n", tmp2(0), tmp2(2));
			DBG_PRINTF("link %f %f\n", link_(0), link_(2));
			DBG_PRINTF("odom %f %f\n", o.link_(0), o.link_(2));
			DBG_PRINTF("tr f2 %f %f %f\t\t\t%f %f %f (f: %f)\n", sim, dev, rel, link_(0), link_(2), deviation().norm(), (relation_factor/dev1.norm()));
		}
		
		void dbg() const {
			DBG_PRINTF("action: ");
			for(int i=0; i<NUM_TRANS+NUM_ROT; i++)
				DBG_PRINTF("%f ", link_(i));
			DBG_PRINTF("\n");
		}
		
		typename Eigen::Transform<TType,3,Eigen::Affine> affine() const {
			BOOST_STATIC_ASSERT(NUM_TRANS<=3);
			BOOST_STATIC_ASSERT(NUM_ROT==1);
			
			Eigen::Matrix<TType, 3, 1> tr = Eigen::Matrix<TType, 3, 1>::Zero();
			for(int i=0; i<NUM_TRANS; i++)
				tr(i) = link_(i);
			return Eigen::Translation<TType,3>(tr)
					* Eigen::AngleAxis<TType>(link_.template tail<NUM_ROT>()(0), Eigen::Matrix<TType, 3, 1>::UnitZ());
		}
		
		UNIVERSAL_SERIALIZE()
		{
		    assert(version==CURRENT_SERIALIZATION_VERSION);
		    
		    for(int i=0; i<NUM_TRANS+NUM_ROT; i++) {
				char buf[16];
				sprintf(buf, "link_%d", i);
		    	ar & UNIVERSAL_SERIALIZATION_NVP_NAMED(buf, link_(i));
			}
		    
		    for(int i=0; i<NUM_TRANS+NUM_ROT; i++) {
				char buf[16];
				sprintf(buf, "dev_%d", i);
		    	ar & UNIVERSAL_SERIALIZATION_NVP_NAMED(buf, deviation_(i));
			}
		}
	};
	
	
	template<class _TransformationLink, class TStatePtr>
	class Transformation : public _TransformationLink {
	public:
		typedef boost::shared_ptr<Transformation> TPtr;
		typedef typename _TransformationLink::TLink TLink;
		typedef typename _TransformationLink::TDist TDist;
		typedef typename _TransformationLink::TType TType;
		
	protected:
		TStatePtr src_;
		
		using typename _TransformationLink::link_;
		
	public:
		
		Transformation()
		{}
		
		Transformation(const TStatePtr &state) :
			src_(state)
		{}

		Transformation(const TLink &link, const TLink &dev, const TStatePtr &state) :
			src_(state)
		{
			this->link_ = link;
			this->deviation_ = dev;
		}

		Transformation(const _TransformationLink &o, const TStatePtr &state) :
			src_(state)
		{
			this->link_ = o.get_data();
			this->deviation_ = o.deviation();
		}
		
		inline const TStatePtr &src() const {return src_;}
		inline TStatePtr &src() {return src_;}

		Transformation directed(const TStatePtr &state) const {
			if(state==src_) return *this;
			return Transformation(-1*this->link_, this->deviation_, state);
		}
	};
}
