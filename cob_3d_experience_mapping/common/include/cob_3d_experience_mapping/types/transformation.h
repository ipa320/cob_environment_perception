#pragma once

#include <Eigen/Core>
#include <boost/math/distributions/normal.hpp>

namespace cob_3d_experience_mapping {
	
	template<typename _TType, int NUM_TRANS, int NUM_ROT, class TStatePtr>
	class Transformation {
	public:
		typedef _TType TType;
		typedef Eigen::Matrix<TType, NUM_TRANS+NUM_ROT, 1> TLink;
		typedef Eigen::Matrix<TType, 2, 1> TDist;
		typedef boost::shared_ptr<Transformation> TPtr;
		
	private:
		TLink link_;
		TStatePtr src_;

		template<int TInd>
		inline static void helper_hystersis(TDist &d, const TDist &thr) {
			if(d(TInd)<thr(TInd)) d(TInd) = 1-d(TInd)/thr(TInd);
			else d(TInd) = 0;
		}
		
	public:
		
		Transformation() : link_(TLink::Zero())
		{}
		
		Transformation(const TStatePtr &state) :
		link_(TLink::Zero()), src_(state)
		{}

		Transformation(const TLink &link, const TStatePtr &state) :
		link_(link), src_(state)
		{}
		
		inline const TStatePtr &src() const {return src_;}
		inline TStatePtr &src() {return src_;}

		Transformation directed(const TStatePtr &state) const {
			if(state==src_) return *this;
			return Transformation(-1*link_, state);
		}
		
		Transformation scale(const TDist &thr) const {
			TLink tmp = link_;
			TDist d = distance(Transformation());
			for(int i=0; d(0) && i<NUM_TRANS; i++)
				tmp(i) *= thr(0)/d(0);
			for(int i=NUM_TRANS; d(1) && i<NUM_TRANS+NUM_ROT; i++)
				tmp(i) *= thr(1)/d(1);
			return Transformation(tmp, src_);
		}

		TDist distance(const Transformation &o) const {
			TLink tmp = link_-o.link_;
			TDist r = TDist::Zero();
			
			for(int i=0; i<NUM_TRANS; i++)
				r(0) += std::pow(tmp(i), 2);
			for(int i=NUM_TRANS; i<NUM_TRANS+NUM_ROT; i++)
				r(1) += std::pow(tmp(i), 2);
				
			r(0) = std::sqrt(r(0));
			r(1) = std::sqrt(r(1));
			return r;
		}

		TDist energy_distance(const Transformation &tmp, const TDist &thr) const {
			TDist r = TDist::Zero();
			TDist l = TDist::Zero();
			
			for(int i=0; i<NUM_TRANS; i++) {
				r(0) += std::pow(tmp(i), 2);
				l(0) += std::pow(link_(i), 2);
			}
			for(int i=NUM_TRANS; i<NUM_TRANS+NUM_ROT; i++) {
				r(1) += std::pow(tmp(i), 2);
				l(1) += std::pow(link_(i), 2);
			}
				
			r(0) = std::sqrt(r(0))/std::max(std::sqrt(l(0), thr(0));
			r(1) = std::sqrt(r(1))/std::max(std::sqrt(l(1), thr(1));
			
			return r.norm();
		}
		
		void integrate(const Transformation &movement) {
			BOOST_STATIC_ASSERT(NUM_TRANS==2);	//TODO: at the moment only implement for 2d case
			BOOST_STATIC_ASSERT(NUM_ROT==1);

			link_(2) += movement.link_(2);
			//clip to 180 deg.
			while(link_(2)<=-M_PI) link_(2) += 2*M_PI;
			while(link_(2)>  M_PI) link_(2) -= 2*M_PI;
			link_(0) += movement.link_(0) * std::cos(link_(2)) + movement.link_(1) * std::sin(link_(2));
			link_(1) += movement.link_(0) * std::sin(link_(2)) - movement.link_(1) * std::cos(link_(2));
		}

		//TODO:
		TType proximity_neg(const TDist &thr, const TType &k) const {
			return -1+factor(energy_distance(link_, thr));
		}

		//TODO:
		TType proximity_pos(const Transformation &o, const TDist &thr, const TType &k) const {
			return factor(energy_distance(o-link_, thr));
		}

		TDist proximity(const boost::math::normal distribution[2]) const {
			return proximity(distribution, Transformation());
		}
		
		TDist proximity(const boost::math::normal distribution[2], const Transformation &o) const {
			TDist r = distance(o);
			r(0) = boost::math::cdf(distribution[0], r(0));
			r(1) = boost::math::cdf(distribution[1], r(1));
			return r;
		}
		
	};
}
