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
		
	public:
		
		Transformation() : link_(TLink::Zero())
		{}
		
		Transformation(const TLink &link, const TStatePtr &state) :
		link_(link), src_(state)
		{}
		
		Transformation directed(const TStatePtr &state) {
			if(state==src_) return *this;
			return Transformation(-1*link_, state);
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
