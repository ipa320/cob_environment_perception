#pragma once

#include "../param.h"
#include <set>
#include <map>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>


//! interfaces and implementations of cob_3d_experience_mapping
namespace cob_3d_experience_mapping {

	//! functions to sort active state list
	namespace sorting {
		
		//!< comparision of two states (by distance 1. deviation distance 2. travel distance)
		template<class TStatePtr>
		bool energy_order(const TStatePtr &a, const TStatePtr &b) {
			if(a->dist_dev() == b->dist_dev())
				return a->dist_trv() < b->dist_trv();
			return a->dist_dev() < b->dist_dev();
		}
		
		//!< function to check if list is sorted (fallback for older C++ versions)
		template <class ForwardIterator, class Compare>
		bool is_sorted (ForwardIterator first, ForwardIterator last, Compare comp)
		{
			if (first==last) return true;
			ForwardIterator next = first;
			while (++next!=last) {
			if (comp(*next,*first))     // or, if (comp(*next,*first)) for version (2)
			  return false;
			++first;
			}
			return true;
		}
		
	}
	
	/*! \class Context
		\brief Representation of (active) map data

		Templated class which contains all active map data:
		 - active state list
		 - virtual state
		 - feature lookup to state
		 - some variables for algorithms
	*/
	template<class _TEnergy, class _TState, class _TFeature, class _TEnergyFactor, class _TTransform>
	class Context {
	public:
		typedef _TEnergy TEnergy;
		typedef _TState TState;
		typedef _TFeature TFeature;
		typedef _TEnergyFactor TEnergyFactor;
		typedef _TTransform TTransform;
		typedef std::vector<typename TState::TPtr> TActList; 
		typedef typename TActList::iterator TActListIterator; 
		typedef Parameter<TEnergyFactor, typename _TTransform::TDist> TParameter;
		typedef std::map<typename TFeature::TID, typename TFeature::TPtr> FeatureMap;
		typedef boost::circular_buffer<typename TFeature::TID> FeatureBuffer;
		
	private:
		TActList active_states_;		//!< active state list
		TParameter param_;			//!< parameter storage
		TEnergy last_dist_min_;		//!< 
		typename TState::TPtr last_active_state_, virtual_state_;
		typename TTransform::TPtr virtual_transistion_;
		FeatureMap features_;
		FeatureBuffer last_features_;
		boost::mutex mtx_;
		bool needs_sort_;
		
	public:
		Context() : last_dist_min_(0), last_features_(10), needs_sort_(true) {
		}
		
		//!< check if sorting is needed (because feature was seen) and resets flag
		bool needs_sort() {
			bool tmp = needs_sort_;
			needs_sort_ = false;
			return tmp;
		}
		
		//!< add a state to the active state list + init. variables + (init. distances if needed)
		void add_to_active(typename TState::TPtr &state, const bool already_set=false) {
			if(!virtual_state() && !already_set)
				return;
			
			//if already present in active list -> skip
			for(size_t i=0; i<active_states_.size(); i++)
				if(active_states_[i]==state) {
					if(!already_set) {
						state->dist_dev() 	= std::min(state->dist_dev(), virtual_state()->dist_dev()+1);
						needs_sort_ = true;
					}
					return;
				}
				
			//somebody else will set this variables from outside
			if(!already_set) {
				state->dist_dev() 	= virtual_state()->dist_dev()+1;
				state->dist_trv()  	= 0.5;	//we are approaching state (assume half way)
				state->hops() 		= 0;
			}
			
			//reset feature proability
			state->is_active() = true;
			state->reset_feature();
			
			//add to list
			active_states_.push_back(state);
			
			needs_sort_ = true;
			
			DBG_PRINTF("DBG: added");
		}		
		
		//!< remove state completely
		void remove_state(typename TState::TPtr &state) {
			if(!state) return;
			
			state->still_exists() = false;
			state->is_active() = false;
			for(size_t i=0; i<active_states_.size(); i++)
				if(active_states_[i]==state)
					active_states_.erase(active_states_.begin()+i);
		}
		
		//!< remove unnecessary states from active state list and keep maximum size of list within limit
		void clean_active_list() {
			assert( sorting::is_sorted(active_states_.begin(),active_states_.end(), sorting::energy_order<typename TState::TPtr>) );
			
			if(active_states_.size()>param().max_active_states_) {
				active_states_.erase(active_states_.begin()+param().max_active_states_, active_states_.end());
				
				DBG_PRINTF("DBG: removing\n");
			}
			
			if(active_states_.size()>0) {
				for(size_t i=1; i<active_states_.size(); i++) {
					
					if(active_states_[i]->dist_trv()<=-1) {
						active_states_[i]->is_active() = false;
						active_states_.erase(active_states_.begin()+i);
						--i;
					}
					
				}
			}
			
		}
		
		//getter/setter
		inline TActList &active_states() {return active_states_;}
		inline const TParameter &param() const {return param_;}
		inline TParameter &param_rw() {return param_;}
		inline typename TState::TPtr current_active_state() {return *active_states_.begin();} //TODO: check
		inline typename TState::TPtr &virtual_state() {return virtual_state_;}
		inline typename TTransform::TPtr &virtual_transistion() {return virtual_transistion_;}
		inline typename TState::TPtr &last_active_state() {return last_active_state_;}
		
		//inline const TEnergy &energy_sum() const {return energy_sum_;}
		//inline const TEnergy &energy_max() const {return energy_max_;}
		inline const TEnergy &last_dist_min() const {return last_dist_min_;}
		inline TEnergy &last_dist_min() {return last_dist_min_;}

		//inline void set_energy_max(const TEnergy &e) {energy_max_=e;}
		
		boost::mutex &get_mutex() {return mtx_;}
		
		void add_feature(const typename TFeature::TID &id, const int ts) {
			boost::lock_guard<boost::mutex> guard(mtx_);
			
			//if( current_active_state() && virtual_state() && current_active_state()->id() < virtual_state()->id()-(param().min_age_+3) )
			//	return;
			
			for(typename FeatureBuffer::iterator it = last_features_.begin(); it!=last_features_.end(); it++)
				if(*it == id)
					return;
			last_features_.push_back(id);
			
			typename FeatureMap::iterator it = features_.find(id);
			if(it==features_.end())
				it = features_.insert(typename FeatureMap::value_type(id, typename TFeature::TPtr(new TFeature(id))) ).first;
			if( !(current_active_state() && virtual_state() && current_active_state()->id() < virtual_state()->id()-(param().min_age_+3) ) )
				it->second->visited(current_active_state().get(), current_active_state());
			it->second->inject(this, ts, param().est_occ_, param().max_active_states_);
		}

		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
		    ROS_ASSERT(version==0); //TODO: version handling
		    
		    param_.serialize(ar, version);
		}
	};
	
	
	template<class TContext, class TGraph, class TMapStates, class TMapTransformations>
	class ContextContainer {
		TContext &ctxt_;
		TGraph &graph_;
		TMapStates &states_;
		TMapTransformations &trans_;
		
	public:
		
		ContextContainer(TContext &ctxt, TGraph &graph, TMapStates &states, TMapTransformations &trans) :
		 ctxt_(ctxt), graph_(graph), states_(states), trans_(trans)
		 {}
		    	
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
		    ROS_ASSERT(version==0); //TODO: version handling
		    
		    ctxt_.serialize(ar, version);
		    
			size_t num=0;
		    if(Archive::is_loading::value) {
				//clear everything
				graph_.clear();
				ctxt_.active_states().clear();
				
				ar & BOOST_SERIALIZATION_NVP(num);
				for(size_t i=0; i<num; i++) {
					typename TContext::TState::TPtr c(new typename TContext::TState);
					c->set_node(graph_.addNode());
					states_.set(c->node(), c);
					c->serialize_single(ar, version);
					
					ctxt_.active_states().push_back(c);
				}
			}
			else { //saving...
				for(typename TGraph::NodeIt it(graph_); it!=lemon::INVALID; ++it)
					++num;
					
				ar & BOOST_SERIALIZATION_NVP(num);
				for(typename TGraph::NodeIt it(graph_); it!=lemon::INVALID; ++it)
					states_[it]->serialize_single(ar, version);
			}
			
			for(typename TGraph::NodeIt it(graph_); it!=lemon::INVALID; ++it)
				states_[it]->template serialize_trans<int>(ar, version, graph_, states_, trans_);
		}
			
	};
}
