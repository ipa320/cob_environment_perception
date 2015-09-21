#pragma once

#include "../defs.h"
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
		
		FeatureMap &get_features() {return features_;}
		
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
				state->dist_trv()  	= 0;	//we are approaching state (assume half way)
				state->hops() 		= 0;
			}
			
			//reset feature proability
			state->is_active() = true;
			state->reset_feature();
			
			//add to list
			active_states_.push_back(state);
			
			needs_sort_ = true;
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
			
			//limit size of list
			if(active_states_.size()>param().max_active_states_) {
				for(size_t i=param().max_active_states_; i<active_states_.size(); i++)
					active_states_[i]->is_active() = false;
					
				active_states_.erase(active_states_.begin()+param().max_active_states_, active_states_.end());
			}
			
			//remove all departed states
			if(active_states_.size()>0) {
				//skip current active state
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
		//!< settter/getter for active state list
		inline TActList &active_states() {return active_states_;}
		//!< getter for parameters
		inline const TParameter &param() const {return param_;}
		//!< settter/getter for parameters
		inline TParameter &param_rw() {return param_;}
		inline typename TState::TPtr current_active_state() {return *active_states_.begin();} //TODO: check
		inline typename TState::TPtr &virtual_state() {return virtual_state_;}
		inline typename TTransform::TPtr &virtual_transistion() {return virtual_transistion_;}
		inline typename TState::TPtr &last_active_state() {return last_active_state_;}
		
		inline const TEnergy &last_dist_min() const {return last_dist_min_;}
		inline TEnergy &last_dist_min() {return last_dist_min_;}
		
		//!< settter/getter for mutex
		boost::mutex &get_mutex() {return mtx_;}
		
		//!< if a feature was seen we connect the active state with the feature and update all connected states
		void add_feature(const typename TFeature::TID &id, const int ts) {
			boost::lock_guard<boost::mutex> guard(mtx_);
			
			//if( current_active_state() && virtual_state() && current_active_state()->id() < virtual_state()->id()-(param().min_age_+3) )
			//	return;
			
			for(typename FeatureBuffer::iterator it = last_features_.begin(); it!=last_features_.end(); it++)
				if(*it == id) {
					DBG_PRINTF("feature %d already set\n", id);
					return;
				}
			last_features_.push_back(id);
			
			typename FeatureMap::iterator it = features_.find(id);
			if(it==features_.end())
				it = features_.insert(typename FeatureMap::value_type(id, typename TFeature::TPtr(new TFeature(id))) ).first;
			if( !(current_active_state() && virtual_state() && current_active_state()->id() < virtual_state()->id()-(param().min_age_+3) ) )
				it->second->visited(current_active_state().get(), current_active_state());
			it->second->inject(this, ts, param().est_occ_, param().max_active_states_);
		}

		UNIVERSAL_SERIALIZE()
		{
		    assert(version==CURRENT_SERIALIZATION_VERSION);;
		    
		    ar & UNIVERSAL_SERIALIZATION_NVP(param_);
		}
	};
	
	
	template<class TContext, class TGraph, class TMapStates, class TMapTransformations>
	class ContextContainer {
		TContext *ctxt_;
		TGraph *graph_;
		TMapStates *states_;
		TMapTransformations *trans_;
		bool incremental_;
		
	public:			
		ContextContainer() :
		 ctxt_(NULL), graph_(NULL), states_(NULL), trans_(NULL), incremental_(false)
		 {}
		 
		ContextContainer(TContext *ctxt, TGraph *graph, TMapStates *states, TMapTransformations *trans, const bool incremental=false) :
		 ctxt_(ctxt), graph_(graph), states_(states), trans_(trans), incremental_(incremental)
		 {}
		 
		bool is_incremental() const {return incremental_;}
		void set_incremental(const bool inc) {incremental_=inc;}
		    	
		UNIVERSAL_SERIALIZE()
		{
		    assert(version==CURRENT_SERIALIZATION_VERSION);
		    
		    //we have to lock everything for consistency
			boost::mutex mtx_tmp;
			boost::lock_guard<boost::mutex> guard(ctxt_ ? ctxt_->get_mutex() : mtx_tmp);
		    
		    if(!incremental_) {
				if(ctxt_)
				 ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("ctxt", *ctxt_);
				else {
					TContext tmp;
					ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("ctxt", tmp);
				}
			}
		    
			std::vector<typename TContext::TState> states;	//we should use references to the state to spare memory/speed up
			std::vector<typename TContext::TState::TransitionSerialization> trans;
		    std::vector<typename TContext::TFeature::FeatureSerialization> fts;
			
			//on saving
		    if(graph_ && states_ && trans_ && UNIVERSAL_CHECK<Archive>::is_saving(ar)) {
				
				for(typename TGraph::NodeIt it(*graph_); it!=lemon::INVALID; ++it) {
					states.push_back(*(*states_)[it]);
					(*states_)[it]->get_trans(trans, *graph_, *states_, *trans_);
				}

				for(typename TContext::FeatureMap::iterator it = ctxt_->get_features().begin(); it!=ctxt_->get_features().end(); it++) {
					fts.push_back( it->second->get_serialization() );
				}
			}
		    
		    ar & UNIVERSAL_SERIALIZATION_NVP(fts);
			ar & UNIVERSAL_SERIALIZATION_NVP(states);
			ar & UNIVERSAL_SERIALIZATION_NVP(trans);
			
			//on loading
		    if(graph_ && states_ && trans_ && UNIVERSAL_CHECK<Archive>::is_loading(ar)) {
				if(!incremental_) {
					//clear everything
					graph_->clear();
					ctxt_->active_states().clear();
				}
				
				//insert states
				for(size_t i=0; i<states.size(); i++) {
					if(incremental_) {	//look if state already exists
						bool found = false;
						for(typename TGraph::NodeIt it(*graph_); it!=lemon::INVALID; ++it) {
							if( (*states_)[it]->id() == states[i].id() ) {
								*(*states_)[it] = states[i];
								found = true;
								break;
							}
						}
							
						if(found) continue;
					}
					
					typename TContext::TState::TPtr c(new typename TContext::TState(states[i]));
					
					c->set_node(graph_->addNode());
					states_->set(c->node(), c);
					//ctxt_.active_states().push_back(states[i]);
				}
				
				//insert transitions (TODO: speed up)
				for(typename TGraph::NodeIt it(*graph_); it!=lemon::INVALID; ++it)
					(*states_)[it]->set_trans(trans, *graph_, *states_, *trans_);
					
				//insert features
				for(size_t i=0; i<fts.size(); i++) {
					typename TContext::TFeature *tmp = new typename TContext::TFeature(fts[i].ft_.id());
					tmp->set_serialization(fts[i], *graph_, *states_);
					
					ctxt_->get_features().insert(typename TContext::FeatureMap::value_type(fts[i].ft_.id(), typename TContext::TFeature::TPtr(tmp)) );
				}
				
			}
		}
			
	};
}
