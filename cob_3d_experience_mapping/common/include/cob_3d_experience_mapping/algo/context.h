#pragma once

#include "../param.h"
#include <set>
#include <map>
#include <boost/thread.hpp>

namespace cob_3d_experience_mapping {

	template<class TStatePtr>
	bool energy_order(const TStatePtr &a, const TStatePtr &b) {
		return a->dist_o() < b->dist_o();
	}
	
	template<class TState, class TEnergy>
	struct Result {
		typename TState::TPtr state_;
		TEnergy delta_energy_;
		
		Result(const typename TState::TPtr &s, const TEnergy &e):
			state_(s), delta_energy_(e)
		{}
	};
	
	template<class _TEnergy, class _TState, class _TFeature, class _TEnergyFactor, class _TTransform>
	class Context {
	public:
		typedef _TEnergy TEnergy;
		typedef _TState TState;
		typedef _TFeature TFeature;
		typedef _TEnergyFactor TEnergyFactor;
		typedef _TTransform TTransform;
		//typedef std::set<typename TState::TPtr> TActList; 
		typedef std::vector<typename TState::TPtr> TActList; 
		typedef typename TActList::iterator TActListIterator; 
		typedef Parameter<TEnergyFactor, typename _TTransform::TDist> TParameter;
		typedef std::map<typename TFeature::TID, typename TFeature::TPtr> FeatureMap;
		
	private:
		TActList active_cells_;
		TParameter param_;
		TEnergy energy_sum_, energy_max_, last_dist_min_;
		typename TState::TPtr last_active_cell_, virtual_cell_;
		typename TTransform::TPtr virtual_transistion_;
		FeatureMap features_;
		boost::mutex mtx_;
		
		//helper functions
		void update_overall_energy(const TEnergy &delta_energy) {
			energy_sum_ += delta_energy;
		}
		
		void update_max_energy() {
			if(active_cells_.size()>0)
				energy_max_ = (*active_cells_.begin())->energy();
			else
				energy_max_ = 0;
		}
		
	public:
		Context() : energy_sum_(0), energy_max_(0), last_dist_min_(0) {
		}
		
		void add_to_active(typename TState::TPtr &cell) {
			for(size_t i=0; i<active_cells_.size(); i++)
				if(active_cells_[i]==cell) return;
			cell->dist_o() = param().energy_max_;
			//if(active_cells_.size()>0) cell->dist_o() += active_cells_.back()->dist_o();
			cell->dist_h_in()  = 0;
			cell->dist_h_out() = 0;
			cell->dbg().hops_ = 0;
			cell->reset_feature();
			//active_cells_.insert(active_cells_.begin()+(active_cells_.size()-1), cell);
			active_cells_.push_back(cell);
			DBG_PRINTF("DBG: added");
		}
		
		void remove_cell(typename TState::TPtr &cell) {
			DBG_PRINTF("DBG: remove_cell");
			if(!cell) return;
			
			cell->still_exists() = false;
			for(size_t i=0; i<active_cells_.size(); i++)
				if(active_cells_[i]==cell)
					active_cells_.erase(active_cells_.begin()+i);
		}
		
		void clean_active_list() {
			std::sort(active_cells_.begin(), active_cells_.end(), energy_order<typename TState::TPtr>);
			if(active_cells_.size()>param().max_active_cells_) {
				DBG_PRINTF("DBG: removing\n");
				active_cells_.erase(active_cells_.begin()+param().max_active_cells_, active_cells_.end());
			}
			if(active_cells_.size()>0 && active_cells_.back()->dist_o()>param().energy_max_) {
				DBG_PRINTF("DBG: rescaling %f\n", param().energy_max_/active_cells_.back()->dist_o());
				for(size_t i=0; i<active_cells_.size(); i++) {
					
					if(active_cells_[i]->dist_o()>=param().energy_max_) {
						active_cells_[i]->dbg().hops_ = 0;
						active_cells_[i]->dist_h_in()  = 1;
						active_cells_[i]->dist_h_out() = 0;
						active_cells_[i]->reset_feature();
					}
						
					active_cells_[i]->dist_o() *= param().energy_max_/active_cells_.back()->dist_o();
				}
			}
		}
		
		//getter/setter
		inline TActList &active_cells() {return active_cells_;}
		inline const TParameter &param() const {return param_;}
		inline typename TState::TPtr current_active_cell() {return *active_cells_.begin();} //TODO: check
		inline typename TState::TPtr &virtual_cell() {return virtual_cell_;}
		inline typename TTransform::TPtr &virtual_transistion() {return virtual_transistion_;}
		inline typename TState::TPtr &last_active_cell() {return last_active_cell_;}
		
		//inline const TEnergy &energy_sum() const {return energy_sum_;}
		//inline const TEnergy &energy_max() const {return energy_max_;}
		inline const TEnergy &last_dist_min() const {return last_dist_min_;}
		inline TEnergy &last_dist_min() {return last_dist_min_;}

		//inline void set_energy_max(const TEnergy &e) {energy_max_=e;}
		
		boost::mutex &get_mutex() {return mtx_;}
		
		void add_feature(const typename TFeature::TID &id, const int ts) {
			boost::lock_guard<boost::mutex> guard(mtx_);
			
			//if( current_active_cell() && virtual_cell() && current_active_cell()->id() < virtual_cell()->id()-(param().min_age_+1) )
			//	return;
			
			typename FeatureMap::iterator it = features_.find(id);
			if(it==features_.end())
				it = features_.insert(typename FeatureMap::value_type(id, typename TFeature::TPtr(new TFeature(id))) ).first;
			it->second->visited(current_active_cell().get(), current_active_cell());
			it->second->inject(this, ts, param().est_occ_);
		}
#if 0		
		template<class TIter>
		void apply_energy_change(const TIter &begin, const TIter &end)
		{
			/*DBG_PRINTF("apply_energy_change");

			//remember all cells with (changed) energy
			typedef std::map<typename TState::TPtr, typename TState::TPtr> TMem;
			TMem mem;
			
			for(typename TActList::iterator it=active_cells_.begin(); it!=active_cells_.end(); it++)
				mem[*it] = *it;
			
			for(TIter it=begin; it!=end; it++) {
				DBG_PRINTF("energy old: %f", it->state_->energy());

				it->delta_energy_ = std::max(it->delta_energy_, -it->state_->energy());	// at least 0
				it->state_->energy() += it->delta_energy_;
				update_overall_energy(it->delta_energy_);
				it->delta_energy_ = 0;
				
				mem[it->state_] = it->state_;

				DBG_PRINTF("energy new: %f", it->state_->energy());
			}
			
			active_cells_.clear();
			for(typename TMem::iterator it=mem.begin(); it!=mem.end(); it++)
				if(it->first->energy()>0)
					active_cells_.insert(it->first);
					
			update_max_energy();

			DBG_PRINTF("new max energy: %f", energy_max());*/
		}
#endif

		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
		    ROS_ASSERT(version==0); //TODO: version handling
		    
		    param_.serialize(ar, version);
		}
	};
	
	
	template<class TContext, class TGraph, class TMapCells, class TMapTransformations>
	class ContextContainer {
		TContext &ctxt_;
		TGraph &graph_;
		TMapCells &cells_;
		TMapTransformations &trans_;
		
	public:
		
		ContextContainer(TContext &ctxt, TGraph &graph, TMapCells &cells, TMapTransformations &trans) :
		 ctxt_(ctxt), graph_(graph), cells_(cells), trans_(trans)
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
				ctxt_.active_cells().clear();
				
				ar & BOOST_SERIALIZATION_NVP(num);
				for(size_t i=0; i<num; i++) {
					typename TContext::TState::TPtr c(new typename TContext::TState);
					c->set_node(graph_.addNode());
					cells_.set(c->node(), c);
					c->serialize_single(ar, version);
					
					ctxt_.active_cells().push_back(c);
				}
			}
			else { //saving...
				for(typename TGraph::NodeIt it(graph_); it!=lemon::INVALID; ++it)
					++num;
					
				ar & BOOST_SERIALIZATION_NVP(num);
				for(typename TGraph::NodeIt it(graph_); it!=lemon::INVALID; ++it)
					cells_[it]->serialize_single(ar, version);
			}
			
			for(typename TGraph::NodeIt it(graph_); it!=lemon::INVALID; ++it)
				cells_[it]->template serialize_trans<int>(ar, version, graph_, cells_, trans_);
		}
			
	};
}
