#pragma once

#include "../param.h"
#include <set>
#include <map>

namespace cob_3d_experience_mapping {
	
	template<class TState, class TEnergy>
	struct Result {
		typename TState::TPtr state_;
		TEnergy delta_energy_;
		
		Result(const typename TState::TPtr &s, const TEnergy &e):
			state_(s), delta_energy_(e)
		{}
	};
	
	template<class _TEnergy, class _TState, class _TEnergyFactor, class _TTransform>
	class Context {
	public:
		typedef _TEnergy TEnergy;
		typedef _TState TState;
		typedef _TEnergyFactor TEnergyFactor;
		typedef _TTransform TTransform;
		typedef std::set<typename TState::TPtr> TActList; 
		typedef typename TActList::iterator TActListIterator; 
		typedef Parameter<TEnergyFactor, typename _TTransform::TDist> TParameter;
		
	private:
		TActList active_cells_;
		TParameter param_;
		TEnergy energy_sum_, energy_max_, last_energy_max_;
		typename TState::TPtr last_active_cell_, virtual_cell_;
		typename TTransform::TPtr virtual_transistion_;
		
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
		Context() : energy_sum_(0), energy_max_(0), last_energy_max_(0) {
		}
		
		//getter/setter
		inline TActList &active_cells() {return active_cells_;}
		inline const TParameter &param() {return param_;}
		inline typename TState::TPtr current_active_cell() {return *active_cells_.begin();} //TODO: check
		inline typename TState::TPtr &virtual_cell() {return virtual_cell_;}
		inline typename TTransform::TPtr &virtual_transistion() {return virtual_transistion_;}
		inline typename TState::TPtr &last_active_cell() {return last_active_cell_;}
		
		inline const TEnergy &energy_sum() const {return energy_sum_;}
		inline const TEnergy &energy_max() const {return energy_max_;}
		inline const TEnergy &last_energy_max() const {return last_energy_max_;}
		inline TEnergy &last_energy_max() {return last_energy_max_;}

		template<class TIter>
		void apply_energy_change(const TIter &begin, const TIter &end)
		{
			ROS_INFO("apply_energy_change");

			//remember all cells with (changed) energy
			typedef std::map<typename TState::TPtr, typename TState::TPtr> TMem;
			TMem mem;
			
			for(typename TActList::iterator it=active_cells_.begin(); it!=active_cells_.end(); it++)
				mem[*it] = *it;
			
			for(TIter it=begin; it!=end; it++) {
				ROS_INFO("energy old: %f", it->state_->energy());

				it->delta_energy_ = std::max(it->delta_energy_, -it->state_->energy());	// at least 0
				it->state_->energy() += it->delta_energy_;
				update_overall_energy(it->delta_energy_);
				it->delta_energy_ = 0;
				
				mem[it->state_] = it->state_;

				ROS_INFO("energy new: %f", it->state_->energy());
			}
			
			active_cells_.clear();
			for(typename TMem::iterator it=mem.begin(); it!=mem.end(); it++)
				if(it->first->energy()>0)
					active_cells_.insert(it->first);
					
			update_max_energy();

			ROS_INFO("new max energy: %f", energy_max());
		}
		
	};
	
}
