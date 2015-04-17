#pragma once

#include <lemon/list_graph.h>
#include <boost/shared_ptr.hpp>

#include <ros/assert.h>

namespace cob_3d_experience_mapping {
	
	struct Empty {};
	
	template<class TMeta>
	class Object {
	protected:
		TMeta meta_;
	};
	
	struct DbgInfo {
		std::string name_;
		std::string info_;
	};
	
	/**
	 * class: State
	 * short description: current state + activation --> Artificial Neuron
	 */
	template<class TMeta, class _TEnergy>
	class State : public Object<TMeta> {
	public:
		//types
		typedef _TEnergy TEnergy;
		typedef lemon::ListDigraph TGraph;
		typedef TGraph::Arc TArc;
		typedef TGraph::Node TNode;
		typedef TGraph::OutArcIt TArcIterator;
		typedef boost::shared_ptr<State> TPtr;
	protected:
		TEnergy energy_, loss_, outflow_, outflow_em_;
		TNode node_;
		DbgInfo dbg_;
		
	public:		
		State(): energy_(0), loss_(0), outflow_(0), outflow_em_(0) {
			static int no = 1;
			char buf[128];
			sprintf(buf, "%d", no++);
			dbg_.name_ = buf;
		}
		
		//setter/getter
		inline TEnergy &energy() {return energy_;}
		inline const TEnergy &energy() const {return energy_;}
		
		inline TEnergy &loss() {return loss_;}
		inline const TEnergy &loss() const {return loss_;}
		
		inline TEnergy &outflow() {return outflow_;}
		inline const TEnergy &outflow() const {return outflow_;}
		
		inline TEnergy &outflow_em() {return outflow_em_;}
		inline const TEnergy &outflow_em() const {return outflow_em_;}
		
		inline void set_node(const TNode &node) {node_ = node;}
		inline TNode &node() {return node_;}
	
		inline DbgInfo &dbg() {return dbg_;}
		inline const DbgInfo &dbg() const {return dbg_;}
		
		//graph operations
		inline TArcIterator edge_begin(const TGraph &graph) {
			return TArcIterator(graph, node_);
		}
		
		inline TArcIterator edge_end(const TGraph &graph) const {
			return lemon::INVALID;
		}
		
		inline TNode opposite_node(const TGraph &graph, const TArc &ait) {
			return graph.oppositeNode(node_, ait);
		}
		
		//operators
		inline bool operator>(const State &o) {
			return energy()>o.energy();
		}
	};
	
	/**
	 * class: Transition
	 * short description: describes transformation of 2 "State"s, costs (e.g. distance), time of create and last update
	 */
	template<class TMeta, class TPtrState, class TTransformation, class TTime>
	class Transition : public Object<TMeta> {
	protected:
		TPtrState src_, dst_;
		TTransformation trans_;
		TTime ts_creation_, ts_update_;
	};
	
	/**
	 * class: Injection
	 * short description: weighted link between sensor input layer and state layer
	 */
	template<class TMeta>
	class Injection : public Object<TMeta> {
	protected:
	};
	
	/**
	 * class: Feature
	 * short description: sensor input
	 */
	template<class TMeta>
	class Feature : public Object<TMeta> {
	protected:
	};
}
