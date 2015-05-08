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
	template<class TMeta, class _TEnergy, class _TGraph>
	class State : public Object<TMeta> {
	public:
		//types
		typedef _TEnergy TEnergy;
		typedef _TGraph TGraph;
		typedef typename TGraph::Node TNode;
		//typedef typename TGraph::Arc TArc;
		typedef typename TGraph::Edge TArc;
		//typedef typename TGraph::OutArcIt TArcIterator;
		typedef typename TGraph::EdgeIt TArcIterator;
		typedef boost::shared_ptr<State> TPtr;
	protected:
		TEnergy do_, dh_, ft_imp_;
		TNode node_;
		DbgInfo dbg_;
		
	public:		
		State(): do_(0), dh_(0), ft_imp_(1) {
			static int no = 1;
			char buf[128];
			sprintf(buf, "%d", no++);
			dbg_.name_ = buf;
		}
		
		//setter/getter
		inline TEnergy &dist_o() {return do_;}
		inline const TEnergy &dist_o() const {return do_;}
		
		inline TEnergy &dist_h() {return dh_;}
		inline const TEnergy &dist_h() const {return dh_;}
		
		inline TEnergy d() const {return std::sqrt(d2());}
		
		inline TEnergy d2() const {return do_*do_ + dh_*dh_;}
		
		inline void set_node(const TNode &node) {node_ = node;}
		inline TNode &node() {return node_;}
	
		inline DbgInfo &dbg() {return dbg_;}
		inline const DbgInfo &dbg() const {return dbg_;}
		
		//graph operations
		inline TArcIterator edge_begin(const TGraph &graph) {
			return TArcIterator(graph);
			//return TArcIterator(graph, node_);
		}
		
		inline TArcIterator edge_end(const TGraph &graph) const {
			return lemon::INVALID;
		}
		
		inline TNode opposite_node(const TGraph &graph, const TArc &ait) {
			return graph.oppositeNode(node_, ait);
		}
		
		//operators
		inline bool operator>(const State &o) {
			return d2()<o.d2();
		}
		
		void update(const int ts, const int no_conn, const int est_occ, const TEnergy prob=1) {			 
			 ft_imp_ *= 1-prob/(no_conn+est_occ);
		}
		
		inline TEnergy get_feature_prob() const {return 1-ft_imp_;}
		
		void reset_feature() {ft_imp_=1;}
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
	
#if 0
	template<class _TCellHandle, class _TType, class TMeta>
	class VisualCell : public Object<TMeta> {
	public:
		typedef _TCellHandle TCellHandle;
		typedef _TType TType;
		typedef boost::shared_ptr<VisualCell> TPtr;
		
	protected:
		TCellHandle h_;
		int last_ts_;
		TType improbability_;
		
	public:
		VisualCell(TCellHandle h) : h_(h), last_ts_(-1), improbability_(1)
		{}
		
		inline const TCellHandle &getHandle() const {return h_;}
		
		void update(const int ts, const int no_conn, const int est_occ, const TType prob=1) {
			if(ts!=last_ts_) {
				last_ts_ = ts;
				improbability_ = 1;
			 }
			 
			 improbability_ *= 1-prob/(no_conn+est_occ);
		}
	};
#endif

	/**
	 * class: Feature
	 * short description: sensor input
	 */
	template<class TInjection, class TMeta>
	class Feature : public Object<TMeta> {
	public:
		typedef int TID;
		typedef boost::shared_ptr<Feature> TPtr;
		typedef void* CellHandle;
		typedef std::map<CellHandle, typename TInjection::TPtr> InjectionMap;
		
	protected:
		TID id_;
		InjectionMap injections_;
		
	public:
		Feature(const TID id) : id_(id)
		{}
		
		void visited(const CellHandle &h, typename TInjection::TPtr inj) {
			if(injections_.find(h)==injections_.end())
				injections_.insert(typename InjectionMap::value_type(h, inj));
		}
		
		void inject(const int ts, const int est_occ, const typename TInjection::TEnergy prob=1) {
			for(typename InjectionMap::iterator it=injections_.begin(); it!=injections_.end(); it++) {
				it->second->update(ts, (int)injections_.size(), est_occ, prob);
			}
			printf("inject %d\n", (int)injections_.size());
		}
		
	};
}
