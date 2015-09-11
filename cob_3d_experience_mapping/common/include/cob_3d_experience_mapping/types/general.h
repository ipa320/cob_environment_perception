#pragma once

#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/adaptors.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

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
		Eigen::Vector3f pose_;
		int hops_;
		
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
		    ROS_ASSERT(version==0); //TODO: version handling
		    
		   ar & BOOST_SERIALIZATION_NVP(name_);
		   ar & BOOST_SERIALIZATION_NVP(info_);
		}
	};
	
	/**
	 * class: State
	 * short description: current state + activation --> Artificial Neuron
	 */
	template<class TMeta, class _TEnergy, class _TGraph, class _TLink >
	class State : public Object<TMeta> {
	public:
		//types
		typedef _TEnergy TEnergy;
		typedef _TGraph TGraph;
		typedef typename TGraph::Node TNode;
		//typedef typename TGraph::Arc TArc;
		//typedef typename TGraph::Edge TArc;
		typedef typename TGraph::Arc TArc;
		//typedef typename TGraph::OutArcIt TArcIterator;
		//typedef typename TGraph::EdgeIt TArcIterator;
		typedef typename TGraph::OutArcIt TArcOutIterator;
		typedef typename TGraph::InArcIt TArcInIterator;
		typedef boost::shared_ptr<State> TPtr;
		typedef int ID;
	protected:
		TEnergy do_, dh_in_, dh_out_, ft_imp_, ft_imp_last_;
		TNode node_;
		ID id_;
		DbgInfo dbg_;
		bool still_exists_;
		
	public:		
		State(): do_(0), dh_in_(0), dh_out_(0), ft_imp_(1), ft_imp_last_(1), still_exists_(true) {
			static int no = 1;
			char buf[128];
			id_ = no;
			sprintf(buf, "%d", no++);
			dbg_.name_ = buf;
			
			dbg_.hops_ = 0;
		}
		
		//setter/getter		
		//inline ID &id() {return id_;}
		inline ID  id() const {return id_;}
		
		inline bool &still_exists() {return still_exists_;}
		inline bool  still_exists() const {return still_exists_;}
		
		inline TEnergy &dist_o() {return do_;}
		inline const TEnergy &dist_o() const {return do_;}
		
		inline TEnergy &dist_h_in() {return dh_in_;}
		inline TEnergy &dist_h_out() {return dh_out_;}
		inline const TEnergy &dist_h() const {return std::max(dh_in_, dh_out_);}
		
		inline TEnergy d() const {return std::sqrt(d2());}
		
		inline TEnergy d2() const {return do_*do_ + dist_h()*dist_h();}
		
		inline void set_node(const TNode &node) {node_ = node;}
		inline TNode &node() {return node_;}
	
		inline DbgInfo &dbg() {return dbg_;}
		inline const DbgInfo &dbg() const {return dbg_;}
		
		//graph operations
		inline TArcInIterator arc_in_begin(const TGraph &graph) {
			return TArcInIterator(graph, node_);
		}
		inline TArcInIterator arc_in_end(const TGraph &graph) const {
			return lemon::INVALID;
		}
		
		inline TArcOutIterator arc_out_begin(const TGraph &graph) {
			return TArcOutIterator(graph, node_);
		}
		inline TArcOutIterator arc_out_end(const TGraph &graph) const {
			return lemon::INVALID;
		}
		
		//e.g. for visualization, where it's not necessary only one direction
		template<typename _TArc_>
		inline _TArc_ arc_flex_begin(const TGraph &graph) {
			return _TArc_(graph, node_);
		}
		template<typename _TArc_>
		inline _TArc_ arc_flex_end(const TGraph &graph) {
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
					 
			ft_imp_ -= ft_imp_*prob/(no_conn+est_occ);
			
			 //ft_imp_ *= 1-prob/(no_conn+est_occ);
			 //DBG_PRINTF("upd %d  %f\n", id(), get_feature_prob());
		}
		
		inline TEnergy get_feature_prob() const {return 1-ft_imp_;}
		inline TEnergy get_last_feature_prob() const {return 1-ft_imp_last_;}
		
		void reset_feature() {ft_imp_last_=ft_imp_; ft_imp_=1;}
		
		template<class Archive>
		void serialize_single(Archive & ar, const unsigned int version)
		{
		   ROS_ASSERT(version==0); //TODO: version handling
		   
		   ar & BOOST_SERIALIZATION_NVP(id_);
		   
		   dbg_.serialize(ar, version);
		}
		
		template<class ID, class Archive, class Graph, class TMapCells, class TMapTransformations>
		void serialize_trans(Archive & ar, const unsigned int version, Graph &graph, TMapCells &cells, TMapTransformations &trans)
		{
			ROS_ASSERT(version==0); //TODO: version handling
		   
			size_t num=0;
		   
			if(Archive::is_saving::value) {
				for(TArcOutIterator ait(arc_out_begin(graph)); ait!=arc_out_end(graph); ++ait)
					++num;
					//ids.push_back( cells[opposite_node(graph, ait)]->dbg().id_ );
			}
			ar & BOOST_SERIALIZATION_NVP(num);
		   
			if(Archive::is_loading::value) {
				TPtr th;
				for(typename TGraph::NodeIt it(graph); it!=lemon::INVALID; ++it)
					if(cells[it]->id_==id_) {
						th = cells[it];
						break;
					}
				ROS_ASSERT(th);
				
				for(size_t i=0; i<num; i++) {
					char buf[16];
					sprintf(buf, "id%d", (int)i);
					
					ID id;
					ar & boost::serialization::make_nvp(buf, id);
					
					//find cell
					typename TGraph::NodeIt it(graph);
					for(; it!=lemon::INVALID; ++it) {
						if(cells[it]->id_==id) {
							typename TMapTransformations::Value link(new typename TMapTransformations::Value::element_type());
							link->serialize(ar, version);
							link->src() = th;
							trans.set(graph.addArc(th->node(), cells[it]->node()), link);
							break;
						}
					}
					ROS_ASSERT(it!=lemon::INVALID);
				}
				
			}
			else {	//saving...
				int i=0;
				for(TArcOutIterator ait(arc_out_begin(graph)); ait!=arc_out_end(graph); ++ait) {
					char buf[16];
					sprintf(buf, "id%d", i);
					++i;
					ar & boost::serialization::make_nvp(buf, cells[opposite_node(graph, ait)]->id_);
					trans[ait]->serialize(ar, version);
				}
			}
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
			if(injections_.find(h)==injections_.end()) {
				injections_.insert(typename InjectionMap::value_type(h, inj));
				
				//debug
				char buf[32];
				sprintf(buf,"ft%d ", id_);
				inj->dbg().info_ += buf;
				
				DBG_PRINTF("add feature %d -> %d\n", id_, inj->id());
			}
		}
		
		template<typename TContext>
		void inject(TContext *ctxt, const int ts, const int est_occ, const int max_occ, const typename TInjection::TEnergy prob=1) {
			for(typename InjectionMap::iterator it=injections_.begin(); it!=injections_.end(); it++) {
				if(!it->second->still_exists()) continue;
				
				it->second->update(ts, std::min(max_occ, (int)injections_.size()), est_occ, prob);
				
				DBG_PRINTF("injectXYZ %d -> %d with %d\n", id_, it->second->id(), (int)(injections_.size()+est_occ));
			
				//check if feature is in active list --> add
				ctxt->add_to_active(it->second);
			}
			DBG_PRINTF("inject %d\n", (int)injections_.size());
		}
		
	};
}
