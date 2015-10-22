#pragma once

#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/adaptors.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

#include <ros/assert.h>


//! interfaces and implementations of cob_3d_experience_mapping
namespace cob_3d_experience_mapping {
	
	/**
	 * class: Empty
	 * parent class which is used by default if no meta data is needed for states (see Object)
	 */
	struct Empty {};
	
	template<class _TState, class _TFeature>
	class SimpleIdTsGenerator {
	public:
		typedef _TState TState;
		typedef _TFeature TFeature;
		typedef typename TState::ID ID;
		
	private:
		ID running_id_;
		/*
		 * std::map<ID, TModification>
		 */
		
	public:
		SimpleIdTsGenerator() : running_id_(1)
		{}
		
		ID new_id() {return running_id_++;}
		
		void register_modification(const typename TState::TPtr &state)
		{ }
		void register_modification(const typename TFeature::TPtr &state)
		{ }
		void register_removal(const typename TState::TPtr &state)
		{ }
	};
	
	/**
	 * class: Object
	 * parent class for a state to keep meta data like additonal map information (laser scans ...)
	 */
	template<class TMeta>
	class Object {
	protected:
		TMeta meta_;
	};
	
	struct DbgInfo {
		std::string name_;
		std::string info_;
		Eigen::Vector3f pose_;
		
		UNIVERSAL_SERIALIZE()
		{
		    ROS_ASSERT(version==CURRENT_SERIALIZATION_VERSION);
		    
		   ar & UNIVERSAL_SERIALIZATION_NVP(name_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(info_);
		}
	};
	
	/**
	 * class: State
	 * short description: current state + activation --> Artificial Neuron
	 */
	template<class TMeta, class _TEnergy, class _TGraph, class _TLink, class _TID, class _TFeatureClass>
	class State : public Object<TMeta> {
	public:
		//types
		typedef _TEnergy TEnergy;
		typedef _TGraph TGraph;
		typedef typename TGraph::Node TNode;
		typedef typename TGraph::Arc TArc;
		typedef typename TGraph::OutArcIt TArcOutIterator;
		typedef typename TGraph::InArcIt TArcInIterator;
		typedef boost::shared_ptr<State> TPtr;
		typedef _TID ID;
		typedef _TFeatureClass TFeatureClass;
		typedef _TLink TLink;
		typedef std::map<TFeatureClass, uint32_t /*counter*/> TFeatureClassMap;
		
	protected:
		TEnergy dist_dev_, dist_trv_, ft_imp_;
		TNode node_;
		TFeatureClassMap ft_class_occurences_;
		ID id_;
		int hops_;
		DbgInfo dbg_;		//!< some debug information like name and additional description (info)
		bool still_exists_;	//!< flag: false if removed from map completely
		bool is_active_;	//!< flag: true if present in active state list
		
	public:		
		State(): dist_dev_(0), dist_trv_(0), ft_imp_(1), id_(-1), hops_(0), still_exists_(true), is_active_(false) {
			dbg_.name_ = "INVALID";
		}
		
		State(const ID &id): dist_dev_(0), dist_trv_(0), ft_imp_(1), id_(id), hops_(0), still_exists_(true), is_active_(false) {
			char buf[128];
			sprintf(buf, "%d", id_);
			dbg_.name_ = buf;
		}
		
		
		void update(const State &o) {
			ft_class_occurences_ = o.ft_class_occurences_;
			id_ = o.id_;
			dbg_ = o.dbg_;
		}
		
		//setter/getter		
		
		//!< getter for identifier
		inline ID  id() const {return id_;}
		
		//!< setter for identifier
		inline void set_id(const ID &id) {id_ = id;}

		//!< setter/getter for existance flag
		inline bool &still_exists() {return still_exists_;}
		//!< getter for existance flag
		inline bool  still_exists() const {return still_exists_;}

		//!< setter/getter for flag if in active state list
		inline bool &is_active() {return is_active_;}
		//!< getter for flag if in active state list
		inline bool  is_active() const {return is_active_;}
		
		//!< setter/getter for deviation distance
		inline TEnergy &dist_dev() {return dist_dev_;}
		//!< getter for deviation distance
		inline const TEnergy &dist_dev() const {return dist_dev_;}
		
		//!< setter/getter for travel distance
		inline TEnergy &dist_trv() {return dist_trv_;}
		//!< getter for travel distance
		inline const TEnergy &dist_trv() const {return dist_trv_;}
		
		//!< setter/getter for hop counter
		inline int &hops() {return hops_;}
		//!< getter for hop counter
		inline int  hops() const {return hops_;}
		
		//!< setter for graph node
		inline void set_node(const TNode &node) {node_ = node;}
		
		//!< setter/getter for graph node
		inline TNode &node() {return node_;}
	
		//!< setter/getter for debug information
		inline DbgInfo &dbg() {return dbg_;}
		//!< getter for debug information
		inline const DbgInfo &dbg() const {return dbg_;}
		
		
		//graph operations
		//!< iterator operation for incoming transitions
		inline TArcInIterator arc_in_begin(const TGraph &graph) {
			return TArcInIterator(graph, node_);
		}
		//!< iterator operation for incoming transitions
		inline TArcInIterator arc_in_end(const TGraph &graph) const {
			return lemon::INVALID;
		}
		
		//!< iterator operation for outgoing transitions
		inline TArcOutIterator arc_out_begin(const TGraph &graph) {
			return TArcOutIterator(graph, node_);
		}
		//!< iterator operation for outgoing transitions
		inline TArcOutIterator arc_out_end(const TGraph &graph) const {
			return lemon::INVALID;
		}
		
		//!< iterator operation for transitions; e.g. for visualization, where it's not necessary only one direction
		template<typename _TArc_>
		inline _TArc_ arc_flex_begin(const TGraph &graph) {
			return _TArc_(graph, node_);
		}
		//!< iterator operation for transitions; e.g. for visualization, where it's not necessary only one direction
		template<typename _TArc_>
		inline _TArc_ arc_flex_end(const TGraph &graph) {
			return lemon::INVALID;
		}
		
		//!< getter to retrieve connected node of state by transition "ait" in "graph"
		inline TNode opposite_node(const TGraph &graph, const TArc &ait) {
			return graph.oppositeNode(node_, ait);
		}
		
		uint32_t get_feature_class_counter(const TFeatureClass &ft_cl) const {
			typename TFeatureClassMap::const_iterator it = ft_class_occurences_.find(ft_cl);
			if(it==ft_class_occurences_.end())
				return 0;
			return it->second;
		}
		
		//!< if a connected feature to this state was visited, we update the feature probability
		void update(const int ts, const int no_conn, const int est_occ, const uint32_t counter, const TFeatureClass &ft_cl, const TEnergy prob=1) {
			DBG_PRINTF("update for %d#%d: %d, %d", id(), (int)ft_class_occurences_.size(), ft_cl, counter);
			assert(ft_class_occurences_.find(ft_cl)!=ft_class_occurences_.end());
			DBG_PRINTF(" %d\n", ft_class_occurences_[ft_cl]);
			assert(counter<=ft_class_occurences_[ft_cl]);
					 
			ft_imp_ -= ft_imp_*prob*counter/((no_conn+est_occ)*ft_class_occurences_[ft_cl]);
			
			 //ft_imp_ *= 1-prob/(no_conn+est_occ);
			 //DBG_PRINTF("upd %d  %f\n", id(), get_feature_prob());
		}
		
		void visited_feature_class(const TFeatureClass &ft_cl) {
			typedef typename TFeatureClassMap::iterator I;
			std::pair<I,bool> const& r=ft_class_occurences_.insert(typename TFeatureClassMap::value_type(ft_cl,1));
			if (!r.second)
				r.first->second++;
			DBG_PRINTF("visited_featuer_class for %d: %d\n", id(), r.first->second);
		}
		
		//!< getter for feature probability
		inline TEnergy get_feature_prob() const {return 1-ft_imp_;}
		
		//!< reset feature probability to default (no feature present)
		void reset_feature() {ft_imp_=1;}
		
		UNIVERSAL_SERIALIZE()
		{
		   assert(version==CURRENT_SERIALIZATION_VERSION);
		   
		   ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("id", id_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(ft_class_occurences_);
		   
		   if(UNIVERSAL_CHECK<Archive>::is_loading(ar))
			DBG_PRINTF("loaded %d with %d\n", (int)id_, (int)ft_class_occurences_.size());
		   
		   dbg_.serialize<Archive, make_nvp>(ar, version);
		}
		
		/**
		 * class: TransitionSerialization
		 * helper structure for serialization of all transitions of a state
		 */
		struct TransitionSerialization {
			ID src_, dst_;
			TLink link_;
			
			TransitionSerialization()
			{}
			
			TransitionSerialization(const ID &src, const ID &dst, const TLink &link) : src_(src), dst_(dst), link_(link)
			{}
			
			UNIVERSAL_SERIALIZE()
			{
				assert(version==CURRENT_SERIALIZATION_VERSION);
				
				ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("src", src_);
				ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("dst", dst_);
				ar & UNIVERSAL_SERIALIZATION_NVP(link_);
			}
		};
		
		template<class Graph, class TMapStates, class TMapTransformations>
		void get_trans(std::vector<TransitionSerialization> &trans_list, Graph &graph, TMapStates &states, TMapTransformations &trans)
		{
			if(!still_exists()) return;
			
			for(TArcOutIterator ait(arc_out_begin(graph)); ait!=arc_out_end(graph); ++ait) {
				if(states[opposite_node(graph, ait)]->still_exists())
					trans_list.push_back( TransitionSerialization(id(), states[opposite_node(graph, ait)]->id(), *trans[ait]) );
			}
		}
		
		template<class Graph, class TMapStates, class TMapTransformations>
		void set_trans(const std::vector<TransitionSerialization> &trans_list, Graph &graph, TMapStates &states, TMapTransformations &trans)
		{
			for(size_t i=0; i<trans_list.size(); i++) {
				if(trans_list[i].src_!=id()) continue;
				
				//check fo existing trans.
				bool found = false;
				for(TArcOutIterator ait(arc_out_begin(graph)); ait!=arc_out_end(graph); ++ait) {
					if(trans_list[i].dst_ == states[opposite_node(graph, ait)]->id()) {
						*trans[ait] = typename TMapTransformations::Value::element_type(trans_list[i].link_, trans[ait]->src());
						found = true;
						break;
					}
				}
				if(found) continue;
				
				TPtr p_dst;
				for(typename TGraph::NodeIt it(graph); it!=lemon::INVALID && !p_dst; ++it) {
					if(states[it]->id_==trans_list[i].dst_)
						p_dst = states[it];
				}
				
				assert(p_dst);
				
				DBG_PRINTF("add arc %d %d\n", trans_list[i].src_, trans_list[i].dst_);
				if(p_dst && p_dst->still_exists())
					trans.set(graph.addArc(p_dst->node(), node()), typename TMapTransformations::Value(
						new typename TMapTransformations::Value::element_type(trans_list[i].link_, p_dst)
					));
			}
		}
		
	};
	

	/**
	 * class: Feature
	 * short description: sensor input
	 */
	template<class TInjection, class TMeta, class _TID>
	class Feature : public Object<TMeta> {
	public:
		typedef _TID TID;
		typedef boost::shared_ptr<Feature> TPtr;
		typedef void* StateHandle;
		typedef typename TInjection::TFeatureClass TFeatureClass;
		
		struct Connection {
			typename TInjection::TPtr state_;
			uint32_t counter_;
			
			Connection(typename TInjection::TPtr &s, uint32_t c):
				state_(s), counter_(c)
			{}
			
			void merge(const Connection &o) {
				assert(state_ == o.state_);
				
				counter_ = o.counter_;	//TODO: this could remove already incremented counter between updates
			}
		};
		
		typedef std::map<StateHandle, Connection> InjectionMap;
		
	protected:
		TID id_;
		InjectionMap injections_;
		
	public:
		Feature(const TID id) : id_(id)
		{}
		
		inline TID id() const {return id_;}
		
		void merge(const TPtr &o) {
			assert(id()==o->id());
			
			DBG_PRINTF("merge feature %d\n", id());
			
			for(typename InjectionMap::iterator it=o->injections_.begin(); it!=o->injections_.end(); it++) {
				typename InjectionMap::iterator itt = injections_.find(it->first);
				if(itt==injections_.end())
					injections_.insert(typename InjectionMap::value_type(it->first, it->second));
				else
					itt->second.merge(it->second);
			}
		}
		
		//!< setter for identifier
		inline void set_id(const TID &id) {id_ = id;}
		
		bool visited(const StateHandle &h, typename TInjection::TPtr inj) {
			typename InjectionMap::iterator it = injections_.find(h);
			if(it==injections_.end()) {
				injections_.insert(typename InjectionMap::value_type(h, Connection(inj, 1)));
				
				//debug
				char buf[32];
				sprintf(buf,"ft%d ", id_);
				inj->dbg().info_ += buf;
				
				DBG_PRINTF("add feature %d -> %d\n", id_, inj->id());
				
				return true;
			}
			else {
				it->second.counter_++;
#ifdef DEBUG_
				DBG_PRINTF("check ft (%d) cnt: %d <= %d for %d\n", id_, it->second.counter_, it->second.state_->get_feature_class_counter(0), it->second.state_->id());
				assert(it->second.counter_<=it->second.state_->get_feature_class_counter(0));
#endif
			}
			
			return false;
		}
		
		template<typename TContext>
		void inject(TContext *ctxt, const int ts, const int est_occ, const int max_occ, const TFeatureClass &ft_cl, const bool update, const typename TInjection::TEnergy prob=1) {
			for(typename InjectionMap::iterator it=injections_.begin(); it!=injections_.end(); it++) {
				//if state is already killed or still to be created --> skip injecting activation by external sensors
				if(!it->second.state_->still_exists() || it->second.state_==ctxt->virtual_state()) continue;
			
				//check if feature is in active list --> add if we not too ambiguous (50% of max. size of active state list)
				if(2*injections_.size() < ctxt->param().max_active_states_)
					ctxt->add_to_active(it->second.state_);
				
				if(update)
					it->second.state_->update(ts, std::min(max_occ, (int)injections_.size()), est_occ, it->second.counter_, ft_cl, prob);
				
				DBG_PRINTF("injectXYZ %d -> %d with %d\n", id_, it->second.state_->id(), (int)(injections_.size()+est_occ));
			}
			DBG_PRINTF("inject %d\n", (int)injections_.size());
		}
		
		/**
		 * class: FeatureSerialization
		 * helper structure for serialization of a feature
		 */
		struct FeatureSerialization {
			Feature ft_;
		    std::vector<typename TInjection::ID> injs_;
		    std::vector<uint32_t> cnts_;
			
			FeatureSerialization() : ft_(-1)
			{}
			
			FeatureSerialization(const Feature &ft) : ft_(ft)
			{}
			
			UNIVERSAL_SERIALIZE()
			{
				assert(version==CURRENT_SERIALIZATION_VERSION);
				
				ar & UNIVERSAL_SERIALIZATION_NVP(ft_);
				ar & UNIVERSAL_SERIALIZATION_NVP(injs_);
				ar & UNIVERSAL_SERIALIZATION_NVP(cnts_);
			}
			
			void merge(const FeatureSerialization &o) {
				for(size_t j=0; j<o.injs_.size(); j++) {
					bool found=false;
					for(size_t i=0; i<injs_.size(); i++) {
						if(injs_[i]!=o.injs_[j]) continue;
						cnts_[i] = std::max(cnts_[i], o.cnts_[j]);
						found = true;
						break;
					}
					if(!found) {
						injs_.push_back(o.injs_[j]);
						cnts_.push_back(o.cnts_[j]);
					}
				}
			}
		};
		
		UNIVERSAL_SERIALIZE()
		{
		    assert(version==CURRENT_SERIALIZATION_VERSION);
		    
		    ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("id", id_);
		}
		
		FeatureSerialization get_serialization() const {
			FeatureSerialization fs(*this);
			
			for(typename InjectionMap::const_iterator it=injections_.begin(); it!=injections_.end(); it++) {
				if(!it->second.state_->still_exists()) continue;
				
				fs.injs_.push_back(it->second.state_->id());
				fs.cnts_.push_back(it->second.counter_);
				
				DBG_PRINTF("send ft (%d) cnt: %d <= %d for %d\n", id_, fs.cnts_.back(), it->second.state_->get_feature_class_counter(0), fs.injs_.back());
				assert(fs.cnts_.back()<=it->second.state_->get_feature_class_counter(0));
			}
			
			return fs;
		}
		
		template<class TGraph, class TMapStates>
		void set_serialization(const FeatureSerialization &fs, TGraph &graph, TMapStates &states) {
			id_ = fs.ft_.id();
			
			//reconnect features with states
			for(size_t i=0; i<fs.injs_.size(); i++) {
				for(typename TGraph::NodeIt it(graph); it!=lemon::INVALID; ++it)
					if(states[it]->id()==fs.injs_[i]) {
						DBG_PRINTF("check ft (%d) cnt: %d <= %d for %d\n", id_, fs.cnts_[i], states[it]->get_feature_class_counter(0), fs.injs_[i]);
						assert(fs.cnts_[i]<=states[it]->get_feature_class_counter(0));
						injections_.insert(typename InjectionMap::value_type(states[it].get(), Connection(states[it], fs.cnts_[i])));
						break;
					}
					
			}
		}
		
	};
}
