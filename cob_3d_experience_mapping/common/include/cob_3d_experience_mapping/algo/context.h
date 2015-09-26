#pragma once

#include "../defs.h"
#include "../param.h"
#include <set>
#include <map>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include "../helpers/network.h"

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
	template<class _TEnergy, class _TState, class _TFeature, class _TEnergyFactor, class _TTransform, class _TIdTsGenerator = SimpleIdTsGenerator<_TState, _TFeature> >
	class Context {
	public:
		typedef _TEnergy TEnergy;
		typedef _TState TState;
		typedef _TIdTsGenerator TIdTsGenerator;
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
		TIdTsGenerator id_generator_;
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
		TIdTsGenerator &id_generator() {return id_generator_;}
		
		//!< add a state to the active state list + init. variables + (init. distances if needed)
		void add_to_active(typename TState::TPtr &state, const bool already_set=false) {
			if(!current_active_state() && !already_set)
				return;
			
			//if already present in active list -> skip
			for(size_t i=0; i<active_states_.size(); i++)
				if(active_states_[i]==state) {
					if(!already_set) {
						state->dist_dev() 	= std::min(state->dist_dev(), current_active_state()->dist_dev()+1);
						needs_sort_ = true;
					}
					return;
				}
				
			//somebody else will set this variables from outside
			if(!already_set) {
				state->dist_dev() 	= current_active_state()->dist_dev()+1;
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
			id_generator().register_removal(state);
			
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
		
		void visited_feature_class(const typename TState::TFeatureClass &ft_class) {
			if(current_active_state()) {
				current_active_state()->visited_feature_class(ft_class);
				id_generator().register_modification(current_active_state());
			}
		}
		
		//!< if a feature was seen we connect the active state with the feature and update all connected states
		void add_feature(const typename TFeature::TID &id, const int ts, const typename TState::TFeatureClass &ft_class) {
			boost::lock_guard<boost::mutex> guard(mtx_);
			
			//if( current_active_state() && virtual_state() && current_active_state()->id() < virtual_state()->id()-(param().min_age_+3) )
			//	return;
			
			bool inject = true;
			for(typename FeatureBuffer::iterator it = last_features_.begin(); it!=last_features_.end(); it++)
				if(*it == id) {
					DBG_PRINTF("feature %d already set\n", id);
					inject = false;
					break;
				}
			last_features_.push_back(id);
			
			bool modified = false;
			typename FeatureMap::iterator it = features_.find(id);
			if(it==features_.end()) {
				it = features_.insert(typename FeatureMap::value_type(id, typename TFeature::TPtr(new TFeature(id))) ).first;
				modified = true;
			}
			//if( !(current_active_state() && virtual_state() && current_active_state()->id() < virtual_state()->id()-(param().min_age_+3) ) )
			if( current_active_state() )
				modified |= it->second->visited(current_active_state().get(), current_active_state());
			if(inject)
				it->second->inject(this, ts, param().est_occ_, param().max_active_states_, ft_class);
			
			if(modified)
				id_generator().register_modification(it->second);
			id_generator().register_modification(current_active_state());
		}
		
		void merge_feature(const typename TFeature::TID &id, const typename TFeature::TPtr &ft) {
			typename FeatureMap::iterator it = features_.find(id);
			if(it==features_.end())
				it = features_.insert(typename FeatureMap::value_type(id, ft) ).first;
			else
				it->second->merge(ft);
		}

		UNIVERSAL_SERIALIZE()
		{
		    assert(version==CURRENT_SERIALIZATION_VERSION);;
		    
		    ar & UNIVERSAL_SERIALIZATION_NVP(param_);
		}
	};
	
	
	template<class TContext, class TGraph, class TMapStates, class TMapTransformations>
	class ContextContainer {
	protected:
		TContext *ctxt_;
		TGraph *graph_;
		TMapStates *states_;
		TMapTransformations *trans_;
		
	public:			
		ContextContainer() :
		 ctxt_(NULL), graph_(NULL), states_(NULL), trans_(NULL)
		 {}
		 
		ContextContainer(TContext *ctxt, TGraph *graph, TMapStates *states, TMapTransformations *trans) :
		 ctxt_(ctxt), graph_(graph), states_(states), trans_(trans)
		 {}
		    	
		UNIVERSAL_SERIALIZE()
		{
		    assert(version==CURRENT_SERIALIZATION_VERSION);
		    
		    //we have to lock everything for consistency
			boost::mutex mtx_tmp;
			boost::lock_guard<boost::mutex> guard(ctxt_ ? ctxt_->get_mutex() : mtx_tmp);
		    
			if(ctxt_)
			 ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("ctxt", *ctxt_);
			else {
				TContext tmp;
				ar & UNIVERSAL_SERIALIZATION_NVP_NAMED("ctxt", tmp);
			}
		    
			std::vector<serialization::serializable_shared_ptr<typename TContext::TState> > states;
			std::vector<typename TContext::TState::TransitionSerialization> trans;
		    std::vector<typename TContext::TFeature::FeatureSerialization> fts;
			
			//on saving
		    if(graph_ && states_ && trans_ && UNIVERSAL_CHECK<Archive>::is_saving(ar)) {
				
				for(typename TGraph::NodeIt it(*graph_); it!=lemon::INVALID; ++it) {
					states.push_back((*states_)[it]);
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
				//clear everything
				graph_->clear();
				ctxt_->active_states().clear();
				
				//insert states
				for(size_t i=0; i<states.size(); i++) {
					typename TContext::TState::TPtr c = states[i];
					
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
	
	
	template<class _TState, class _TFeature>
	class ClientIdTsGenerator {
	public:
		typedef _TState TState;
		typedef _TFeature TFeature;
		typedef typename TState::ID ID;
		
	private:
		ID running_id_;
		
		std::map<ID, typename TState::TPtr>   modification_states_;
		std::map<ID, typename TFeature::TPtr> modification_fts_;
		
	public:
		ClientIdTsGenerator() : running_id_(1)
		{}
		
		ID new_id() {return running_id_++;}
		
		void register_modification(const typename TState::TPtr &state)
		{
			assert(state);
			modification_states_[state->id()] = state;
		}
		void register_modification(const typename TFeature::TPtr &ft)
		{
			assert(ft);
			modification_fts_[ft->id()] = ft;
		}
		void register_removal(const typename TState::TPtr &state)
		{
			assert(state);
			modification_states_[state->id()] = state;
		}
		
		void clear() {
			modification_states_.clear();
			modification_fts_.clear();
		}
		
		void get_lists(std::vector<serialization::serializable_shared_ptr<TState> > &updated_states, std::vector<ID> &removed_states, std::vector<typename TFeature::TPtr> &updated_fts)
		{
			for(typename std::map<ID, typename TState::TPtr>::iterator it = modification_states_.begin(); it!=modification_states_.end(); it++) {
				if( it->second->still_exists() )
					updated_states.push_back( serialization::serializable_shared_ptr<TState>(it->second) );
				else
					removed_states.push_back( it->first );
			}
			
			for(typename std::map<ID, typename TFeature::TPtr>::iterator it = modification_fts_.begin(); it!=modification_fts_.end(); it++)
				updated_fts.push_back( it->second );
		}
		
	};
	
	template<class TContext, class TGraph, class TMapStates, class TMapTransformations, class _TClientId, class TArchiveIn = boost::archive::binary_iarchive, class TArchiveOut = boost::archive::binary_oarchive>
	class IncrementalContextContainer : public ContextContainer<TContext, TGraph, TMapStates, TMapTransformations> {
		
	public:
		typedef _TClientId TClientId;
		typedef typename TContext::TState TState;
		typedef typename TContext::TFeature TFeature;
		typedef typename TState::ID ID;
		typedef serialization::NetworkHeader<TClientId, hiberlite::sqlid_t> TNetworkHeader;
		
	private:
		TNetworkHeader net_header_;
#ifdef SERVER_
		boost::shared_ptr<hiberlite::Database> server_;
#endif
		
		std::vector<serialization::serializable_shared_ptr<TState> > copy_updated_states_;
		std::vector<typename TContext::TFeature::FeatureSerialization> copy_fts_;
		std::vector<typename TContext::TState::TransitionSerialization> copy_trans_;
		
	public:
		
		IncrementalContextContainer() :
		 ContextContainer<TContext, TGraph, TMapStates, TMapTransformations>()
		 {}
		 
		IncrementalContextContainer(TContext *ctxt, TGraph *graph, TMapStates *states, TMapTransformations *trans, const TClientId &client_id=(TClientId)-1) :
		 ContextContainer<TContext, TGraph, TMapStates, TMapTransformations>(ctxt, graph, states, trans)
		{
			 //if(ctxt) ctxt->id_generator().set_client_id(client_id);
		}
		
		TNetworkHeader get_network_header() const {
			return net_header_;
		}
		
		void set_network_header(const TNetworkHeader &nh) {
			net_header_ = nh;
		}
		
		void upload(const char * addr, const char * port, const int timeout_secs=120) {
			serialization::sync_content_client<TArchiveIn, TArchiveOut> (*this, addr, port, timeout_secs);
			//this->ctxt->id_generator().set_client_id(net_header_.client_);
			this->ctxt_->id_generator().clear();
		}
		
#ifdef SERVER_
		void on_client(std::iostream &stream, TClientId &running_client_id) {
			clock_t begin = clock();
			
			if(!server_) {
				server_.reset( new hiberlite::Database("sample.db") );
				
				server_->registerBeanClass<TState>();
				server_->registerBeanClass<typename TContext::TState::TransitionSerialization>();
				server_->registerBeanClass<typename TContext::TFeature::FeatureSerialization>();
				
				server_->createModel();
			}
			
			DBG_PRINTF("took_1 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
			serialization::sync_content_server_import_header<TArchiveIn, TArchiveOut> (*this, stream);
			
			if(net_header_.client_==(TClientId)-1) {
				net_header_.client_ = running_client_id;
				
				DBG_PRINTF("got new client, setting id to %d\n", net_header_.client_);
			}
			
			//update next client id
			running_client_id = std::max(running_client_id, (TClientId)(net_header_.client_+1));
			
			copy_updated_states_.clear();
			copy_fts_.clear();
			copy_trans_.clear();
			
			DBG_PRINTF("took_2 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
			std::vector< hiberlite::bean_ptr<TState> > v_s = server_->getAllBeanAfter<TState>(net_header_.ts_states_);
			for(size_t i=0; i<v_s.size(); i++) {
				copy_updated_states_.push_back( serialization::serializable_shared_ptr<TState>(v_s[i].shared_ptr()));
				net_header_.ts_states_ = std::max(v_s[i].get_id(), net_header_.ts_states_);
			}
			
			std::vector< hiberlite::bean_ptr<typename TContext::TFeature::FeatureSerialization> > v_f = server_->getAllBeanAfter<typename TContext::TFeature::FeatureSerialization>(net_header_.ts_fts_);
			for(size_t i=0; i<v_f.size(); i++) {
				copy_fts_.push_back(*v_f[i]);
				net_header_.ts_fts_ = std::max(v_f[i].get_id(), net_header_.ts_fts_);
				v_f[i].shared_ptr(); //prevent db update
			}
			
			std::vector< hiberlite::bean_ptr<typename TContext::TState::TransitionSerialization> > v_t = server_->getAllBeanAfter<typename TContext::TState::TransitionSerialization>(net_header_.ts_trans_);
			for(size_t i=0; i<v_t.size(); i++) {
				copy_trans_.push_back(*v_t[i]);
				net_header_.ts_trans_ = std::max(v_t[i].get_id(), net_header_.ts_trans_);
				v_t[i].shared_ptr(); //prevent db update
			}
			
			DBG_PRINTF("took_3 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
				
			serialization::sync_content_server_import<TArchiveIn, TArchiveOut> (*this, stream);
			
			DBG_PRINTF("took_4 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
			serialization::sync_content_server_export<TArchiveIn, TArchiveOut> (*this, stream);
			
			DBG_PRINTF("took_5 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
			//ok, now set the loaded content to the list and export it
			copy_updated_states_.clear();
			copy_fts_.clear();
			copy_trans_.clear();
		}
#endif
		  
		UNIVERSAL_SERIALIZE()
		{
			clock_t begin = clock();
			
			std::vector<serialization::serializable_shared_ptr<TState> > updated_states;
			std::vector<ID> removed_states;
			std::vector<typename TContext::TState::TransitionSerialization> trans;
		    std::vector<typename TContext::TFeature::FeatureSerialization> fts;
		    
			DBG_PRINTF("took_A1 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
			if(this->ctxt_ && this->graph_ && this->states_ && this->trans_ && UNIVERSAL_CHECK<Archive>::is_saving(ar)) {
#ifdef SERVER_
					get_lists_server(updated_states, fts, trans, removed_states);
#else
					get_lists_client(updated_states, fts, trans, removed_states);
#endif
			}
		    
			DBG_PRINTF("took_A2 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
		    ar & UNIVERSAL_SERIALIZATION_NVP(fts);
		    ar & UNIVERSAL_SERIALIZATION_NVP(updated_states);
		    ar & UNIVERSAL_SERIALIZATION_NVP(removed_states);
		    ar & UNIVERSAL_SERIALIZATION_NVP(trans);
		    
			DBG_PRINTF("took_A3 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			DBG_PRINTF("serialize size %d %d %d\n", (int)updated_states.size(), (int)trans.size(), (int)removed_states.size());
		    
			if(this->ctxt_ && this->graph_ && this->states_ && this->trans_ && UNIVERSAL_CHECK<Archive>::is_loading(ar)) {
#ifdef SERVER_
				if(server_)
					on_loaded_server(updated_states, fts, trans, removed_states);
#else
					on_loaded_client(updated_states, fts, trans, removed_states);
#endif
			}
		    
			DBG_PRINTF("took_A4 %f\n", double(clock() - begin) / CLOCKS_PER_SEC);
			
		}
		
		virtual void get_lists_server(
			std::vector<serialization::serializable_shared_ptr<TState> > &updated_states,
			std::vector<typename TContext::TFeature::FeatureSerialization> &fts, 
			std::vector<typename TContext::TState::TransitionSerialization> &trans,
			std::vector<ID> &removed_states
		) {
			updated_states = copy_updated_states_;
			fts = copy_fts_;
			trans = copy_trans_;
			removed_states.clear();
		}
		
		virtual void get_lists_client(
			std::vector<serialization::serializable_shared_ptr<TState> > &updated_states,
			std::vector<typename TContext::TFeature::FeatureSerialization> &fts, 
			std::vector<typename TContext::TState::TransitionSerialization> &trans,
			std::vector<ID> &removed_states
		) {
		    //we have to lock everything for consistency
			boost::mutex mtx_tmp;
			boost::lock_guard<boost::mutex> guard(this->ctxt_ ? this->ctxt_->get_mutex() : mtx_tmp);
			
			std::vector<typename TFeature::TPtr> updated_fts;
			
			this->ctxt_->id_generator().get_lists(updated_states, removed_states, updated_fts);
			
			for(size_t i=0; i<updated_states.size(); i++)
				updated_states[i]->get_trans(trans, *this->graph_, *this->states_, *this->trans_);

			for(size_t i=0; i<updated_fts.size(); i++)
				fts.push_back( updated_fts[i]->get_serialization() );
		}
		
#ifdef SERVER_
		virtual void on_loaded_server(
			std::vector<serialization::serializable_shared_ptr<TState> > &updated_states,
			std::vector<typename TContext::TFeature::FeatureSerialization> &fts, 
			std::vector<typename TContext::TState::TransitionSerialization> &trans,
			const std::vector<ID> &removed_states
		) {
			assert(server_);
			
			//remove old or modified entries
			//TODO: do this
			
			std::vector< hiberlite::bean_ptr<typename TContext::TFeature::FeatureSerialization> > v_f = server_->getAllBeans<typename TContext::TFeature::FeatureSerialization>();
			for(size_t i=0; i<v_f.size(); i++) {
				DBG_PRINTF("ft %d %d\n", (int)i, (int)(v_f[i].operator->()!=NULL));
				if(v_f[i].operator->()==NULL) continue;
				
				bool found = false;
				for(size_t j=0; j<fts.size(); j++) {
					if(v_f[i]->ft_.id()==fts[j].ft_.id()) {
						DBG_PRINTF("removed old feature from db\n");
						v_f[i].destroy();	//remove old one
						found = true;
						break;
					}
				}
				if(!found) v_f[i].shared_ptr(); //prevent db update
			}
			
			//save to db	
			server_->begin_transaction();
			for(size_t i=0; i<updated_states.size(); i++) {
				updated_states[i]->set_id( (updated_states[i]->id()<<8) | net_header_.client_ );
				hiberlite::bean_ptr<TState> p=server_->copyBean(*updated_states[i]);
				net_header_.ts_states_ = std::max(p.get_id(), net_header_.ts_states_);
			}
			
			for(size_t i=0; i<trans.size(); i++) {
				trans[i].src_ = (trans[i].src_<<8) | net_header_.client_;
				trans[i].dst_ = (trans[i].dst_<<8) | net_header_.client_;
				hiberlite::bean_ptr<typename TContext::TState::TransitionSerialization> p=server_->copyBean(trans[i]);
				net_header_.ts_trans_ = std::max(p.get_id(), net_header_.ts_trans_);
			}
			
			for(size_t i=0; i<fts.size(); i++) {
				for(size_t j=0; j<fts[i].injs_.size(); j++)
					fts[i].injs_[j] = (fts[i].injs_[j]<<8) | net_header_.client_;
				hiberlite::bean_ptr<typename TContext::TFeature::FeatureSerialization> p=server_->copyBean(fts[i]);
				net_header_.ts_fts_ = std::max(p.get_id(), net_header_.ts_fts_);
			}
			server_->commit_transaction();
		}
#endif
		
		virtual void on_loaded_client(
			const std::vector<serialization::serializable_shared_ptr<TState> > &updated_states,
			const std::vector<typename TContext::TFeature::FeatureSerialization> &fts, 
			const std::vector<typename TContext::TState::TransitionSerialization> &trans,
			const std::vector<ID> &removed_states
		) {
		    //we have to lock everything for consistency
			boost::mutex mtx_tmp;
			boost::lock_guard<boost::mutex> guard(this->ctxt_ ? this->ctxt_->get_mutex() : mtx_tmp);
			
			//insert states
			for(size_t i=0; i<updated_states.size(); i++) {
				DBG_PRINTF("load state %d", (int)updated_states[i]->id());
				bool found = false;
				for(typename TGraph::NodeIt it(*this->graph_); it!=lemon::INVALID; ++it) {
					if( (*this->states_)[it]->id() == updated_states[i]->id() ) {
						DBG_PRINTF(": updated\n");
						*(*this->states_)[it] = *updated_states[i];
						found = true;
						break;
					}
				}
					
				if(found) continue;
				DBG_PRINTF("\n");
				
				typename TContext::TState::TPtr c(updated_states[i]);
				
				c->set_node(this->graph_->addNode());
				this->states_->set(c->node(), c);
			}
			
			for(size_t i=0; i<removed_states.size(); i++) {
				bool found = false;
				for(typename TGraph::NodeIt it(*this->graph_); it!=lemon::INVALID; ++it) {
					if( (*this->states_)[it]->id() == removed_states[i] ) {
						this->ctxt_->remove_state( (*this->states_)[it] );
						found = true;
						break;
					}
				}
				
				if(!found)
					DBG_PRINTF_URGENT("WARNING: could not find state to be removed\n");
			}
			
			//insert transitions (TODO: speed up)
			for(typename TGraph::NodeIt it(*this->graph_); it!=lemon::INVALID; ++it)
				(*this->states_)[it]->set_trans(trans, *this->graph_, *this->states_, *this->trans_);
			
			//insert features
			for(size_t i=0; i<fts.size(); i++) {
				typename TContext::TFeature *tmp = new typename TContext::TFeature(fts[i].ft_.id());
				tmp->set_serialization(fts[i], *this->graph_, *this->states_);
				
				this->ctxt_->merge_feature(fts[i].ft_.id(), typename TContext::TFeature::TPtr(tmp));
			}
		}
		
	};
	
}
