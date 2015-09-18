//path integration (input from odometry)
#include "../helpers/gpx.hpp"

template<class TGraph, class TMapStates, class TMapTransformations, typename TState>
void insert_state(TGraph &graph, TMapStates &states, TMapTransformations &trans, TState &new_state) {
	new_state->set_node(graph.addNode());
	states.set(new_state->node(), new_state);
}

template<class TTransformation, class TGraph, class TMapTransformations, typename TState>
void insert_transistion(TGraph &graph, TMapTransformations &trans, const TState &new_state, TTransformation &link) {
	ROS_ASSERT(link->src());
	trans.set(graph.addArc(new_state->node(), link->src()->node()), link);
	//trans.set(graph.addEdge(new_state->node(), link->src()->node()), link);
}

template<class TGraph, typename TState>
void remove_state(TGraph &graph, TState &state) {
	if(!state) return;
	graph.erase(state->node()); //deletes also arcs
}

template<class TTransformation, class TGraph, class TContext, class TMapStates, class TMapTransformations>
void init(TGraph &graph, TContext &ctxt, TMapStates &states, TMapTransformations &trans)
{
	typedef typename TContext::TState TState;

	typename TContext::TState::TPtr state(new TState);
	state->dist_dev() = 1;
	ctxt.active_states().push_back(state);
	insert_state(graph, states, trans, state);

	//ctxt.virtual_state().reset(new TState);
	//insert_state<TTransformation>(graph, states, trans, ctxt.virtual_state(), ctxt.current_active_state());
}
		
template<typename TStatePtr, typename TEnergy, typename TContext, typename TGraph, typename TStates, typename TTrans, typename TAction>
TEnergy inflow(TStatePtr &th, const TEnergy &offset, const TContext &ctxt, const TGraph &graph, const TStates &states, const TTrans &trans, const TAction &odom) /*const*/ {
	TEnergy I = 0;
	for(typename TStatePtr::element_type::TArcIterator ait(th->edge_begin(graph)); ait!=th->edge_end(graph); ++ait) {
		TStatePtr opposite = states[th->opposite_node(graph, ait)];
		const typename TAction::TType trans_fact = trans[ait]->directed(th).transition_factor(odom, ctxt.param().prox_thr_);
		ROS_ASSERT_MSG(trans_fact==0 || (opposite->outflow()>=0 && opposite->outflow()<=1), "outflow %f out of bounds [0,1] (d=%f)", opposite->outflow(), trans_fact);
		
		I = std::max(I, trans_fact*opposite->outflow());
	}
	DBG_PRINTF("inflow %f", I);
	I-=offset;
	
	//TODO: add sensor input here
	//I += p*(1-energy());
	
	return I;
}

template<class TStateVector, class TEnergyFactor, class TGraph, class TContext, class TMapStates, class TMapTransformations, class TTransformation>
void path_integration(TStateVector &active_states/*, const TEnergyFactor &weight*/, TGraph &graph, TContext &ctxt, TMapStates &states, TMapTransformations &trans, const TTransformation &odom, const Eigen::Vector3f &dbg_pose)
{
	typedef typename TContext::TState TState;
	typedef typename TStateVector::iterator TIter;
	typedef typename TState::TArcOutIterator TArcIter_out;
	typedef typename TState::TArcInIterator TArcIter_in;
	
	boost::lock_guard<boost::mutex> guard(ctxt.get_mutex());	//TODO: reduce locking area
	
#ifndef NDEBUG
	{
		TIter begin = active_states.begin();
		TIter end   = active_states.end();
		
		if(ctxt.needs_sort())
			std::sort(begin, end, sorting::energy_order<typename TStateVector::value_type>);
		assert( sorting::is_sorted(begin, end, sorting::energy_order<typename TStateVector::value_type>) );
		
		/*for(TIter it=begin; it!=end; it++) {
			DBG_PRINTF("state %f %f\n", (*it)->dist_h(), (*it)->dist_dev());
			if(it+1!=end)
				ROS_ASSERT( (*it)->d()<=(*(it+1))->d() );
		}*/
		ROS_ASSERT(ctxt.current_active_state()==*begin);
		
		//debug
		{
			int i=0;
			DBG_PRINTF("current_list\n");
			for(TIter it=begin; it!=end; it++) {
				bool gt = (dbg_pose.template head<2>()-(*it)->dbg().pose_.template head<2>()).norm()<ctxt.param().prox_thr_(0);
				gt &= (dbg_pose.template tail<1>()-(*it)->dbg().pose_.template tail<1>()).norm()<ctxt.param().prox_thr_(1);
				DBG_PRINTF("%d:\t %f:%f:%f:%d   %s\n", (*it)->id(), (*it)->dist_dev(), (*it)->d(), (*it)->dist_trv(), (*it)->hops(),
					gt?"MATCH_GT":""
				);
				++i;
				//if(i>3) break;
			}
			DBG_PRINTF("\n");
		}
	}
#endif
	
	ROS_ASSERT(ctxt.active_states().size()>0);
	ROS_ASSERT(ctxt.current_active_state());
	
	//if(ctxt.virtual_state() && ctxt.current_active_state()!=ctxt.virtual_state())
	//	ctxt.virtual_state()->dist_dev() = ctxt.current_active_state()->dist_dev();
	
	DBG_PRINTF("current id %d\n", ctxt.current_active_state()->id());

#ifndef NDEBUG
	static Debug_GPX gpx("/tmp/reloc.gpx");
	
	gpx.add_pt(ctxt.current_active_state()->dbg().pose_(1), ctxt.current_active_state()->dbg().pose_(0));
#endif

	if(!ctxt.virtual_state() || (ctxt.last_active_state()!=ctxt.current_active_state() && ctxt.current_active_state()->dist_trv()<=0) || (ctxt.current_active_state()!=ctxt.virtual_state() && ctxt.current_active_state()->d2()<ctxt.last_dist_min() && ctxt.current_active_state()->dist_trv()<=0) || ctxt.virtual_state()->dist_trv()<=0) {
		
		if(ctxt.virtual_state()) {
			ctxt.virtual_transistion()->dbg();
		}
	
		DBG_PRINTF("resetting virtual state %d (%f)\n",
			(int)(ctxt.last_active_state()!=ctxt.current_active_state()),
			ctxt.virtual_state()?ctxt.virtual_state()->dist_trv():0.
		);
		
		if(ctxt.current_active_state()==ctxt.virtual_state()) {
			DBG_PRINTF("virtual state is inserted to map (action dist.: %f)\n", ctxt.virtual_transistion()->dist(ctxt.param().prox_thr_));
			ctxt.virtual_transistion()->dbg();
		}
		else {
			DBG_PRINTF_URGENT("relocalized %d -> %d with %d hops\n", ctxt.last_active_state()?ctxt.last_active_state()->id():-1, ctxt.current_active_state()?ctxt.current_active_state()->id():-1,
			ctxt.current_active_state()?ctxt.current_active_state()->hops():0);
			
			if(ctxt.last_active_state()!=ctxt.current_active_state() && ctxt.last_active_state() && ctxt.current_active_state()) {
				
				bool exist=false;
				for(TArcIter_out ait(ctxt.current_active_state()->arc_out_begin(graph)); ait!=ctxt.current_active_state()->arc_out_end(graph); ++ait) {
					typename TIter::value_type opposite = states[ctxt.current_active_state()->opposite_node(graph, ait)];
					if(opposite==ctxt.virtual_transistion()->src()) {
						exist=true;
						break;
					}
				}
#ifdef BEDIRECTIONAL
				for(TArcIter_in ait(ctxt.current_active_state()->arc_in_begin(graph)); (!exist) && ait!=ctxt.current_active_state()->arc_in_end(graph); ++ait) {
					typename TIter::value_type opposite = states[ctxt.current_active_state()->opposite_node(graph, ait)];
					if(opposite==ctxt.virtual_transistion()->src()) {
						exist=true;
						break;
					}
				}
#endif
				
				if(exist) {
					DBG_PRINTF("not inserted new link as exists already\n");
				}
				else {
					insert_transistion(graph, trans, ctxt.current_active_state(), ctxt.virtual_transistion());
					DBG_PRINTF("inserted new link");
				}
			
#ifndef NDEBUG
				std::cout<<"old pose: "<<ctxt.last_active_state()->dbg().pose_.transpose()<<std::endl;
				std::cout<<"new pose: "<<ctxt.current_active_state()->dbg().pose_.transpose()<<std::endl;
				std::cout<<"cur pose: "<<dbg_pose.transpose()<<std::endl;
				
				bool gt = (dbg_pose.template head<2>()-ctxt.current_active_state()->dbg().pose_.template head<2>()).norm()<=ctxt.param().prox_thr_(0)*2;
				gt &= (dbg_pose.template tail<1>()-ctxt.current_active_state()->dbg().pose_.template tail<1>()).norm()<=ctxt.param().prox_thr_(1)*2;
				
				DBG_PRINTF_URGENT("pose match %f %f "
					, (dbg_pose.template head<2>()-ctxt.current_active_state()->dbg().pose_.template head<2>()).norm() * 40008000 / 360 
					, (ctxt.last_active_state()->dbg().pose_.template head<2>()-ctxt.current_active_state()->dbg().pose_.template head<2>()).norm() * 40008000 / 360 );
				DBG_PRINTF_URGENT( (dbg_pose.template head<2>()-ctxt.current_active_state()->dbg().pose_.template head<2>()).norm() * 40008000 / 360<=(dbg_pose(2)+ctxt.current_active_state()->dbg().pose_(2))?
					"SUCCESS1  ":"FAILED1  "
				);
				DBG_PRINTF_URGENT( (ctxt.last_active_state()->dbg().pose_.template head<2>()-ctxt.current_active_state()->dbg().pose_.template head<2>()).norm() * 40008000 / 360<=(ctxt.last_active_state()->dbg().pose_(2)+ctxt.current_active_state()->dbg().pose_(2))?
					"SUCCESS2 ":"FAILED2 "
				);
				DBG_PRINTF_URGENT( gt ? "SUCCESS3\n":"FAILED3\n"	);
#endif
			}
			
			/*ctxt.virtual_state()->still_exists() = false;
			for(TIter it=active_states.begin(); it!=active_states.end(); it++)
				if(*it == ctxt.virtual_state()) {
					active_states.erase(it);
					break;
				}*/

			ctxt.remove_state(ctxt.virtual_state());
			remove_state(graph, ctxt.virtual_state());
		}
		
		ctxt.virtual_state().reset(new TState);
		ctxt.virtual_state()->dist_trv()  = 1;
		ctxt.virtual_state()->dist_dev() = ctxt.current_active_state()->dist_dev();
		
		ctxt.virtual_state()->dbg().pose_ = dbg_pose;
		
		insert_state(graph, states, trans, ctxt.virtual_state());
		ctxt.virtual_transistion().reset(new TTransformation(ctxt.current_active_state()));
		insert_transistion(graph, trans, ctxt.virtual_state(), ctxt.virtual_transistion());

		ctxt.last_active_state() = ctxt.current_active_state();
		
		active_states.push_back(ctxt.virtual_state());
	}
	
	ctxt.last_dist_min() = ctxt.current_active_state()->d2();
	
	ROS_ASSERT(ctxt.virtual_state());
	ROS_ASSERT(ctxt.virtual_transistion());

	ctxt.virtual_transistion()->integrate(odom);
	
	{
		TIter begin = active_states.begin();
		TIter end   = active_states.end();
		
		//step 2: increase dist.
		for(TIter it=begin; it!=end; it++) {
			//if( (*it)->id() < ctxt.virtual_state()->id()-ctxt.param().min_age_ && (*it)->get_feature_prob()>0)
			//	continue;
			
			DBG_PRINTF("%d before dist (%f/%f) h:%d\n", (*it)->id(), (*it)->dist_trv(),(*it)->dist_dev(), (*it)->hops());	
	
			if( (*it)==ctxt.virtual_state()) {
				const typename TState::TEnergy du = odom.dist_uncertain(ctxt.param().prox_thr_), dc = odom.dist(ctxt.param().prox_thr_);
				
				(*it)->dist_trv() =  std::max((typename TState::TEnergy)-1, 1-ctxt.virtual_transistion()->dist(ctxt.param().prox_thr_));
				(*it)->dist_dev() += (du-dc);
				
				DBG_PRINTF("%d changing dist V (%f/%f) by (%f/%f)\n", (*it)->id(), (*it)->dist_trv(),(*it)->dist_dev(), dc, du-dc);
			}
			else {
				typename TState::TEnergy dh_max = 0;
				
				if((*it)->dist_trv()>0) {
					for(TArcIter_out ait((*it)->arc_out_begin(graph)); ait!=(*it)->arc_out_end(graph); ++ait)
						dh_max = std::max(dh_max, trans[ait]->directed(*it).transition_factor(odom, ctxt.param().prox_thr_));
					
					(*it)->dist_trv() -= dh_max/*+odom.deviation()*/;
				}
				else {
					/*if( (*it)->id() < ctxt.virtual_state()->id()-ctxt.param().min_age_ && (*it)->get_feature_prob()>0)
						odom.dbg();
						
					for(TArcIter_in ait((*it)->arc_in_begin(graph)); ait!=(*it)->arc_in_end(graph); ++ait) {
						tmp = trans[ait]->directed(*it).transition_factor(odom, ctxt.param().prox_thr_);
						if(dh_max<tmp) {
							dh_max = tmp;
							dev_max= trans[ait]->deviation();
						}
						
						if( (*it)->id() < ctxt.virtual_state()->id()-ctxt.param().min_age_ && (*it)->get_feature_prob()>0) {
							trans[ait]->directed(*it).dbg();
							std::cout<<"t-factor2: "<<trans[ait]->directed(*it).transition_factor(odom, ctxt.param().prox_thr_)<<std::endl;
							
							/*typename TIter::value_type opposite = states[(*it)->opposite_node(graph, ait)];
							trans[ait]->directed(opposite).dbg();
							std::cout<<"t-factor: "<<trans[ait]->directed(opposite).transition_factor(odom, ctxt.param().prox_thr_)<<std::endl;*
						}
					}*/
					
					(*it)->dist_trv() -= odom.dist(ctxt.param().prox_thr_);
				}
				
				const typename TState::TEnergy travel = odom.dist(ctxt.param().prox_thr_);
				const typename TState::TEnergy dist = std::abs(travel-dh_max);
				
				(*it)->dist_dev() += dist+odom.deviation()*std::exp(-(*it)->hops()/5.f);
				
				DBG_PRINTF("%d changing dist(%f/%f)\n", (*it)->id(),
					(*it)->dist_trv(),(*it)->dist_dev());
				/*DBG_PRINTF("%d changing dist(%f/%f) by (%f/%f) %f %f %f\nfeature factor: %f/%f from %f %f %f %f\n",
					(*it)->id(),
					(*it)->dist_h(),(*it)->dist_dev(),
					delta, std::sqrt(std::pow(odom.dist_uncertain(ctxt.param().prox_thr_),2)-delta*delta),
					odom.dist(ctxt.param().prox_thr_)+odom.deviation(), std::pow(odom.dist(ctxt.param().prox_thr_),2)-delta*delta,
					(*it)->get_feature_prob(),
					(1-1.f/(ctxt.param().est_occ_+1)*std::exp(-dist*dist/(2*travel*travel*(dev_max*dev_max+odom_dev*odom_dev)))), 1-std::sqrt(2*M_PI)*odom_dev*dev_max*std::exp(-dist*dist/(2*(dev_max*dev_max+odom_dev*odom_dev)))/std::sqrt(dev_max*dev_max+odom_dev*odom_dev),
					odom_dev, dev_max, dist, travel);*/
					
			}
			
			assert( (*it)->dist_trv()==(*it)->dist_trv() );
			assert( (*it)->dist_dev()==(*it)->dist_dev() );
		}
	}
	
#ifndef NDEBUG
	{ //DEBUG
		if(ctxt.virtual_state()->dbg().info_.find("V")==std::string::npos) ctxt.virtual_state()->dbg().info_ +="V ";
		if(ctxt.current_active_state()->dbg().info_.find("C")==std::string::npos) ctxt.current_active_state()->dbg().info_+="C ";
		
		ctxt.virtual_transistion()->dbg();
	} //DEBUG
#endif
	
	TIter begin = active_states.begin();
	TIter end   = active_states.end();
	
	std::list<typename TIter::value_type> to_be_added;
	
	//step 1: set min. dist.
	for(TIter it=begin; it!=end; it++) {
		typename TState::TEnergy dh_max = 0;
		int hops = -1;
		
		for(TArcIter_in ait((*it)->arc_in_begin(graph)); ait!=(*it)->arc_in_end(graph); ++ait) {
				typename TIter::value_type opposite = states[(*it)->opposite_node(graph, ait)];
				if( (*it)->dist_trv()>0.35/*trans[ait]->deviation()*/ || opposite->id() >= ctxt.virtual_state()->id()-ctxt.param().min_age_ /*|| (*it)->dist_h_out()<0.5-odom.deviation()-trans[ait]->deviation()*/ ) continue;
				
				if( trans[ait]!=ctxt.virtual_transistion() && 
					(!opposite->is_active() || opposite->dist_dev() > (*it)->dist_dev())
					//&& trans[ait]->directed(*it).transition_factor(odom, ctxt.param().prox_thr_)>odom.deviation()
					/*std::pow(trans[ait]->dist(ctxt.param().prox_thr_),2) + std::pow(opposite->dist_dev(),2)
					<
					(*it)->d2()*/
				) {	
					to_be_added.push_back(opposite);
										
					opposite->dist_trv()  = std::min((typename TState::TEnergy)1, std::max((typename TState::TEnergy)-1, trans[ait]->dist(ctxt.param().prox_thr_)+(*it)->dist_trv()));
					opposite->dist_dev() = (*it)->dist_dev();
					opposite->hops() = std::max(opposite->hops(), 1+(*it)->hops());
					
					DBG_PRINTF("1 %d:%d setting dist %f/%f with %d hops (dir %f)\n",
						opposite->id(), (*it)->id(),
						opposite->dist_trv(), opposite->dist_dev(),
						opposite->hops(),
						trans[ait]->directed(opposite).transition_factor(odom, ctxt.param().prox_thr_));
				}
		}
		
	}
	
	//step 0: update feature prob.
	for(TIter it=begin; it!=end; it++) {
		if( (*it)->id() >= ctxt.virtual_state()->id()-ctxt.param().min_age_ )
			continue;
			
		typename TState::TEnergy ft_prob = (*it)->get_feature_prob();
			
		assert(ft_prob>=0 && ft_prob<=1);
		
		if(ft_prob)
			DBG_PRINTF("%d: injecting energy %f (feature probability)    %f*%f  %d hops  %f %f\n", (*it)->id(), ft_prob, (*it)->get_feature_prob(), odom.dist(ctxt.param().prox_thr_), (*it)->hops(), (*it)->dist_dev(), (*begin)->dist_dev());
		
		(*it)->dist_dev() -= std::min((typename TState::TEnergy)1, (*it)->dist_dev()) * std::min((typename TState::TEnergy)1, ft_prob);
		
		if(ft_prob)
			DBG_PRINTF("%d: new energy %f\n", (*it)->id(), (*it)->dist_dev());
	}
	
	for(typename std::list<typename TIter::value_type>::iterator it=to_be_added.begin(); it!=to_be_added.end(); it++)
		ctxt.add_to_active(*it);
	
	//sort again
	{
		begin = active_states.begin();
		end   = active_states.end();
		std::sort(begin, end, sorting::energy_order<typename TStateVector::value_type>);
	}
	
#ifndef NDEBUG
	//debug
	{
		int i=0;
		DBG_PRINTF("distlist");
		for(TIter it=begin; it!=end; it++) {
			DBG_PRINTF("\t %f:%d:%d", (*it)->d(), (*it)->dist_trv()<=0, (*it)->id());
			++i;
			if(i>3) break;
		}
		DBG_PRINTF("\n");
	}
#endif
}

template<class TStateVector>
void reset_features(TStateVector &active_states)
{
	typedef typename TStateVector::iterator TIter;
	
	TIter begin = active_states.begin();
	TIter end   = active_states.end();
	
	//reset features
	for(TIter it=begin; it!=end; it++) {
		(*it)->reset_feature();
	}
}

