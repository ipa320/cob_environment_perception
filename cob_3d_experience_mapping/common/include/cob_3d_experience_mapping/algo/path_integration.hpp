//path integration (input from odometry)

template<class TGraph, class TMapCells, class TMapTransformations, typename TState>
void insert_cell(TGraph &graph, TMapCells &cells, TMapTransformations &trans, TState &new_cell) {
	new_cell->set_node(graph.addNode());
	cells.set(new_cell->node(), new_cell);
}

template<class TTransformation, class TGraph, class TMapTransformations, typename TState>
void insert_transistion(TGraph &graph, TMapTransformations &trans, const TState &new_cell, TTransformation &link) {
	ROS_ASSERT(link->src());
	trans.set(graph.addArc(new_cell->node(), link->src()->node()), link);
	//trans.set(graph.addEdge(new_cell->node(), link->src()->node()), link);
}

template<class TGraph, typename TState>
void remove_cell(TGraph &graph, TState &cell) {
	if(!cell) return;
	graph.erase(cell->node()); //deletes also arcs
}

template<class TTransformation, class TGraph, class TContext, class TMapCells, class TMapTransformations>
void init(TGraph &graph, TContext &ctxt, TMapCells &cells, TMapTransformations &trans)
{
	typedef typename TContext::TState TState;

	typename TContext::TState::TPtr cell(new TState);
	//cell->energy() = 1;
	ctxt.active_cells().push_back(cell);
	insert_cell(graph, cells, trans, cell);

	//ctxt.virtual_cell().reset(new TState);
	//insert_cell<TTransformation>(graph, cells, trans, ctxt.virtual_cell(), ctxt.current_active_cell());
}
		
template<typename TStatePtr, typename TEnergy, typename TContext, typename TGraph, typename TCells, typename TTrans, typename TAction>
TEnergy inflow(TStatePtr &th, const TEnergy &offset, const TContext &ctxt, const TGraph &graph, const TCells &cells, const TTrans &trans, const TAction &odom) /*const*/ {
	TEnergy I = 0;
	for(typename TStatePtr::element_type::TArcIterator ait(th->edge_begin(graph)); ait!=th->edge_end(graph); ++ait) {
		TStatePtr opposite = cells[th->opposite_node(graph, ait)];
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

template<class TCellVector, class TEnergyFactor, class TGraph, class TContext, class TResultList, class TMapCells, class TMapTransformations, class TTransformation>
void path_integration(TCellVector &active_cells/*, const TEnergyFactor &weight*/, TGraph &graph, TContext &ctxt, TMapCells &cells, TMapTransformations &trans, const TTransformation &odom, TResultList &result, const Eigen::Vector3f &dbg_pose)
{
	typedef typename TContext::TState TState;
	typedef typename TCellVector::iterator TIter;
	typedef typename TState::TArcOutIterator TArcIter_out;
	typedef typename TState::TArcInIterator TArcIter_in;
	
	boost::lock_guard<boost::mutex> guard(ctxt.get_mutex());	//TODO: reduce locking area
	
	{
		TIter begin = active_cells.begin();
		TIter end   = active_cells.end();
		std::sort(begin, end, energy_order<typename TCellVector::value_type>);
		/*for(TIter it=begin; it!=end; it++) {
			DBG_PRINTF("cell %f %f\n", (*it)->dist_h(), (*it)->dist_o());
			if(it+1!=end)
				ROS_ASSERT( (*it)->d()<=(*(it+1))->d() );
		}*/
		ROS_ASSERT(ctxt.current_active_cell()==*begin);
		
		//debug
		{
			int i=0;
			DBG_PRINTF("current_list\n");
			for(TIter it=begin; it!=end; it++) {
				DBG_PRINTF("%d:\t %f:%f:%d\n", (*it)->id(), (*it)->d(), (*it)->dist_h(), (*it)->dbg().hops_);
				++i;
				//if(i>3) break;
			}
			DBG_PRINTF("\n");
		}
	}
	
	ROS_ASSERT(ctxt.active_cells().size()>0);
	ROS_ASSERT(ctxt.current_active_cell());
	
	//if(ctxt.virtual_cell() && ctxt.current_active_cell()!=ctxt.virtual_cell())
	//	ctxt.virtual_cell()->dist_o() = ctxt.current_active_cell()->dist_o();
	
	DBG_PRINTF("current id %d\n", ctxt.current_active_cell()->id());

	if(!ctxt.virtual_cell() || (ctxt.last_active_cell()!=ctxt.current_active_cell() && ctxt.current_active_cell()->dist_h()<=0) || (ctxt.current_active_cell()!=ctxt.virtual_cell() && ctxt.current_active_cell()->d2()<ctxt.last_dist_min() && ctxt.current_active_cell()->dist_h()<=0) || ctxt.virtual_cell()->dist_h()<=0) {
		DBG_PRINTF("resetting virtual cell %d (%f %f)\n",
			(int)(ctxt.last_active_cell()!=ctxt.current_active_cell()),
			ctxt.virtual_cell()?ctxt.virtual_cell()->dist_h():0., ctxt.virtual_cell()?ctxt.virtual_cell()->dist_o():0.
		);
		
		if(ctxt.current_active_cell()==ctxt.virtual_cell()) {
			DBG_PRINTF("virtual cell is inserted to map (action dist.: %f)\n", ctxt.virtual_transistion()->dist(ctxt.param().prox_thr_));
			ctxt.virtual_transistion()->dbg();
		}
		else {
			DBG_PRINTF("relocalized %d -> %d with %d hops\n", ctxt.last_active_cell()?ctxt.last_active_cell()->id():-1, ctxt.current_active_cell()?ctxt.current_active_cell()->id():-1,
			ctxt.current_active_cell()?ctxt.current_active_cell()->dbg().hops_:0);
			
			if(ctxt.last_active_cell()!=ctxt.current_active_cell() && ctxt.last_active_cell() && ctxt.current_active_cell()) {
				
				bool exist=false;
				for(TArcIter_out ait(ctxt.current_active_cell()->arc_out_begin(graph)); ait!=ctxt.current_active_cell()->arc_out_end(graph); ++ait) {
					typename TIter::value_type opposite = cells[ctxt.current_active_cell()->opposite_node(graph, ait)];
					if(opposite==ctxt.virtual_transistion()->src()) {
						exist=true;
						break;
					}
				}
				
				if(exist) {
					DBG_PRINTF("not inserted new link as exists already\n");
				}
				else {
					insert_transistion(graph, trans, ctxt.current_active_cell(), ctxt.virtual_transistion());
					DBG_PRINTF("inserted new link");
				}
			
				std::cout<<"old pose: "<<ctxt.last_active_cell()->dbg().pose_.transpose()<<std::endl;
				std::cout<<"new pose: "<<ctxt.current_active_cell()->dbg().pose_.transpose()<<std::endl;
			}
			
			/*ctxt.virtual_cell()->still_exists() = false;
			for(TIter it=active_cells.begin(); it!=active_cells.end(); it++)
				if(*it == ctxt.virtual_cell()) {
					active_cells.erase(it);
					break;
				}*/

			ctxt.remove_cell(ctxt.virtual_cell());
			remove_cell(graph, ctxt.virtual_cell());
		}
		
		ctxt.virtual_cell().reset(new TState);
		ctxt.virtual_cell()->dist_h() = 1;
		ctxt.virtual_cell()->dist_o() = ctxt.current_active_cell()->dist_o();
		
		ctxt.virtual_cell()->dbg().pose_ = dbg_pose;
		
		insert_cell(graph, cells, trans, ctxt.virtual_cell());
		ctxt.virtual_transistion().reset(new TTransformation(ctxt.current_active_cell()));
		insert_transistion(graph, trans, ctxt.virtual_cell(), ctxt.virtual_transistion());

		ctxt.last_active_cell() = ctxt.current_active_cell();
		
		active_cells.push_back(ctxt.virtual_cell());
	}
	
	ctxt.last_dist_min() = ctxt.current_active_cell()->d2();
	
	ROS_ASSERT(ctxt.virtual_cell());
	ROS_ASSERT(ctxt.virtual_transistion());

	ctxt.virtual_transistion()->integrate(odom);
	
	{ //DEBUG
		if(ctxt.virtual_cell()->dbg().info_.find("V")==std::string::npos) ctxt.virtual_cell()->dbg().info_ +="V ";
		if(ctxt.current_active_cell()->dbg().info_.find("C")==std::string::npos) ctxt.current_active_cell()->dbg().info_+="C ";
		
		ctxt.virtual_transistion()->dbg();
	} //DEBUG
	
	
	TIter begin = active_cells.begin();
	TIter end   = active_cells.end();
	
	//step 0: update feature prob.
	for(TIter it=begin; it!=end; it++) {
		if( (*it)->id() >= ctxt.virtual_cell()->id()-ctxt.param().min_age_ )
			continue;
			
		typename TState::TEnergy ft_prob = (*it)->get_feature_prob();/*, ft_prob_max=0;
		typename TState::TEnergy prob_chang = ft_prob-(*it)->get_last_feature_prob(), ft_prob_ch_max=0;
		
		for(TArcIter_out ait((*it)->arc_out_begin(graph)); ait!=(*it)->arc_out_end(graph); ++ait) {
			std::cout<<"xo "<<(*it).get()<<" "<<((*it)->opposite_node(graph, ait)==lemon::INVALID)<<std::endl;
			typename TIter::value_type opposite = cells[(*it)->opposite_node(graph, ait)];
			ft_prob_max = std::max(ft_prob_max, opposite->get_feature_prob());
			ft_prob_ch_max = std::max(ft_prob_ch_max, trans[ait]->directed(*it).transition_factor_dbg(odom, ctxt.param().prox_thr_)*(prob_chang-(opposite->get_feature_prob()-opposite->get_last_feature_prob()))/2);
		}
		for(TArcIter_in ait((*it)->arc_in_begin(graph)); ait!=(*it)->arc_in_end(graph); ++ait) {
			std::cout<<"xi "<<(*it).get()<<" "<<((*it)->opposite_node(graph, ait)==lemon::INVALID)<<std::endl;
			typename TIter::value_type opposite = cells[(*it)->opposite_node(graph, ait)];
			ft_prob_max = std::max(ft_prob_max, opposite->get_feature_prob());
			ft_prob_ch_max = std::max(ft_prob_ch_max, trans[ait]->directed(*it).transition_factor_dbg(odom, ctxt.param().prox_thr_)*(prob_chang-(opposite->get_feature_prob()-opposite->get_last_feature_prob()))/2);
		}
		
		ft_prob = std::max((typename TState::TEnergy)0, ft_prob-ft_prob_max)*odom.dist(ctxt.param().prox_thr_);
	
		if(ft_prob+ft_prob_ch_max)
			DBG_PRINTF("%d: injecting energy %f (feature probability)    %f %f %f    %f %f", (*it)->id(), ft_prob+ft_prob_ch_max, ft_prob, (*it)->get_feature_prob(), ft_prob_max, prob_chang, ft_prob_ch_max);
			
		(*it)->dist_o() *= std::max((typename TState::TEnergy)0, 1-ft_prob-ft_prob_ch_max);*/
		
		ft_prob *= std::min(odom.dist(ctxt.param().prox_thr_), (typename TState::TEnergy)1);	//limit travelled distance to 1 step
		//ft_prob *= std::exp( (*it)->dbg().hops_/3. );
		if(ft_prob)
			DBG_PRINTF("%d: injecting energy %f (feature probability)    %f*%f\n", (*it)->id(), ft_prob, (*it)->get_feature_prob(), odom.dist(ctxt.param().prox_thr_));
			
		(*it)->dist_o() *= std::max((typename TState::TEnergy)0, 1-ft_prob);
	}
	
	//step 1: set min. dist.
	for(TIter it=begin; it!=end; it++) {
		typename TState::TEnergy dh_max = 0;
		int hops = -1;
		
		for(TArcIter_out ait((*it)->arc_out_begin(graph)); ait!=(*it)->arc_out_end(graph); ++ait) {
				typename TIter::value_type opposite = cells[(*it)->opposite_node(graph, ait)];
				if(opposite->dist_h()>0) continue;
				
				if( trans[ait]!=ctxt.virtual_transistion() && 
					opposite->dist_o() < (*it)->dist_o()
					/*std::pow(trans[ait]->dist(ctxt.param().prox_thr_),2) + std::pow(opposite->dist_o(),2)
					<
					(*it)->d2()*/
				) {						
					(*it)->dist_h() = trans[ait]->dist(ctxt.param().prox_thr_);
					(*it)->dist_o() = opposite->dist_o();
					
					hops = std::max(opposite->dbg().hops_, hops);
					
					DBG_PRINTF("1 %d:%d setting dist %f/%f with %d hops\n",
						(*it)->id(), opposite->id(),
						(*it)->dist_h(), (*it)->dist_o(),
						(*it)->dbg().hops_);
				}
		}
		
		/*for(TArcIter_in ait((*it)->arc_in_begin(graph)); ait!=(*it)->arc_in_end(graph); ++ait) {
				typename TIter::value_type opposite = cells[(*it)->opposite_node(graph, ait)];
				if(opposite->dist_h()>0) continue;
				
				if( trans[ait]!=ctxt.virtual_transistion() && 
					opposite->dist_o() < (*it)->dist_o()
					/*std::pow(trans[ait]->dist(ctxt.param().prox_thr_),2) + std::pow(opposite->dist_o(),2)
					<
					(*it)->d2()*
				) {						
					(*it)->dist_h() = trans[ait]->dist(ctxt.param().prox_thr_);
					(*it)->dist_o() = opposite->dist_o();
					
					hops = std::max(opposite->dbg().hops_, hops);
					
					DBG_PRINTF("2 %d:%d setting dist %f/%f with %d hops\n",
						(*it)->id(), opposite->id(),
						(*it)->dist_h(), (*it)->dist_o(),
						(*it)->dbg().hops_);
				}
		}*/
					
		if(hops>=0) (*it)->dbg().hops_ = 1+hops;
	}
	
	//step 2: increase dist.
	for(TIter it=begin; it!=end; it++) {
		
		if( (*it)==ctxt.virtual_cell()) {
			const typename TState::TEnergy du = odom.dist_uncertain(ctxt.param().prox_thr_), dc = odom.dist(ctxt.param().prox_thr_);
			(*it)->dist_h() -=  std::min((*it)->dist_h(), dc);
			(*it)->dist_o() += du-dc;
			
			DBG_PRINTF("%d changing dist V (%f/%f) by (%f/%f)\n", (*it)->id(), (*it)->dist_h(),(*it)->dist_o(), std::min((*it)->dist_h(), dc), du-dc);
		}
		else {
			typename TState::TEnergy dh_max = 0;
			
			for(TArcIter_out ait((*it)->arc_out_begin(graph)); ait!=(*it)->arc_out_end(graph); ++ait)
				dh_max = std::max(dh_max, trans[ait]->directed(*it).transition_factor(odom, ctxt.param().prox_thr_) );
			for(TArcIter_in ait((*it)->arc_in_begin(graph)); ait!=(*it)->arc_in_end(graph); ++ait)
				dh_max = std::max(dh_max, trans[ait]->directed(*it).transition_factor(odom, ctxt.param().prox_thr_) );
			
			const typename TState::TEnergy delta = std::min((*it)->dist_h(), dh_max);
			(*it)->dist_h() -= delta;
			(*it)->dist_o() += std::sqrt(std::pow(odom.dist_uncertain(ctxt.param().prox_thr_),2)-dh_max*dh_max);
		
			DBG_PRINTF("%d changing dist(%f/%f) by (%f/%f) %f %f\n", (*it)->id(), (*it)->dist_h(),(*it)->dist_o(), delta, std::sqrt(std::pow(odom.dist_uncertain(ctxt.param().prox_thr_),2)-dh_max*dh_max), odom.dist(ctxt.param().prox_thr_), std::pow(odom.dist(ctxt.param().prox_thr_),2)-dh_max*dh_max);
			ROS_ASSERT((std::pow(odom.dist_uncertain(ctxt.param().prox_thr_),2)-dh_max*dh_max)>=0);
		}
		
		ROS_ASSERT( (*it)->dist_h()==(*it)->dist_h() );
		ROS_ASSERT( (*it)->dist_o()==(*it)->dist_o() );
	}
	
	
	//debug
	{
		int i=0;
		DBG_PRINTF("distlist");
		for(TIter it=begin; it!=end; it++) {
			DBG_PRINTF("\t %f:%d:%d", (*it)->d(), (*it)->dist_h()<=0, (*it)->id());
			++i;
			if(i>3) break;
		}
		DBG_PRINTF("\n");
	}

}

template<class TCellVector>
void reset_features(TCellVector &active_cells)
{
	typedef typename TCellVector::iterator TIter;
	
	TIter begin = active_cells.begin();
	TIter end   = active_cells.end();
	
	//reset features
	for(TIter it=begin; it!=end; it++) {
		(*it)->reset_feature();
	}
}

