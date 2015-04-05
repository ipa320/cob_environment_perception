//path integration (input from odometry)

template<class TGraph, class TMapCells, class TMapTransformations, typename TState>
void insert_cell(TGraph &graph, TMapCells &cells, TMapTransformations &trans, TState &new_cell) {
	new_cell->set_node(graph.addNode());
	cells.set(new_cell->node(), new_cell);
}

template<class TTransformation, class TGraph, class TMapTransformations, typename TState>
void insert_transistion(TGraph &graph, TMapTransformations &trans, TState &new_cell, TTransformation &link) {
	ROS_ASSERT(link->src());
	trans.set(graph.addArc(new_cell->node(), link->src()->node()), link);
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
	cell->energy() = 1;
	ctxt.active_cells().insert(cell);
	insert_cell(graph, cells, trans, cell);

	//ctxt.virtual_cell().reset(new TState);
	//insert_cell<TTransformation>(graph, cells, trans, ctxt.virtual_cell(), ctxt.current_active_cell());
}

template<class TIter, class TEnergyFactor, class TArcIter, class TGraph, class TContext, class TResultList, class TMapCells, class TMapTransformations, class TTransformation>
void path_integration(const TIter &begin, const TIter &end/*, const TEnergyFactor &weight*/, TGraph &graph, TContext &ctxt, TMapCells &cells, TMapTransformations &trans, const TTransformation &odom, TResultList &result)
{
	typedef typename TContext::TState TState;
	
	ROS_ASSERT(ctxt.active_cells().size()>0);
	ROS_ASSERT(ctxt.current_active_cell());

	if(ctxt.last_active_cell()!=ctxt.current_active_cell() || ctxt.last_energy_max() < ctxt.energy_max()) {
		ROS_INFO("resetting virtual cell");

		remove_cell(graph, ctxt.virtual_cell());
		ctxt.virtual_cell().reset(new TState);
		insert_cell(graph, cells, trans, ctxt.virtual_cell());
		ctxt.virtual_transistion().reset(new TTransformation(ctxt.current_active_cell()));

		ctxt.last_energy_max()  = ctxt.energy_max();
		ctxt.last_active_cell() = ctxt.current_active_cell();
	}
	
	ROS_ASSERT(ctxt.virtual_cell());
	ROS_ASSERT(ctxt.virtual_transistion());

	ctxt.virtual_transistion()->integrate(2*odom);
	ctxt.virtual_cell()->energy() += ctxt.dist2energyfactor(ctxt.virtual_transistion()->scale(ctxt.param().prox_thr_).proximity_pos(odom, ctxt.param().prox_thr_)) * ctxt.current_active_cell()->energy();
//TODO: fix energy trans.
#if 0
	//if distance between last cell and new pose is greater than threshold
	// -> generate new cell
	
	ctxt.relative_pose() = TTransformation::integrate_pose(ctxt.relative_pose(), odom);
	if(std::abs(ctxt.relative_pose())>ctxt.param_step_threshold()) {
		add_cell(ctxt.last_cell(), ctxt.relative_pose());
		ctxt.relative_pose() = 0;
	}

	// iterate through all states with e>0
	// spread energy (dependent on distance)

	typedef typename TTransformation::TDist TDist;
	
	std::vector<TDist> mem_dists;
	for(TIter it=begin; it!=end; it++) {
		TDist sum_dists = 0;
		mem_dists.clear();
		
		for(TArcIter ait(it->edge_begin(graph)); ait!=it->edge_end(graph); ++ait) {			
			typename TIter::value_type opposite = it->opposite_node(ait);
			mem_dists.push_back(ait->directed(*it).proximity(odom));
			sum_dists += mem_dists.back();
		}
		
		typename std::vector<TDist>::const_iterator dit = mem_dists.begin();
		for(TArcIter ait(it->edge_begin(graph)); ait!=it->edge_end(graph); ++ait, dit++) {
			typename TIter::value_type opposite = it->opposite_node(ait);
			result.push_back(opposite, weight* ((*dit)/sum_dists)  * it->energy());
		}
		
	}
#endif

	for(TIter it=begin; it!=end; it++) {

		//transfer energy from here to all connected cells
		for(TArcIter ait((*it)->edge_begin(graph)); ait!=(*it)->edge_end(graph); ++ait) {
			ROS_INFO("energy transistion by odom (pos): %f", trans[ait]->directed(*it).proximity_pos(odom, ctxt.param().prox_thr_) * (*it)->energy());
			typename TIter::value_type opposite = cells[(*it)->opposite_node(graph, ait)];
			result.push_back(typename TResultList::value_type(opposite,
					ctxt.dist2energyfactor(trans[ait]->directed(*it).proximity_pos(odom, ctxt.param().prox_thr_)) * (*it)->energy()
			));
		}

		ROS_INFO("energy transistion by odom (neg): %f", odom.proximity_neg(ctxt.param().prox_thr_) * (*it)->energy());

		//remove energy from this node as it would be ideally moved forward
		result.push_back(typename TResultList::value_type(*it, -ctxt.dist2energyfactor(odom.proximity_neg(ctxt.param().prox_thr_)= * (*it)->energy()));
	}


	ROS_INFO("virtual energy: %f", ctxt.virtual_cell()->energy());

	if(ctxt.current_active_cell()->energy() < ctxt.virtual_cell()->energy()) {
		ROS_INFO("virtual cell is inserted to map");

		insert_transistion(graph, trans, ctxt.virtual_cell(), ctxt.virtual_transistion());
		ctxt.last_active_cell() = ctxt.virtual_cell();
		ctxt.virtual_cell().reset(new TState);
		insert_cell(graph, cells, trans, ctxt.virtual_cell());
	}
}
