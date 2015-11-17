//path integration (input from odometry)

template<class TIter, class TEnergyFactor, class TArcIter, class TGraph, class TContext, class TResultList, class TTransformation>
void path_integration(const TIter &begin, const TIter &end, const TEnergyFactor &weight, const TGraph &graph, TContext &ctxt, const TTransformation &odom, TResultList &result)
{
	typedef typename TTransformation::TDist TDist;
	
	//if distance between last cell and new pose is greater than threshold
	// -> generate new cell
	
	ctxt.relative_pose() = TTransformation::integrate_pose(ctxt.relative_pose(), odom);
	if(std::abs(ctxt.relative_pose())>ctxt.param_step_threshold()) {
		add_cell(ctxt.last_cell(), ctxt.relative_pose());
		ctxt.relative_pose() = 0;
	}
	
	// iterate through all states with e>0
	// spread energy (dependent on distance)
	
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
}
