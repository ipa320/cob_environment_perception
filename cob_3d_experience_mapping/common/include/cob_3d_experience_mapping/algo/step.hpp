
template<class TGraph, class TContext, class TMapCells, class TMapTransformations, class TTransformation>
void step(TGraph &graph, TContext &context, TMapCells &cells, TMapTransformations &trans, const TTransformation &odom, const Eigen::Vector3f &dbg_pose) {
	typedef typename TContext::TState TState;
	typedef std::vector<Result<typename TContext::TState, typename TContext::TEnergy> > TResultList;
	TResultList result;

	//TODO:
	//if empty or max. energy<=0 -> init (creates new map segment)

	typedef std::vector<typename TState::TPtr> TCellVector;
	/*context.active_cells().clear();
	for(typename TGraph::NodeIt it(graph); it!=lemon::INVALID; ++it) {
		context.active_cells().push_back(cells[it]);
		cells[it]->dbg().info_="";
	}*/

	path_integration<
		TCellVector, typename TContext::TEnergyFactor,
		TGraph, TContext, TResultList, TMapCells, TMapTransformations, TTransformation
	>(context.active_cells(), graph, context, cells, trans, odom, result, dbg_pose);
	
	ROS_INFO("no. cells: %d", (int)context.active_cells().size());
	
	context.clean_active_list();
	ROS_INFO("no. cells: %d", (int)context.active_cells().size());
	
	ActionSearchResult<TTransformation> action = find_next_action<ActionSearchResult<TTransformation>, typename TState::TPtr, TGraph, TContext, TMapCells, TMapTransformations, TTransformation>
	(context.virtual_cell(), context.current_active_cell(), graph, context, cells, trans);
	ROS_INFO("test shortest path: %d %d", (int)action.found(), (int)action.reached());
	
	save_schedule(ContextContainer<TContext,TGraph,TMapCells,TMapTransformations>(context, graph, cells, trans), "/tmp/exp_mapping.xml");
}
