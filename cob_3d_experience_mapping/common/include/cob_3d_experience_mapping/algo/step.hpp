
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
	
	DBG_PRINTF("no. cells: %d", (int)context.active_cells().size());
	
	context.clean_active_list();
	DBG_PRINTF("no. cells: %d", (int)context.active_cells().size());
	
	typename TState::TPtr tgt = context.active_cells()[rand()%context.active_cells().size()];
	ActionSearchResult<TTransformation> action = find_next_action<ActionSearchResult<TTransformation>, typename TState::TPtr, TGraph, TContext, TMapCells, TMapTransformations, TTransformation>
	(context.virtual_cell(), tgt, graph, context, cells, trans);
	DBG_PRINTF("test shortest path: %d %d", (int)action.found(), (int)action.reached());
	
	ActionSearchResult<TTransformation> action2 = find_next_action<ActionSearchResult<TTransformation>, typename TState::TPtr, TGraph, TContext, TMapCells, TMapTransformations, TTransformation>
	(tgt, context.virtual_cell(), graph, context, cells, trans);
	DBG_PRINTF("test REVERSE shortest path: %d %d", (int)action2.found(), (int)action2.reached());
	
	cob_3d_experience_mapping::serialization::save_content<boost::archive::binary_oarchive>(ContextContainer<TContext,TGraph,TMapCells,TMapTransformations>(context, graph, cells, trans), "/tmp/exp_mapping.xml.zip", true);
	
	//ContextContainer<TContext,TGraph,TMapCells,TMapTransformations> container(context, graph, cells, trans);
	//restore_content(container, "/tmp/exp_mapping.xml");
}
