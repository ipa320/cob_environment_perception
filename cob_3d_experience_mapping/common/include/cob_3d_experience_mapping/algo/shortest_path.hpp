//find shortest path between two nodes

template<class TDijkstra, class TNode, class TGraph>
bool graph_shortest_path(boost::shared_ptr<TDijkstra> &dijkstra, const TNode &src, const TNode &tgt, TGraph &graph) {
	dijkstra.reset(new TDijkstra(graph, typename TDijkstra::LengthMap(graph,1)));
	//dijkstra.distMap(dist);
	dijkstra->init();
	dijkstra->addSource(src);
	dijkstra->start(tgt);
	
	if(!dijkstra->reached(tgt)) {
		ROS_INFO("could not find a path");
		return false;
	}
	
	ROS_INFO("found path with dist %f", (float)dijkstra->path(tgt).length());
		
	return true;
}

template<class TAction, class TCell, class TGraph, class TContext, class TMapCells, class TMapTransformations, class TTransformation>
TAction find_next_action(const TCell &src, const TCell &tgt, TGraph &graph, TContext &ctxt, TMapCells &cells, TMapTransformations &trans)
{
	typedef typename lemon::Dijkstra<TGraph> TSearchAlgo;
	typedef typename TContext::TState TState;
	typedef typename TState::TArcOutIterator TArcIter_out;
	
	boost::shared_ptr<TSearchAlgo> res;
	if(!graph_shortest_path<TSearchAlgo>(res, src->node(), tgt->node(), graph))
		return TAction::None();
	
	typename TGraph::Node next_node, tmp=tgt->node();
	while(tmp!=src->node()) {
		next_node = tmp;
		tmp = res->predNode(tmp);
	}
	
	if(next_node==tgt->node()) {
		ROS_INFO("reached");
		return TAction::Reached();
	}
	
	for(TArcIter_out ait(src->arc_out_begin(graph)); ait!=src->arc_out_end(graph); ++ait) {
		if(ctxt.current_active_cell()->opposite_node(graph, ait)==next_node) {
			ROS_INFO("found next action");
			return TAction( trans[ait] );
		}
	}
	
	ROS_ERROR("run in some error...");
	return TAction::None();
	
	/*for(int i=0; i<path->length(); i++) {
		typename TPath::ArcIt it = path->nthIt(i);
	}
	
	
    for (Node v=t;v != s; v=dijkstra_test.predNode(v)) {
      std::cout << g.id(v) << "<-";
    }
    
    
	* 
	* */
}

