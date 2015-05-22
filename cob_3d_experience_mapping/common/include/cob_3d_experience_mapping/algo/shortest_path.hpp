//find shortest path between two nodes

template<class TDijkstra, class TNode, class TGraph>
boost::shared_ptr<typename TDijkstra::Path> graph_shortest_path(const TNode &src, const TNode &tgt, TGraph &graph) {
	TDijkstra dijkstra(graph, typename TDijkstra::LengthMap());
	//dijkstra.distMap(dist);
	dijkstra.init();
	dijkstra.addSource(src);
	dijkstra.start(tgt);
	
	if(!dijkstra.reached(tgt)) {
		ROS_INFO("could not find a path");
		return boost::shared_ptr<typename TDijkstra::Path>();
	}
	
	ROS_INFO("found path with dist %f", dijkstra.path(tgt).length());
		
	return boost::shared_ptr<typename TDijkstra::Path>(new typename TDijkstra::Path(dijkstra.path(tgt)));
}

template<class TAction, class TCell, class TGraph, class TContext, class TMapCells, class TMapTransformations, class TTransformation>
TAction find_next_action(const TCell &src, const TCell &tgt, TGraph &graph, TContext &ctxt, TMapCells &cells, TMapTransformations &trans)
{
	typedef typename lemon::Dijkstra<TGraph>::Path TPath;
	
	boost::shared_ptr<TPath> path = graph_shortest_path<lemon::Dijkstra<TGraph> >(src->node(), tgt->node(), graph);
	if(!path) return TAction::None();
	if(path->length()<=1) return TAction::Reached();
	
	return TAction(trans[path->nthIt(0)]);
	
	/*for(int i=0; i<path->length(); i++) {
		typename TPath::ArcIt it = path->nthIt(i);
	}*/
}

