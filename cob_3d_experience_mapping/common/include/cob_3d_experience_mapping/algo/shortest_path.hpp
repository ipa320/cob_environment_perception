//find shortest path between two nodes

template<class TDijkstra, class TNode, class TGraph>
boost::shared_ptr<typename TDijkstra::Path> graph_shortest_path(const TNode &src, const TNode &tgt, TGraph &graph) {
	TDijkstra dijkstra(graph, length);
	//dijkstra.distMap(dist);
	dijsktra.init();
	dijkstra.addSource(src);
	dijkstra.start(tgt);
	
	if(!dijkstra.reached(tgt)) {
		ROS_INFO("could not find a path");
		return boost::shared_ptr<typename TDijkstra::Path>();
	}
	
	ROS_INFO("found path with dist %f", dijkstra.path(tgt).length());
		
	return boost::shared_ptr<TPath>(new TPath(dijkstra.path(tgt)));
}

template<class TCell, class TGraph, class TContext, class TMapCells, class TMapTransformations, class TTransformation>
void find_shortest_path(const TCell &src, const TCell &tgt, TGraph &graph, TContext &ctxt, TMapCells &cells, TMapTransformations &trans, const TTransformation &odom, TResultList &result, const Eigen::Vector3f &dbg_pose)
{
	typedef TPath;
	
	boost::shared_ptr<TPath> path = graph_shortest_path<lemon::Dijkstra>(src->node(), tgt->node(), graph);
	if(!path) return;
}

