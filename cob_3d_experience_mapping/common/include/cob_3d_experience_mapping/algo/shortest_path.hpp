//find shortest path between two nodes

template<class TDijkstra, class TNode, class TGraph, class TLengthMap>
bool graph_shortest_path(boost::shared_ptr<TDijkstra> &dijkstra, const TNode &src, const TNode &tgt, const TGraph &graph, const TLengthMap &length_map) {
	dijkstra.reset(new TDijkstra(graph, length_map));
	//dijkstra.distMap(dist);
	dijkstra->init();
	dijkstra->addSource(src);
	dijkstra->start(tgt);
	
	if(!dijkstra->reached(tgt)) {
		DBG_PRINTF("could not find a path");
		return false;
	}
	
	DBG_PRINTF("found path with dist %f", (float)dijkstra->path(tgt).length());
		
	return true;
}

template<class TAction, class TCell, class TGraph, class TContext, class TMapCells, class TMapTransformations, class TTransformation>
TAction find_next_action(const TCell &src, const TCell &tgt, TGraph &graph, TContext &ctxt, TMapCells &cells, TMapTransformations &trans)
{
	typedef typename lemon::Dijkstra<lemon::Undirector<TGraph> > TSearchAlgo;
	typedef typename TContext::TState TState;
	typedef typename TState::TArcOutIterator TArcIter_out;
	typedef typename TState::TArcInIterator TArcIter_in;
	
	if(src->node()==tgt->node()) {
		DBG_PRINTF("reached as we are already here");
		return TAction::Reached();
	}
	
	boost::shared_ptr<TSearchAlgo> res;
	lemon::Undirector<TGraph> uni_graph(graph);
	typename TSearchAlgo::LengthMap length_map(uni_graph,1);
	if(!graph_shortest_path<TSearchAlgo>(res, src->node(), tgt->node(), uni_graph, length_map ))
		return TAction::None();
	
	typename TGraph::Node next_node, tmp=tgt->node();
	while(tmp!=src->node()) {
		next_node = tmp;
		tmp = res->predNode(tmp);
	}
	
	if(next_node==tgt->node()) {
		DBG_PRINTF("reached");
		return TAction::Reached();
	}
	
	for(TArcIter_out ait(src->arc_out_begin(graph)); ait!=src->arc_out_end(graph); ++ait) {
		if(src->opposite_node(graph, ait)==next_node) {
			DBG_PRINTF("found next action (1)");
			return TAction( trans[ait] );
		}
	}
	for(TArcIter_in ait(src->arc_in_begin(graph)); ait!=src->arc_in_end(graph); ++ait) {
		if(src->opposite_node(graph, ait)==next_node) {
			DBG_PRINTF("found next action (2)");
			return TAction( trans[ait] );
		}
	}
	
	ERROR_PRINTF("run in some error...");
	ERROR_PRINTF("from %d to %d", src->dbg().id_, tgt->dbg().id_);
	if(next_node!=lemon::INVALID)
		ERROR_PRINTF(" got %d",  cells[next_node]->dbg().id_);
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

