//1. state update

template<class TIter, class TEnergyFactor, class TArcIter, class TGraph, class TMapCells, class TMapTransformations, class TResultList>
void state_update(const TIter &begin, const TIter &end, const TEnergyFactor &weight, const TGraph &graph, const TMapCells &cells, const TMapTransformations &trans, const boost::math::normal distribution[2], TResultList &result)
{
	// iterate through all states with e>0
	// spread energy (dependent on distance)
	for(TIter it=begin; it!=end; it++) {
		
		for(TArcIter ait((*it)->edge_begin(graph)); ait!=(*it)->edge_end(graph); ++ait) {
			typename TIter::value_type opposite = cells[(*it)->opposite_node(graph, ait)];
			result.push_back(typename TResultList::value_type(opposite,
				(weight*trans[ait]->directed(*it).proximity(distribution) * (*it)->energy())(0)
			));
		}
		
	}
}
