
template<class TGraph, class TContext, class TMapCells, class TMapTransformations>
void step(const TGraph &graph, TContext &context, const TMapCells &cells, const TMapTransformations &trans) {
	typedef typename TContext::TState TState;
	typedef std::vector<Result<typename TContext::TState, typename TContext::TEnergy> > TResultList;
	TResultList result;
	
	boost::math::normal distribution[2];
	//TODO:
	
	//step 1
	state_update<typename TContext::TActListIterator, typename TContext::TEnergyFactor, typename TState::TArcIterator, TGraph, TMapCells, TMapTransformations, TResultList>
	(context.active_cells().begin(), context.active_cells().end(), context.param().algo1_energy_weight_, graph, cells, trans, distribution, result);
	context.apply_energy_change(result.begin(), result.end());
	
	//step 2
	inhibition<typename TContext::TActListIterator, typename TContext::TEnergy, typename TContext::TEnergyFactor, TResultList>
	(context.active_cells().begin(), context.active_cells().end(), context.energy_max(), context.param().algo2_inhibition_constant_, result);
	context.apply_energy_change(result.begin(), result.end());
	
	//step 3
	normalization<TGraph, TContext>(graph, context);
}
