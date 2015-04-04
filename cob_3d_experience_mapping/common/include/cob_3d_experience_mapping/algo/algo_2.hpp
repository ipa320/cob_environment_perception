//2. inhibition


template<class TIter, class TEnergy, class TEnergyFactor, class TResultList, class TContext>
void inhibition(const TIter &begin, const TIter &end, const TEnergy &energy_max, const TEnergyFactor &inhibition_constant, TResultList &result, TContext &ctxt)
{
	const TEnergyFactor fem = inhibition_constant*energy_max;
	
	// iterate through all states with e>0
	// reduce energy
	for(TIter it=begin; it!=end; it++)
		//if( (*it)!=ctxt.virtual_cell() )
		//TODO: do this for sensor energy only
		;//result.push_back(typename TResultList::value_type(*it, (inhibition_constant*(*it)->energy() - fem)(0)));
}
