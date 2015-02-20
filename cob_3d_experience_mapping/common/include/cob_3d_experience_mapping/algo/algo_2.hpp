//2. inhibition


template<class TIter, class TEnergy, class TEnergyFactor, class TResultList>
void inhibition(const TIter &begin, const TIter &end, const TEnergy &energy_max, const TEnergyFactor &inhibition_constant, TResultList &result)
{
	const TEnergyFactor fem = inhibition_constant*energy_max;
	
	// iterate through all states with e>0
	// reduce energy
	for(TIter it=begin; it!=end; it++)
		result.push_back(typename TResultList::value_type(*it, (inhibition_constant*(*it)->energy() - fem)(0)));
}
