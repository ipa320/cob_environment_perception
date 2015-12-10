//3. normalization

template<class TGraph, class TContext>
void normalization(const TGraph &graph, TContext &context)
{
	//TODO:
	//   check if energy_sum is above sqrt(numeric limit<TEnergy>)
	//   if yes then normalize energy
	
	const static typename TContext::TEnergy limit = std::numeric_limits<typename TContext::TEnergy>::max();
	if(context.energy_sum()>limit) {
		std::cout<<"Warning: not implemented normalization would be needed now"<<std::endl;
	}
}
