#pragma once

namespace cob_3d_experience_mapping {
	
	template<class TEnergyFactor>
	struct Parameter {
		//typedef float TScalar;
		
		TEnergyFactor algo1_energy_weight_;
		TEnergyFactor algo2_inhibition_constant_;
		
		Parameter() {
			//TODO: read from parameter server
		}
		
	};
	
}
