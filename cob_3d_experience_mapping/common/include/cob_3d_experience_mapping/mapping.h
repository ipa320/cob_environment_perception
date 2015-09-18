#pragma once

#include "helpers/log.h"
#include "types/serialization.h"
#include "types/general.h"
#include "types/transformation.h"
#include "algo/context.h"
#include <limits>

//! interfaces and implementations of cob_3d_experience_mapping
namespace cob_3d_experience_mapping {
	
	//! algorithms for localization and mapping in cob_3d_experience_mapping
	namespace algorithms {
		
		#include "algo/path_integration.hpp"
		#include "algo/shortest_path.hpp"
		#include "algo/step.hpp"
		
	}
}
