#pragma once

#include <cob_3d_visualization/simple_marker.h>


namespace cob_3d_geometry_map {
	
	namespace Visualization {
		
		class Marker {
		public:
		
			Marker() {
				cob_3d_visualization::RvizMarkerManager::get().clear();
			}
			
			~Marker() {
				cob_3d_visualization::RvizMarkerManager::get().publish();
			}
			
			inline cob_3d_visualization::RvizMarkerManager &manager() {return cob_3d_visualization::RvizMarkerManager::get();}
			
		};

	}

}
