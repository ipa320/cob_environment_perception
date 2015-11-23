#include <cob_3d_mapping_geometry_map_v2/types/classifier.h>
#include <cob_3d_mapping_geometry_map_v2/types/plane.h>

using namespace cob_3d_geometry_map;
using namespace cob_3d_geometry_map::DefaultClassifier;


Class::Ptr Classifier_Floor::classifiy(Object::Ptr obj, ContextPtr ctxt)
{
	Plane *plane = dynamic_cast<Plane*>(obj.get());
	if(plane) {		
		//std::cout<<"normal: "<<plane->normal_eigen().transpose()<<std::endl;
		//std::cout<<"offset: "<<plane->offset_eigen().transpose()<<std::endl;
	
		const nuklei::kernel::r3xs2 params = plane->plane_params();
		nuklei::coord_pair dist( std::abs( (floor_params_.loc_-params.loc_).Dot(floor_params_.dir_) ), 1-std::abs(params.dir_.Dot(floor_params_.dir_)) );
		
		//std::cout<<"Classifier_Floor: "<<dist.first<<" "<<dist.second<<std::endl<<std::endl;
		
		if(dist<std::make_pair(floor_params_.loc_h_+params.loc_h_, floor_params_.dir_h_+params.dir_h_))
			return Class::Ptr(new Class_Simple(this));
	}
	
	return Class::Ptr();
}
