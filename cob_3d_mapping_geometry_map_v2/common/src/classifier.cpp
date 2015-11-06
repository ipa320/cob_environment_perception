#include <cob_3d_mapping_geometry_map_v2/types/classifier.h>
#include <cob_3d_mapping_geometry_map_v2/types/plane.h>

using namespace cob_3d_geometry_map;
using namespace cob_3d_geometry_map::DefaultClassifier;


Class::Ptr Classifier_Floor::classifiy(Object::Ptr obj, ContextPtr ctxt)
{
	Plane *plane = dynamic_cast<Plane*>(obj.get());
	if(plane) {
	
		nuklei::coord_pair dist = floor_params_.distanceTo(plane->pose());
		
		if(dist<std::make_pair(0.,0.))
			return Class::Ptr(new Class_Simple(this));
	}
	
	return Class::Ptr();
}
