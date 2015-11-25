#include <cob_3d_mapping_geometry_map_v2/types/classifier.h>
#include <cob_3d_mapping_geometry_map_v2/types/plane.h>

using namespace cob_3d_geometry_map;
using namespace cob_3d_geometry_map::DefaultClassifier;
using namespace cob_3d_geometry_map::CustomClassifier;


Vector2 Classifier_Floor::operator()(const nuklei_wmf::Vector3<double> &pt3) const {
	Eigen::Matrix3f M;
	M.col(2) = cast(floor_params_.dir_);
	M.col(1) = Eigen::Vector3f::UnitX().cross(M.col(2));
	M.col(0) = M.col(2).cross(M.col(1));
	
	return (M*(cast(pt3)-cast(floor_params_.loc_))).head<2>();
}

Class::Ptr Classifier_Floor::classifiy(Object::Ptr obj, ContextPtr ctxt, const bool single_shot)
{
	Plane *plane = dynamic_cast<Plane*>(obj.get());
	if(plane) {		
		//std::cout<<"normal: "<<plane->normal_eigen().transpose()<<std::endl;
		//std::cout<<"offset: "<<plane->offset_eigen().transpose()<<std::endl;
	
		const nuklei::kernel::r3xs2 params = plane->plane_params();
		nuklei::coord_pair dist( std::abs( (floor_params_.loc_-params.loc_).Dot(floor_params_.dir_) ), 1-std::abs(params.dir_.Dot(floor_params_.dir_)) );
		
		//std::cout<<"Classifier_Floor: "<<dist.first<<" "<<dist.second<<std::endl<<std::endl;
		
		if(dist.first<=floor_params_.loc_h_+params.loc_h_ && dist.second<=floor_params_.dir_h_+params.dir_h_) {
			std::cout<<"Classifier_Floor: dist "<<dist.first<<" "<<dist.second<<std::endl<<std::endl;
			std::cout<<"Classifier_Floor: band "<<params.loc_h_<<" "<<params.dir_h_<<std::endl<<std::endl;
			return Class::Ptr(new Class_Simple(this));
		}
		
		if(!single_shot && rays_.size()>0) {
			std::vector<Plane_Polygon::Ptr> polys;
			plane->project(*this, polys);
			
			std::vector<Plane_Point::Ptr> pts(2);
			pts[0].reset(new Plane_Point(0,0));
			pts[1].reset(new Plane_Point(0,0));
			
			for(size_t i=0; i<polys.size(); i++) {
				for(size_t j=0; j<rays_.size(); j++) {
					pts[1]->pos(0) = max_range()*std::cos(j*2*M_PI/rays_.size());
					pts[1]->pos(1) = max_range()*std::sin(j*2*M_PI/rays_.size());
					
					std::vector<Plane_Point::Ptr> result;
					typedef boost::geometry::model::linestring<Plane_Point::Ptr> Linestring;
					Linestring ls(pts.begin(), pts.end());
					boost::geometry::intersection(ls,polys[i]->boundary(),result);
					
					for(size_t k=0; k<result.size(); k++)
						rays_[j] = std::min(std::isinf(rays_[j])?5.:rays_[j], (double)result[k]->pos.squaredNorm());
					
				}
			}
		}
		
	}
	
	return Class::Ptr();
}

Class::Ptr Classifier_Carton::classifiy(Object::Ptr obj, ContextPtr ctxt, const bool single_shot)
{
	if(obj->has_class(CLASSIFIER_FLOOR) || obj->has_class(CLASSIFIER_WALL))
		return Class::Ptr();
	
	Plane *plane = dynamic_cast<Plane*>(obj.get());
	if(plane) {
		//1. contact to shelf panel? (intersection)
		ObjectVolume shelf = interest_volume_;
		{
			Eigen::Vector3f vi=interest_volume_.bb_in_pose().min(), va=interest_volume_.bb_in_pose().max();
			va(1) = vi(1)+0.01; //1cm
			vi(1) -= 0.03; 	//3cm
			shelf._bb() = ObjectVolume::TBB(vi, va);
		}
		if(!shelf.overlaps(*plane))
			return Class::Ptr();
		
		//2. in box of front  (contains with big margin)
		ObjectVolume over_shelf = interest_volume_;
		shelf._bb().min()(0) -= 0.2; //20cm
		shelf._bb().max()(0) += 0.2; //20cm
		if(!shelf.contains(*plane))
			return Class::Ptr();
		
		//3. check pose
		const nuklei::kernel::r3xs2 params = plane->plane_params();
		const nuklei::kernel::r3xs2 box_params = Plane::plane_params(interest_volume_.pose());
		nuklei::coord_pair dist( std::abs( (box_params.loc_-params.loc_).Dot(box_params.dir_) ), 1-std::abs(params.dir_.Dot(box_params.dir_)) );
		
		
		if(dist.first<=box_params.loc_h_+params.loc_h_ && dist.second<=box_params.dir_h_+params.dir_h_) {
		
			//4. then add to part of shelf (for width calculation)
			return Class::Ptr(new Class_Simple(this));			
		}
		
	}
	
	return Class::Ptr();
}

