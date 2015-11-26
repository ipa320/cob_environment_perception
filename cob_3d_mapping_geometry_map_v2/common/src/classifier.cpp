#include <cob_3d_mapping_geometry_map_v2/types/classifier.h>
#include <cob_3d_mapping_geometry_map_v2/types/context.h>
#include <cob_3d_mapping_geometry_map_v2/types/plane.h>
#include <cob_3d_mapping_geometry_map_v2/visualization/vis_obj.h>

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

void Classifier_Floor::visualize(ContextPtr ctxt, std::vector<boost::shared_ptr<Visualization::Object> > &objs)
{
	Visualization::Mesh::Ptr mesh( new Visualization::Mesh("Classifier::Floor") );
	
	const Eigen::Vector3f z = Eigen::Vector3f::UnitX().cross(cast(floor_params_.dir_))*10;
	const Eigen::Vector3f x = Eigen::Vector3f::UnitZ().cross(cast(floor_params_.dir_))*10;
	const Eigen::Vector3f o = cast(floor_params_.loc_);
	mesh->add(
		(o-x-z), (o-x+z), (o+x+z)
	);
	mesh->add(
		(o+x+z), (o+x-z), (o-x-z)
	);
	
	mesh->color()(3) = 0.5;
	objs.push_back( mesh );
}

Class::Ptr Classifier_Carton::classifiy(Object::Ptr obj, ContextPtr ctxt, const bool single_shot)
{
	if(obj->has_class(CLASSIFIER_FLOOR) || obj->has_class(CLASSIFIER_WALL))
		return Class::Ptr();
	
	Plane *plane = dynamic_cast<Plane*>(obj.get());
	if(plane) {
		if(classifier_front_)
			return classifiy_side(plane, ctxt, single_shot);
		else
			return classifiy_front(plane, ctxt, single_shot);
	}
	
	return Class::Ptr();
}

Class::Ptr Classifier_Carton::classifiy_front(Plane *plane, ContextPtr ctxt, const bool single_shot)
{
	//1. contact to shelf panel? (intersection)
	if(!generate_shelf().overlaps(*plane))
		return Class::Ptr();
	
	//2. in box of front  (contains with big margin)
	if(!generate_over_shelf().contains(*plane))
		return Class::Ptr();
	
	//3. check pose
	const nuklei::kernel::r3xs2 params = plane->plane_params();
	const nuklei::kernel::r3xs2 box_params = Plane::plane_params(interest_volume_.pose());
	nuklei::coord_pair dist( std::abs( (box_params.loc_-params.loc_).Dot(box_params.dir_) ), 1-std::abs(params.dir_.Dot(box_params.dir_)) );
	
	if(dist.first<=box_params.loc_h_+params.loc_h_ && dist.second<=box_params.dir_h_+params.dir_h_) {
		//4. then add to part of shelf (for width calculation)
		classified_planes_.push_back(Classification(plane));
		
		return Class::Ptr(new Class_Simple(this));
	}
	
	return Class::Ptr();
}

bool CmpSmallestY(const Plane_Point::Ptr &a, const Plane_Point::Ptr &b) {
	return a->pos(1)<b->pos(1);
}
		
Class::Ptr Classifier_Carton::classifiy_side(Plane *plane, ContextPtr ctxt, const bool single_shot)
{
	assert(classifier_front_);
	
	if(!interest_volume_.overlaps(*plane))
		return Class::Ptr();
	
	//1. check pose
	const nuklei::kernel::r3xs2 params = plane->plane_params();

	nuklei::kernel::r3xs2 box_params;
	box_params.loc_ = interest_volume_.pose().loc_;
	box_params.dir_ = interest_volume_.pose().ori_.Rotate(nuklei_wmf::Vector3<double>::UNIT_X);
	box_params.loc_h_ = interest_volume_.pose().loc_h_;
	box_params.dir_h_ = interest_volume_.pose().ori_h_;
		
	nuklei::coord_pair dist( std::abs( (box_params.loc_-params.loc_).Dot(box_params.dir_) ), 1-std::abs(params.dir_.Dot(box_params.dir_)) );
	
	if(dist.first>box_params.loc_h_+params.loc_h_ || dist.second>box_params.dir_h_+params.dir_h_)
		return Class::Ptr();
		
	//2. within region of interest
	if(!generate_over_shelf().contains(*plane))
		return Class::Ptr();
		
	//3. generate model
	Projector_Plane projector(plane);
	Vector2 Tf(0,0), Bf, Tb, Bb; //top point of front
	
	//3.1 find top point of front
	for(size_t i=0; i<classifier_front_->classified_planes_.size(); i++) {
		std::vector<Plane_Polygon::Ptr> res;
		classifier_front_->classified_planes_[i].plane_->project(projector, res);
		for(size_t j=0; j<res.size(); j++) {
			Plane_Point::Ptr pt = *std::max_element(res[j]->boundary().begin(), res[j]->boundary().end(), CmpSmallestY);
			if(pt->pos(1)<Tf(1))
				Tf = pt->pos;
		}
	}
	
	//3.2 find bottom point of front by projecting top point of front to ground
	Bf = projector( nuklei::la::transform(interest_volume_.pose().loc_, interest_volume_.pose().ori_, cast(interest_volume_.bb_in_pose().min()) ) );
	Bf(0) = Tf(0);
	
	//3.3 find top point of "back"
	{
		Eigen::Vector3f v = plane->bb_in_pose().max();
		v(1) = plane->bb_in_pose().min()(1);
		Tb = projector( nuklei::la::transform(plane->pose().loc_, plane->pose().ori_, cast(v)) );
	}
	
	//3.4 find bottom point of "back" by projecting top point of "back" to ground
	Bb = projector( nuklei::la::transform(interest_volume_.pose().loc_, interest_volume_.pose().ori_, cast((Eigen::Vector3f)(interest_volume_.bb_in_pose().min()+Eigen::Vector3f(0,0,interest_volume_.bb_in_pose().sizes()(2)) )) ) );
	Bb = (Bb-Bf).dot(Tb-Bf)/(Bb-Bf).squaredNorm()*(Bb-Bf) + Bf;
	
	//3.5 create rectangle
	Plane_Polygon::Ptr rect(new Plane_Polygon);
	rect->boundary().push_back(new Plane_Point(Bb));
	rect->boundary().push_back(new Plane_Point(Bf));
	rect->boundary().push_back(new Plane_Point(Tf));
	rect->boundary().push_back(new Plane_Point(Tb));
	rect->boundary().push_back(new Plane_Point(Bb));
	
	Plane rect_obj(ctxt, rect, plane->pose());	
	
	rect_obj.save_as_svg("/tmp/rect.svg");
	
	//3.6 union
	rect_obj.merge(*plane, 0);
	
	//3.7 model fitting
	double score_coverage, score_matching;
	ctxt->fitModel(rect_obj, score_coverage, score_matching);
	std::cout<<"SIDE "<<score_coverage<<" "<<score_matching<<std::endl;
	if(score_coverage<0.8)
		return Class::Ptr();
	
	//3.8 move by width
	double last_w=0;
	for(size_t i=0; i<widths_.size(); i++) {
		rect_obj.pose().loc_ += (widths_[i]-last_w)*interest_volume_.pose().ori_.Rotate(nuklei_wmf::Vector3<double>::UNIT_X);
		last_w = widths_[i];
		
		ctxt->fitModel(rect_obj, score_coverage, score_matching);
		if(score_coverage>=0.8) {
			classified_planes_.push_back(Classification(plane, widths_[i]));
			return Class::Ptr(new Class_Simple(this));
		}
	}
	
	return Class::Ptr();
}

void Classifier_Carton::visualize(ContextPtr ctxt, std::vector<boost::shared_ptr<Visualization::Object> > &objs)
{
	Visualization::Box *box;
	
	box = new Visualization::Box("Classifier::"+name()+"::interest", cast(interest_volume_.pose()), interest_volume_.bb_in_pose());
	box->color() = Eigen::Vector4f(0.5,0.5,0.5,0.5);
	objs.push_back( Visualization::Object::Ptr(box) );
	
	box = new Visualization::Box("Classifier::"+name()+"::interest_shelf", cast(interest_volume_.pose()), generate_shelf().bb_in_pose());
	box->color() = Eigen::Vector4f(0.75,0.25,0.25,0.5);
	objs.push_back( Visualization::Object::Ptr(box) );
	
	box = new Visualization::Box("Classifier::"+name()+"::interest_over_shelf", cast(interest_volume_.pose()), generate_over_shelf().bb_in_pose());
	box->color() = Eigen::Vector4f(0.25,0.75,0.25,0.5);
	objs.push_back( Visualization::Object::Ptr(box) );
	
	
}
