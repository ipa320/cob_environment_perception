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
	//only on map data (otherwise pose won't match)
	if(single_shot)
		return Class::Ptr();
		
	obj->rem_class(class_id());
	
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
	//only on map data (otherwise pose won't match)
	if(single_shot)
		return Class::Ptr();
	
	obj->rem_class(class_id());
	
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
	if( //hack
		!(interest_volume_.pose().loc_.Y()>-0.7 || interest_volume_.pose().loc_.Y()<-1.3)
		&&
		!generate_floor_shelf().overlaps(*plane))
		return Class::Ptr();
	
	//2. in box of front  (contains with big margin)
	if(!plane->is_contained(generate_over_shelf()))
		return Class::Ptr();
	
	//3. check pose
	const nuklei::kernel::r3xs2 params = plane->plane_params();
	const nuklei::kernel::r3xs2 box_params = Plane::plane_params(interest_volume_.pose());
	nuklei::coord_pair dist( std::abs( (box_params.loc_-params.loc_).Dot(box_params.dir_) ), 1-std::abs(params.dir_.Dot(box_params.dir_)) );
	
	if(/*dist.first<=box_params.loc_h_+params.loc_h_ && */dist.second<=box_params.dir_h_+params.dir_h_) {
		//4. then add to part of shelf (for width calculation)
		classified_planes_.push_back(Classification(plane));
		
		return Class::Ptr(new Class_Simple(this));
	}
	
	return Class::Ptr();
}

bool Classifier_Carton::find_extrema(const Projector &projector, const CmpSmallestAxis &cmp, Vector2 &ex1, Vector2 &ex2) const {
	bool any = false;
	
	for(size_t i=0; i<classifier_front_->classified_planes_.size(); i++) {
		std::vector<Plane_Polygon::Ptr> res;
		classifier_front_->classified_planes_[i].plane_->project(projector, res);
		for(size_t j=0; j<res.size(); j++) {
			
			Plane_Point::Ptr pt = *std::min_element(res[j]->boundary().begin(), res[j]->boundary().end(), cmp);
			if(!any||cmp(pt,ex1))
				ex1 = pt->pos;
			
			pt = *std::min_element(res[j]->boundary().begin(), res[j]->boundary().end(), cmp);
			if(!any||cmp(pt,ex2))
				ex2 = pt->pos;
				
			any = true;
		}
	}
	
	return any;
}
		
Class::Ptr Classifier_Carton::classifiy_side(Plane *plane, ContextPtr ctxt, const bool single_shot)
{
	assert(classifier_front_);
	
	//disabled side recognition
	return Class::Ptr();
	
	if(!interest_volume_.overlaps(*plane))
		return Class::Ptr();
	
//	std::cout<<"SIDEarea "<<plane->bb_in_pose().sizes()(2)*plane->bb_in_pose().sizes()(1)<<std::endl;
//if(plane->bb_in_pose().sizes()(2)*plane->bb_in_pose().sizes()(1)<0.001) return Class::Ptr();

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
	Projector_Volume projector_bottom(&interest_volume_, Projector_Volume::BOTTOM);
	Vector2 Tf(0,0), Bf, Tb, Bb; //top point of front
	bool any=false;
	
	//3.1 find top point of front
	CmpSmallestAxis cmpY(projector(cast((Eigen::Vector3f)Eigen::Vector3f::Zero()))-projector(cast((Eigen::Vector3f)Eigen::Vector3f::UnitY())));
	for(size_t i=0; i<classifier_front_->classified_planes_.size(); i++) {
		std::vector<Plane_Polygon::Ptr> res;
		classifier_front_->classified_planes_[i].plane_->project(projector, res);
		for(size_t j=0; j<res.size(); j++) {
			Plane_Point::Ptr pt = *std::min_element(res[j]->boundary().begin(), res[j]->boundary().end(), cmpY);
			if(!any||cmpY(pt,Tf))
				Tf = pt->pos;
			any=true;
		}
	}
	//Tf = projector( projector_top(projector_top( projector(Tf) )) );
	
	//3.2 find bottom point of front by projecting top point of front to ground
	Bf = projector( projector_bottom(projector_bottom( projector(Tf) )) );
	
	//3.3 find top point of "back"
	any = false;
	for(size_t j=0; j<plane->polygons().size(); j++) {
		Plane_Point::Ptr pt = *std::min_element(plane->polygons()[j]->boundary().begin(), plane->polygons()[j]->boundary().end(), cmpY);
		if(!any||cmpY(pt,Tb))
			Tb = pt->pos;
		any=true;
	}
	//move point to edge of expected carton by adding depth (equals projection to back)
	Projector_Volume projector_back(&interest_volume_, Projector_Volume::BACK);
	Tb = projector( projector_back(projector_back( projector(Tb) )) );
	
	//3.4 find bottom point of "back" by projecting top point of "back" to ground
	Bb = projector( projector_bottom(projector_bottom( projector(Tb) )) );
	
	//3.5 create rectangle
	Plane_Polygon::Ptr rect(new Plane_Polygon);
	rect->boundary().push_back(new Plane_Point(Bb));
	rect->boundary().push_back(new Plane_Point(Bf));
	rect->boundary().push_back(new Plane_Point(Tf));
	rect->boundary().push_back(new Plane_Point(Tb));
	rect->boundary().push_back(new Plane_Point(Bb));
	
#ifdef DEBUG_
	std::cout<<cmpY.dir_.transpose()<<std::endl;
	std::cout<<"bot back  "<<Bb.transpose()<<std::endl;
	std::cout<<"bot front "<<Bf.transpose()<<std::endl;
	std::cout<<"top front "<<Tf.transpose()<<std::endl;
	std::cout<<"top back  "<<Tb.transpose()<<std::endl;
	
	std::cout<<"bot back  "<<cast(projector(Bb)).transpose()<<std::endl;
	std::cout<<"bot front "<<cast(projector(Bf)).transpose()<<"              "<<cast(projector_bottom(projector_bottom( projector(Tf) ))).transpose()<<std::endl;
	std::cout<<"top front "<<cast(projector(Tf)).transpose()<<std::endl;
	std::cout<<"top back  "<<cast(projector(Tb)).transpose()<<std::endl;
	
	for(int i=0; i<8; i++)
		std::cout<<"edge  "<<(cast(plane->pose())*plane->bb_in_pose().corner((ObjectVolume::TBB::CornerType)i)).transpose()<<std::endl;
#endif

	Plane rect_obj(ctxt, rect, plane->pose());	
	
	static int n=0;
	++n;
	rect_obj.save_as_svg("/tmp/rect"+boost::lexical_cast<std::string>(n)+".svg");
	
	//3.6 union
	rect_obj.merge(*plane, 0);
	
#ifdef DEBUG_
	rect_obj.save_as_svg("/tmp/rect"+boost::lexical_cast<std::string>(n)+"_2.svg");
#endif

	//3.7 model fitting
	double score_coverage, score_matching;
	ctxt->fitModel(rect_obj, score_coverage, score_matching);
	std::cout<<"SIDE "<<n<<"  "<<score_coverage<<" "<<score_matching<<std::endl;
	if(score_coverage<min_coverage_seeing_)
		return Class::Ptr();
	
	//classified_planes_.push_back(Classification(new Plane(rect_obj), 0));
	
#ifdef DEBUG_
	std::cout<<"SIDEarea "<<plane->bb_in_pose().sizes()(2)*plane->bb_in_pose().sizes()(1)<<std::endl;
#endif
	
	//3.8 move by width
	double last_w=0;
	for(size_t i=0; i<widths_.size(); i++) {
		//rect_obj.pose().loc_ += (widths_[i]-last_w)*plane->pose().ori_.Rotate(nuklei_wmf::Vector3<double>::UNIT_X);
		//rect_obj.pose().loc_ += (widths_[i]-last_w)*interest_volume_.pose().ori_.Rotate(nuklei_wmf::Vector3<double>::UNIT_X);
		rect_obj.pose().loc_ += (widths_[i]-last_w)*(nuklei_wmf::Vector3<double>::UNIT_X);
		last_w = widths_[i];
		
		//classified_planes_.push_back(Classification(new Plane(rect_obj), widths_[i]));
		
		ctxt->fitModel(rect_obj, score_coverage, score_matching);
		std::cout<<"SIDE2 "<<score_coverage<<" "<<score_matching<<std::endl;
		if(score_coverage>=min_coverage_expecting_) {
			classified_planes_.push_back(Classification(plane, widths_[i]));
			return Class::Ptr(new Class_Simple(this));
		}
	}
	
	return Class::Ptr();
}

Eigen::Vector3f Classifier_Carton::meanNormal() const {
	Eigen::Vector3f n = Eigen::Vector3f::Zero();
	for(size_t i=0; i<classified_planes_.size(); i++)
		n+=classified_planes_[i].plane_->normal_eigen();
	return n.normalized();
}

Eigen::Vector3f Classifier_Carton::meanNormalFront() const {
	float z = interest_volume_.bb_in_pose().max()(2);
	for(size_t i=0; i<classified_planes_.size(); i++) {
		z = std::min(z, (cast(interest_volume_.pose()).inverse()*cast(classified_planes_[i].plane_->pose())*classified_planes_[i].plane_->bb_in_pose().center())(2));
	}
	
	Eigen::Vector3f n = Eigen::Vector3f::Zero();
	for(size_t i=0; i<classified_planes_.size(); i++) {
		if( std::abs( (cast(interest_volume_.pose()).inverse()*cast(classified_planes_[i].plane_->pose())*classified_planes_[i].plane_->bb_in_pose().center())(2)-z )> 0.02 )
			continue;
		n+=classified_planes_[i].plane_->normal_eigen();
	}
	return n.normalized();
}

/*Eigen::Vector3f Classifier_Carton::mostFrontPt() const {
	Projector_Volume projector(&volume, Projector_Volume::SIDE);
	CmpSmallestAxis cmpZ(projector_front(cast((Eigen::Vector3f)Eigen::Vector3f::UnitZ()))-projector_front(cast((Eigen::Vector3f)Eigen::Vector3f::Zero())));
	
	Eigen::Vector3f pt = Eigen::Vector3f::Zero();
	for(size_t i=0; i<classified_planes_.size(); i++) {
		Plane_Point::Ptr pts = *std::min_element(res[j]->boundary().begin(), res[j]->boundary().end(), cmpZ);
		if(cmpZ(pts,Pleft3.head<2>()))
			pt(2) = std::max(pts->pos(0), volume.bb_in_pose().min()(0));
	}
	return pt;
}*/

void Classifier_Carton::extend(ObjectVolume &vol) const {
	const nuklei::kernel::se3 pose = vol.pose().inverseTransformation();
	for(size_t i=0; i<classified_planes_.size(); i++) {
		for(size_t j=0; j<classified_planes_[i].plane_->polygons().size(); j++)
			for(size_t k=0; k<classified_planes_[i].plane_->polygons()[j]->boundary().size(); k++)
				vol._bb().extend(cast( nuklei::la::transform(pose.loc_, pose.ori_, classified_planes_[i].plane_->to3D(classified_planes_[i].plane_->polygons()[j]->boundary()[k]->pos)) ));
	}
}

std::vector<ObjectVolume> Classifier_Carton::get_cartons() const {
	if(classifier_front_ && classifier_front_->classified_planes_.size()>0) {
		Eigen::Vector3f n_side;
		if(classified_planes_.size()>0)
			n_side = meanNormal();
		else
			n_side = cast(interest_volume_.pose()).matrix().col(0).head<3>();
		Eigen::Vector3f n_front= classifier_front_->meanNormalFront();
		//Eigen::Vector3f pt_front= classifier_front_->mostFrontPt();
		
		//correct box orientation...
		n_front(1) = 0;
		n_front.normalize();
		
		ObjectVolume vol = interest_volume_;
		Eigen::Matrix3f M;
		M.col(1) = n_front.cross(n_side);
		M.col(0) = M.col(1).cross(n_front);
		M.col(2) = M.col(0).cross(M.col(1));
		
		vol.pose().ori_ = cast(Eigen::Quaternionf(M));
		//vol.pose().loc_.Z() = pt_front(2);
		
		if(classified_planes_.size()>0) {
			vol._bb().setEmpty();
			extend(vol);
		}
		classifier_front_->extend(vol);
		
		return get_cartons(vol);
	}
	
	return get_cartons(interest_volume_);
}
	
std::vector<ObjectVolume> Classifier_Carton::get_cartons(const ObjectVolume &volume) const {
	Projector_Volume projector_front(&volume, Projector_Volume::FRONT);
	
	if(classifier_front_) {
		std::vector<ObjectVolume> r = classifier_front_->get_cartons(volume);
		
		//if(classified_planes_.size()<1)
			return r;
			
		if(r.size()==0)
			r.push_back(volume);
		
		std::vector<double> offsets;
		for(size_t i=0; i<classified_planes_.size(); i++) {
			offsets.push_back(projector_front(cast( classified_planes_[i].plane_->local2global(classified_planes_[i].plane_->bb_in_pose().center()) ))(0));
			offsets.push_back(offsets.back()+classified_planes_[i].width_);
		}
			
		std::sort(offsets.begin(), offsets.end());
		
#ifdef DEBUG_
		for(size_t i=0; i<offsets.size(); i++)
			std::cout<<"offset "<<offsets[i]<<std::endl;
		for(size_t i=0; i<widths_.size(); i++)
			std::cout<<"widths_ "<<widths_[i]<<std::endl;
#endif
		
		std::vector<ObjectVolume> r2;
		double min_width = std::abs((widths_.size()>0?widths_[0]:0))-0.02; //+toleracne=2cm
		size_t i=0;
		while(i<offsets.size()) {
			//clustering
			size_t n=i+1;
			while(n<offsets.size() && offsets[i]+min_width>offsets[n]) {
				++n;
			}
			if(n>=offsets.size()) break;
			
#ifdef DEBUG_
			std::cout<<"offset n "<<n<<std::endl;
#endif
			for(size_t j=0; j<r.size(); j++) {
				r2.push_back(r[j]);
				r2.back()._bb().min()(0) = offsets[i];
				r2.back()._bb().max()(0) = offsets[n];
				if(!generate_over_shelf().contains(r2.back()))
					r2.pop_back();
			}
			
			i = n;
		}
		
		return r2;
	}
	
	if(classified_planes_.size()<1)
		return std::vector<ObjectVolume>();
	
	
	Eigen::Vector3f Pleft3 = volume.bb_in_pose().max(), Pright3 = volume.bb_in_pose().min();
	
	Pright3(2) = Pleft3(2);
	for(size_t i=0; i<classified_planes_.size(); i++) {
		Pright3(2) = std::min(Pright3(2), (cast(volume.pose()).inverse()*cast(classified_planes_[i].plane_->pose())*classified_planes_[i].plane_->bb_in_pose().center())(2));
	}
	
	//hack
	if(interest_volume_.pose().loc_.Y()>-0.7)
		Pright3(2) += 0.05;
	
	//find most left and right points of front
	CmpSmallestAxis cmpX(projector_front(cast((Eigen::Vector3f)Eigen::Vector3f::UnitX()))-projector_front(cast((Eigen::Vector3f)Eigen::Vector3f::Zero())));
	for(size_t i=0; i<classified_planes_.size(); i++) {
		std::vector<Plane_Polygon::Ptr> res;
		classified_planes_[i].plane_->project(projector_front, res);

		for(size_t j=0; j<res.size(); j++) {
			Plane_Point::Ptr pts = *std::min_element(res[j]->boundary().begin(), res[j]->boundary().end(), cmpX);
			if(cmpX(pts,Pleft3.head<2>()))
				Pleft3(0) = std::max(pts->pos(0), volume.bb_in_pose().min()(0));
				
			Plane_Point::Ptr ptr = *std::max_element(res[j]->boundary().begin(), res[j]->boundary().end(), cmpX);
			if(!cmpX(ptr,Pright3.head<2>()))
				Pright3(0) = std::min(ptr->pos(0), volume.bb_in_pose().max()(0));
		}
	}
		
#ifdef DEBUG_
	std::cout<<"Pleft3 "<<Pleft3.transpose()<<std::endl;
	std::cout<<"Pright3 "<<Pright3.transpose()<<std::endl;
#endif
		
	ObjectVolume r = volume;
	
	r._bb().setEmpty();
	r._bb().extend(Pleft3);
	r._bb().extend(Pright3);
	
	if( //hack
		interest_volume_.pose().loc_.Y()<-1.3
	)
		return std::vector<ObjectVolume>(1,r);
	
	for(size_t i=0; i<widths_.size(); i++) {
		if( std::abs(std::abs(Pleft3(0)-Pright3(0))-widths_[i]) < 0.09f )
			return std::vector<ObjectVolume>(1,r);
	}
	
	return std::vector<ObjectVolume>();
}

void Classifier_Carton::visualize(ContextPtr ctxt, std::vector<boost::shared_ptr<Visualization::Object> > &objs)
{
	Visualization::Box *box;
	
	box = new Visualization::Box("Classifier::"+name()+"::interest", cast(interest_volume_.pose()), interest_volume_.bb_in_pose());
	box->color() = Eigen::Vector4f(0.5,0.5,0.5,0.15);
	objs.push_back( Visualization::Object::Ptr(box) );
	
	box = new Visualization::Box("Classifier::"+name()+"::interest_shelf", cast(interest_volume_.pose()), generate_shelf().bb_in_pose());
	box->color() = Eigen::Vector4f(0.75,0.25,0.25,0.05);
	objs.push_back( Visualization::Object::Ptr(box) );
	
	box = new Visualization::Box("Classifier::"+name()+"::interest_over_shelf", cast(interest_volume_.pose()), generate_over_shelf().bb_in_pose());
	box->color() = Eigen::Vector4f(0.25,0.75,0.25,0.05);
	objs.push_back( Visualization::Object::Ptr(box) );
	
	box = new Visualization::Box("Classifier::"+name()+"::interest_floor_shelf", cast(interest_volume_.pose()), generate_floor_shelf().bb_in_pose());
	box->color() = Eigen::Vector4f(0.25,0.25,0.75,0.25);
	objs.push_back( Visualization::Object::Ptr(box) );
	
	if(classifier_front_) {
		std::vector<ObjectVolume> boxes = get_cartons();
		for(size_t i=0; i<boxes.size(); i++) {
			box = new Visualization::Box("Classifier::"+name()+"::carton", cast(boxes[i].pose()), boxes[i].bb_in_pose());
			box->color() = Eigen::Vector4f(0,0,0,0.75);
			objs.push_back( Visualization::Object::Ptr(box) );
		}
	}
	
	/*for(size_t i=0; i<classified_planes_.size(); i++) {
		if(!classified_planes_[i].plane_->has_class(class_id())) classified_planes_[i].plane_->add_class(Class::Ptr(new Class_Simple(this)));
		classified_planes_[i].plane_->visualization(objs);
	}*/
}
