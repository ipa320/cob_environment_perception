#include <cob_3d_mapping_geometry_map_v2/types/context.h>
#include <cob_3d_mapping_geometry_map_v2/types/impl/bvh_query.hpp>
#include <iostream>
#include <boost/make_shared.hpp>

#include <cob_3d_mapping_geometry_map_v2/types/plane.h>
#include <cob_3d_mapping_geometry_map_v2/types/impl/cutter.hpp>

#include <cob_3d_mapping_geometry_map_v2/visualization/marker.h>
#include <cob_3d_mapping_geometry_map_v2/visualization/vis_obj.h>

using namespace cob_3d_geometry_map;


GlobalContext::GlobalContext() : scene_(new Context) {
}

void GlobalContext::add_scene(const cob_3d_mapping_msgs::PlaneScene &scene, TransformationEstimator * const tf_est)
{
	Context::Ptr ctxt = boost::make_shared<Context>();
	ctxt->add_scene(ctxt, scene);
	classify(ctxt, true);
	ctxt->optimize(ctxt);
	
	assert(scene_);
	
	nuklei::kernel::se3 tf;
	if(tf_est)
		if(!tf_est->register_scene(ctxt, scene_, tf)) {
			ROS_WARN("failed to register scene --> SKIPPING!");
			return;
		}
	
	scene_2d_.reset(new Context2D(scene_, proj_.cast<float>(), tf));
	scene_->set_context_2d(scene_2d_.get());
	scene_2d_->merge(ctxt);
	
#ifdef DEBUG_
	std::cout<<"scene_ "<<scene_->end()-scene_->begin()<<std::endl;
#endif
	
	scene_->build();
	classify(scene_, false);
	scene_->optimize(scene_);
}

bool GlobalContext::registerClassifier(Classifier *c) {
	assert(c);
	
	for(ClassifierSet::const_iterator it=classifiers_.begin(); it!=classifiers_.end(); it++)
		if((*it)->class_id()==c->class_id() || (*it)->name()==c->name()) {
			ROS_WARN("classsifier %s (%d) was already registered under %s (%d)", 
				c->name().c_str(), c->class_id(),  (*it)->name().c_str(), (*it)->class_id());
			return false;
		}
		
	classifiers_.push_back(Classifier::Ptr(c));
}

void GlobalContext::classify(const Context::Ptr &ctxt, const bool single_shot)
{
	for(ClassifierSet::iterator it=classifiers_.begin(); it!=classifiers_.end(); it++) {
		(*it)->start(ctxt);
		ctxt->classify(ctxt, *it, single_shot);
		(*it)->end(ctxt);
	}
}

void GlobalContext::visualize_markers() {
	if(!cob_3d_visualization::RvizMarkerManager::get().needed()) {
		ROS_DEBUG("skiped marker visualization as no subscribers are present");
		return;
	}
		
	std::vector<boost::shared_ptr<Visualization::Object> > vis_objs;
	scene_->visualize(vis_objs);
	for(ClassifierSet::iterator it=classifiers_.begin(); it!=classifiers_.end(); it++)
		(*it)->visualize(scene_, vis_objs);
	
	Visualization::Marker stream;
	for(size_t i=0; i<vis_objs.size(); i++)
		vis_objs[i]->serialize(stream);
}

void Context::add_scene(const Context::Ptr &this_ctxt, const cob_3d_mapping_msgs::PlaneScene &scene)
{
	//TODO: add image first
	
	for(size_t i=0; i<scene.planes.size(); i++) {
		Plane *plane = new Plane(this_ctxt, scene.planes[i]);
		if(!plane->simplify_by_area(0.01*0.01))
			delete plane;
		else
			add(Object::Ptr( plane ));
	}
	
	build();
}

namespace Eigen {
     namespace internal {
         Eigen::AlignedBox<float,3> bounding_box(const Context::BVH_Point::Object &v) { return Eigen::AlignedBox<float,3>(v.pos_); }
     }
}

void Context::build()
{
	std::vector<typename BVH_Volume::Object> boxes;
	std::vector<typename BVH_Point::Object> points;
	
	for(size_t i=0; i<objs_.size(); i++) {
		ObjectVolume *volume = dynamic_cast<ObjectVolume*>(objs_[i].get());
		if(volume) {
			boxes.push_back(typename BVH_Volume::Object(objs_[i], volume->bb_in_context()));
		}
		
		Object3D *point = dynamic_cast<ObjectVolume*>(objs_[i].get());
		if(point) {
			points.push_back(typename BVH_Point::Object(objs_[i], cast(point->pose().loc_)));
		}
		
	}
	
	search_volume_.init(boxes.begin(),  boxes.end());
	search_point_ .init(points.begin(), points.end());
}

void Context::classify(const Context::Ptr &ctxt, const Classifier::Ptr &classifier, const bool single_shot)
{
	assert(ctxt.get()==this);
	
	const bool once = classifier->once();
	for(size_t i=0; i<objs_.size(); i++) {
		if(once && objs_[i]->has_class(classifier->class_id())) continue;
		
		Class::Ptr cl = classifier->classifiy(objs_[i], ctxt, single_shot);
		
		if(cl) {
			objs_[i]->add_class(cl);
		}
	}
}

void Context::add(const Object::Ptr &obj)
{
	objs_.push_back(obj);
}

void Context::visualize(std::vector<boost::shared_ptr<Visualization::Object> > &vis_objs) {
	for(size_t i=0; i<objs_.size(); i++)
		objs_[i]->visualization(vis_objs);
}

template<class TSearchResult, class BVH>
struct VolumeIntersection
{
	const Eigen::AlignedBox<Context::Scalar, Context::Dim> &bb_;
	TSearchResult &search_result_;
	VolumeIntersection(const Eigen::AlignedBox<Context::Scalar, Context::Dim> &bb, TSearchResult &search_result) : bb_(bb), search_result_(search_result) {}
  
	bool intersectVolume(const typename BVH::Volume &volume) {
		return !bb_.intersection(volume).isEmpty();
	}
	bool intersectObject(const typename BVH::Object &object) {
		return !bb_.intersection(Eigen::internal::bounding_box(object)).isEmpty();
	}
	
	bool add(const typename BVH::Object &object) {
		search_result_.push_back(object.obj_);
		return false; //go on
	}
};

void Context::volume_search_in_volume(const Eigen::AlignedBox<Scalar, Dim> &bb, std::vector<Object::Ptr> &result) {
	VolumeIntersection<std::vector<Object::Ptr>, BVH_Volume> intersector(bb, result);
	Eigen::BVIntersect2(search_volume_, intersector);
}

void Context::volume_search_in_points(const Eigen::AlignedBox<Scalar, Dim> &bb, std::vector<Object::Ptr> &result) {
	VolumeIntersection<std::vector<Object::Ptr>, BVH_Point> intersector(bb, result);
	Eigen::BVIntersect2(search_point_, intersector);
}

void Context::intersects(const Intersection_Volume &search_param, std::vector<Object::Ptr> &result) {
	std::vector<Object::Ptr> tmp;
	volume_search_in_volume(search_param.bb(), tmp);
	volume_search_in_points(search_param.bb(), tmp);
	
#ifdef DEBUG_
	std::cout<<"Context::intersects stage1 "<<tmp.size()<<" from "<<objs_.size()<<std::endl;
#endif
	
	for(size_t i=0; i<tmp.size(); i++)
		if(search_param.intersects(tmp[i].get())) {
			bool found=false;
			for(size_t j=0; j<result.size(); j++)
				if(result[j]==tmp[i]) {
					found=true;
					break;
				}
			if(!found)
				result.push_back(tmp[i]);
		}
}

#define CAST_TO(Classname, Input, Name) \
{\
Classname *Name = dynamic_cast<Classname*>((Input).get());\
if(Name) {

#define CAST_TO_END }}
	
void Context::optimize(const Context::Ptr &ctxt) {
	//snapping
	for(size_t i=0; i<objs_.size(); i++) {
		
		CAST_TO(Plane, objs_[i], obj);
			std::vector<Object::Ptr> result;
			volume_search_in_volume(obj->bb_in_context(), result);
			
#ifdef DEBUG_
			std::cout<<"snapping: "<<result.size()<<std::endl;
#endif
			for(size_t j=0; j<result.size(); j++) {
				if(result[j]==objs_[i]) continue;
				CAST_TO(Plane, result[j], other_obj);
					if(obj->snap(*other_obj)) {
						objs_.erase(objs_.begin()+i);
						--i;
						break;
					}
				CAST_TO_END;
			}
			
		CAST_TO_END;
		
	}
	
	build();
}

Context2D::Context2D(const Context::Ptr &ctxt, const Eigen::Matrix3f &proj, const nuklei::kernel::se3 &tf, const double fc):
	scene_(ctxt), tf_(tf)
{
	rtree_.clear();
	
	Eigen::Matrix4f proj4 = Eigen::Matrix4f::Identity();
	proj4.topLeftCorner<3,3>() = proj;
	projector_.setT(proj4*cast(tf).inverse().matrix());
	
	std::vector<Object::Ptr> result;
	ctxt->intersects(Intersection_Volume_Viewport(
		cast(tf),
		proj, fc
	), result);
	
#ifdef DEBUG_
	std::cout<<"Context2D: "<<result.size()<<" from "<<(ctxt->end()-ctxt->begin())<<std::endl;
#endif
	for(size_t i=0; i<result.size(); i++) {
		insert(result[i]);
	
#ifdef DEBUG_
		save_as_svg("/tmp/Context2D.svg");
#endif
	}
	
#ifdef DEBUG_
	static int n=0;
	char buf[256];
	sprintf(buf, "/tmp/Context2D_%d.svg", n++);
	save_as_svg(buf);
#endif
}

void Context2D::insert(Object::Ptr obj, const std::vector<Plane_Polygon> &ps) {
	//limit to viewport
	Plane_Polygon cut_viewpoint;
	
	cut_viewpoint.boundary().push_back(new Plane_Point(0,0));
	cut_viewpoint.boundary().push_back(new Plane_Point(0,1));
	cut_viewpoint.boundary().push_back(new Plane_Point(1,1));
	cut_viewpoint.boundary().push_back(new Plane_Point(1,0));
	cut_viewpoint.boundary().push_back(new Plane_Point(0,0));
	
	for(size_t i=0; i<ps.size(); i++)
		_insert(obj, ps[i]&cut_viewpoint);
}

void Context2D::insert(Object::Ptr obj, const Plane_Polygon &p) {
	//limit to viewport
	Plane_Polygon cut_viewpoint;
	
	cut_viewpoint.boundary().push_back(new Plane_Point(0,0));
	cut_viewpoint.boundary().push_back(new Plane_Point(0,1));
	cut_viewpoint.boundary().push_back(new Plane_Point(1,1));
	cut_viewpoint.boundary().push_back(new Plane_Point(1,0));
	cut_viewpoint.boundary().push_back(new Plane_Point(0,0));
	
	_insert(obj, p&cut_viewpoint);
}
	
void Context2D::_insert(Object::Ptr obj, const std::vector<Plane_Polygon> &ps) {
#ifdef DEBUG_
	std::cout<<"Context2D: _insert "<<ps.size()<<std::endl;
#endif
	
	for(size_t i=0; i<ps.size(); i++) {
		//3. insert to search tree
		ProjectedPolygon::Ptr pp(new ProjectedPolygon);
		pp->obj_ = obj;
		pp->poly_.reset(new Plane_Polygon(ps[i]));
		
		//boost::geometry::simplify(ps[i], *pp->poly_, 0.05);
		boost::geometry::correct(*pp->poly_);
		
#ifdef DEBUG_
		std::cout<<"Context2D: boundary "<<pp->poly_->boundary().size()<<std::endl;
#endif
		if(pp->poly_->boundary().size()==0 || !boost::geometry::is_valid(*pp->poly_) ) continue;
		
#ifdef DEBUG_
		pp->poly_->save_as_svg("/tmp/probe.svg");
#endif

		rtree_.insert(std::make_pair(pp->get_box(), pp));
#ifdef DEBUG_
		std::cout<<"Context2D: rtree_ "<<rtree_.size()<<std::endl;
#endif
	}
}

void Context2D::save_as_svg(const std::string &fn) const
{
    std::ofstream svg(fn.c_str());
    boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);
    
	Plane_Polygon cut_viewpoint;
	
	cut_viewpoint.boundary().push_back(new Plane_Point(0,0));
	cut_viewpoint.boundary().push_back(new Plane_Point(0,1));
	cut_viewpoint.boundary().push_back(new Plane_Point(1,1));
	cut_viewpoint.boundary().push_back(new Plane_Point(1,0));
	cut_viewpoint.boundary().push_back(new Plane_Point(0,0));
	
	cut_viewpoint.save_as_svg(mapper);
    
    for(RTREE::const_iterator it=rtree_.begin(); it!=rtree_.end(); it++)
		it->second->poly_->save_as_svg(mapper);
}

void Context2D::insert(Object::Ptr _obj)
{
	CAST_TO(Plane, _obj, obj);
	
	//1. project polygon to view plane
	std::vector<Plane_Polygon::Ptr> polys;
	obj->project(projector_, polys);
	
#ifdef DEBUG_
	std::cout<<"Context2D: polys "<<polys.size()<<std::endl;
#endif

	for(size_t j=0; j<polys.size(); j++) {
	
		//2. check for intersections with current view --> only take front
		box b = ProjectedPolygon::get_box(polys[j]);
		std::vector<value> result_n;
		rtree_.query(boost::geometry::index::intersects(b), std::back_inserter(result_n));
		
#ifdef DEBUG_
		std::cout<<"Context2D: inter "<<result_n.size()<<std::endl;
#endif

		boost::shared_ptr<std::vector<Plane_Polygon> > p_new(new std::vector<Plane_Polygon>), p_new_sw(new std::vector<Plane_Polygon>);
		p_new->push_back(*polys[j]);
		
		for(size_t i=0; i<result_n.size(); i++) {
			CAST_TO(Plane, result_n[i].second->obj_, matched_plane);
				if(!boost::geometry::overlaps(*result_n[i].second->poly_,*polys[j])) continue;
			
				//split into 3 parts
				std::vector<Plane_Polygon> p_org = *result_n[i].second->poly_-*polys[j];
				std::vector<Plane_Polygon> p_union = *result_n[i].second->poly_ & *polys[j];
				p_new_sw = p_new;
				p_new.reset(new std::vector<Plane_Polygon>);
				for(size_t k=0; k<p_new_sw->size(); k++) {
					std::vector<Plane_Polygon> p_new_tmp = (*p_new_sw)[k]-*result_n[i].second->poly_;
					p_new->insert(p_new->end(), p_new_tmp.begin(), p_new_tmp.end());
				}
				
				insert(result_n[i].second->obj_, p_org);
				
				//split union by dividing line
				box tmp_box = b;
				boost::geometry::expand(tmp_box, result_n[i].first);
				
				const Eigen::Vector3f line_normal = obj->normal_eigen().cross(matched_plane->normal_eigen());
				const Eigen::Vector3f proj_normal = line_normal.cross(matched_plane->normal_eigen());
				Eigen::Vector3f p = matched_plane->offset_eigen()-obj->offset_eigen();
				p = (-obj->normal_eigen().dot(p)/obj->normal_eigen().dot(proj_normal)) * proj_normal + p + obj->offset_eigen();
				
				if(proj_normal.squaredNorm()>0.001f) {
				
#ifdef DEBUG_
					std::cout<<"n1 "<<proj_normal.transpose()<<std::endl;
					std::cout<<"n2 "<<matched_plane->normal_eigen().transpose()<<std::endl;
					std::cout<<"n3 "<<obj->normal_eigen().transpose()<<std::endl;
					
					std::cout<<"dots "
					<< (p-obj->offset_eigen()).dot(obj->normal_eigen()) <<" "
					<< (p-matched_plane->offset_eigen()).dot(matched_plane->normal_eigen()) <<" "
					<< (p+line_normal-obj->offset_eigen()).dot(obj->normal_eigen()) <<" "
					<< (p+line_normal-matched_plane->offset_eigen()).dot(matched_plane->normal_eigen()) <<" "
					<<std::endl;
#endif

					assert(p(0)==p(0));
					assert( std::abs( (p-obj->offset_eigen()).dot(obj->normal_eigen()) )<0.001f );
					assert( std::abs( (p-matched_plane->offset_eigen()).dot(matched_plane->normal_eigen()) )<0.001f );
					assert( std::abs( (p+line_normal-obj->offset_eigen()).dot(obj->normal_eigen()) )<0.001f );
					assert( std::abs( (p+line_normal-matched_plane->offset_eigen()).dot(matched_plane->normal_eigen()) )<0.001f );
					
					Vector2 p1 = projector_(cast(p));
					Vector2 p2 = projector_(cast((Eigen::Vector3f)(p+line_normal)));
					
					Vector2 p_test = projector_(cast((Eigen::Vector3f)(p+proj_normal)));
					
#ifdef DEBUG_
					std::cout<<"p1 "<<p1.transpose()<<std::endl;
					std::cout<<"p2 "<<p2.transpose()<<std::endl;
#endif
					
					assert(p1!=p2);
					
					Eigen::Vector2f p_min(boost::geometry::get<boost::geometry::min_corner, 0>(tmp_box), boost::geometry::get<boost::geometry::min_corner, 1>(tmp_box));
					Eigen::Vector2f p_max(boost::geometry::get<boost::geometry::max_corner, 0>(tmp_box), boost::geometry::get<boost::geometry::max_corner, 1>(tmp_box));
					Eigen::Vector2f center = (p_min+p_max)/2;
					
					Eigen::Vector2f projected_center = ( center-p2 ).dot(p1-p2)/(p1-p2).dot(p1-p2)*(p1-p2) + p2;
					Eigen::Vector2f edge1 = projected_center + (p_min-p_max).norm()*5 * (p1-p2).normalized();
					Eigen::Vector2f edge2 = projected_center - (p_min-p_max).norm()*5 * (p1-p2).normalized();
					
					Eigen::Vector2f edge3_1 = projected_center + (p_min-p_max).norm()*5 * (center-projected_center).normalized();
					Eigen::Vector2f edge3_2 = projected_center - (p_min-p_max).norm()*5 * (center-projected_center).normalized();
					
					Plane_Polygon cut1, cut2;
					
					cut1.boundary().push_back(new Plane_Point(edge1));
					cut1.boundary().push_back(new Plane_Point(edge2));
					cut1.boundary().push_back(new Plane_Point(edge3_1));
					
					cut2.boundary().push_back(new Plane_Point(edge1));
					cut2.boundary().push_back(new Plane_Point(edge2));
					cut2.boundary().push_back(new Plane_Point(edge3_2));
						
#ifdef DEBUG_
					for(size_t l=0; l<cut1.boundary().size(); l++) {
						std::cout<<"e1 "<<cast(Plane_Polygon::to3Dvar(cut1.boundary()[l], obj->pose()).loc_).transpose()<<std::endl;
						std::cout<<"e1 "<<cast(Plane_Polygon::to3Dvar(cut1.boundary()[l], matched_plane->pose()).loc_).transpose()<<std::endl;
					}
					
					std::cout<<"e2 "<<cast(Plane_Polygon::to3Dvar(cut2.boundary().back(), obj->pose()).loc_).transpose()<<std::endl;
					std::cout<<"e2 "<<cast(Plane_Polygon::to3Dvar(cut2.boundary().back(), matched_plane->pose()).loc_).transpose()<<std::endl;
#endif

					if( 
						(p_test-p1).dot(edge3_1-p1) * 
						( ((p+proj_normal) - cast(Plane_Polygon::to3Dvar(Plane_Polygon::to2D(cast((Eigen::Vector3f)(p+proj_normal)), obj->pose()),obj->pose()).loc_)).dot(cast(tf_)*Eigen::Vector3f::UnitZ()) )
						< 0
					 ) {						
						std::swap(cut1.boundary().back(), cut2.boundary().back());
					}
					
					boost::geometry::correct(cut1);
					boost::geometry::correct(cut2);
					//boost::geometry::simplify(cut1, cut1, 0.05);
					//boost::geometry::simplify(cut2, cut2, 0.05);
					
#ifdef DEBUG_
					cut1.save_as_svg("/tmp/cut1.svg");					
					cut2.save_as_svg("/tmp/cut2.svg");
#endif
					
					for(size_t k=0; k<p_union.size(); k++) {
						insert(_obj, p_union[k]-cut1 );
						insert(result_n[i].second->obj_, p_union[k]-cut2 );
					}
				}
				else {
					if( (matched_plane->offset_eigen()-obj->offset_eigen()).dot(obj->normal_eigen())>0 )
						insert(_obj, p_union);
					else
						insert(result_n[i].second->obj_, p_union);
				}
				
			CAST_TO_END;
			
			rtree_.remove(result_n[i]);
		}
		
		insert(_obj, *p_new);
		
	}
	
	CAST_TO_END;
}
			
struct Candidate {
	Plane* p;
	double relation;
	
	Candidate(Plane *p, double r) : p(p), relation(r) {}
};

void Context2D::merge(const Context::Ptr &ctxt)
{
#ifdef DEBUG_
	std::cout<<"merge: "<<std::endl;
#endif

	//1. project polygon to view plane
	
	for(std::vector<Object::Ptr>::iterator it_obj=ctxt->begin(); it_obj!=ctxt->end(); it_obj++) {
			CAST_TO(Plane, (*it_obj), obj);
			
#ifdef DEBUG_
				obj->save_as_svg("/tmp/merge_input.svg");
#endif

				obj->pose().makeTransformWith(tf_);
				
				std::map<Object::Ptr, std::vector<Candidate> > candidates, candidates_neg;
				int unknown=0;
				std::vector<Plane_Polygon::Ptr> polys;
				obj->project(projector_, polys);
				
				std::map<Object::Ptr, std::vector<value> > ints;
				std::vector<value> result;
				box bb;
				if(polys.size()>0)
					bb=ProjectedPolygon::get_box(polys[0]);
				for(size_t j=1; j<polys.size(); j++)
					boost::geometry::expand(bb, ProjectedPolygon::get_box(polys[j]));
					
				std::vector<value> result_n;
				rtree_.query(boost::geometry::index::intersects(bb), std::back_inserter(result_n));
				
				for(size_t k=0; k<result_n.size(); k++)
					ints[result_n[k].second->obj_].push_back(result_n[k]);
				
				for(std::map<Object::Ptr, std::vector<value> >::iterator it=ints.begin(); it!=ints.end(); it++) {	
#ifdef DEBUG_
					{
						double area_in=boost::geometry::area(bb), area_map=0, area_union = 0;
						box bb_tmp;
						for(size_t k=0; k<it->second.size(); k++) {
							area_map += boost::geometry::area(it->second[k].second->get_box());
							if(k==0)
								bb_tmp = it->second[k].second->get_box();
							else
								boost::geometry::expand(bb_tmp, it->second[k].second->get_box());
						}
							
						Eigen::Vector2d a(
							std::max(boost::geometry::get<boost::geometry::min_corner, 0>(bb), boost::geometry::get<boost::geometry::min_corner, 0>(bb_tmp)),
							std::max(boost::geometry::get<boost::geometry::min_corner, 1>(bb), boost::geometry::get<boost::geometry::min_corner, 1>(bb_tmp)) );
						Eigen::Vector2d b(
							std::min(boost::geometry::get<boost::geometry::max_corner, 0>(bb), boost::geometry::get<boost::geometry::max_corner, 0>(bb_tmp)),
							std::min(boost::geometry::get<boost::geometry::max_corner, 1>(bb), boost::geometry::get<boost::geometry::max_corner, 1>(bb_tmp)) );
						area_union += (b-a)(0)*(b-a)(1);
					
						std::cout<<"areas approx "<<100*area_union/std::min(area_in,area_map)<<"% "<<100*area_union/std::max(area_in,area_map)<<"% "<<area_in<<" "<<area_map<<" "<<area_union<<std::endl;

						if(area_union/std::min(area_in,area_map) < 0.3f) continue;
					}
#endif

					double area_in=0, area_map=0, area_union=0;
					{
#ifdef DEBUG_
						std::ofstream svg("/tmp/m3.svg");
						boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);
#endif

						for(size_t j=0; j<polys.size(); j++) {
							area_in += boost::geometry::area(*polys[j]);
							for(size_t k=0; k<it->second.size(); k++) {
								area_map += boost::geometry::area(*it->second[k].second->poly_);
								std::vector<Plane_Polygon> u=*polys[j] & *it->second[k].second->poly_;
								for(size_t n=0; n<u.size(); n++) {
									area_union += boost::geometry::area(u[n]);	
#ifdef DEBUG_
									u[n].save_as_svg(mapper);
#endif
								}
							}
						}
					}
						
#ifdef DEBUG_
					std::cout<<"areas "<<100*area_union/std::min(area_in,area_map)<<"% "<<100*area_union/std::max(area_in,area_map)<<"% "<<area_in<<" "<<area_map<<" "<<area_union<<std::endl;
					
					{std::ofstream svg("/tmp/m1.svg");
					boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);
					
					for(size_t j=0; j<polys.size(); j++)
						polys[j]->save_as_svg(mapper);
					}
					{std::ofstream svg("/tmp/m2.svg");
					boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);
					
					for(size_t k=0; k<it->second.size(); k++)
						it->second[k].second->poly_->save_as_svg(mapper);
					}
#endif
		
					assert(area_union<=area_in+0.00001);
					assert(area_union<=area_map+0.00001);
					
					if( area_union/std::min(area_in,area_map)>0.7f && area_union/std::max(area_in,area_map)>0.2f ) {
						if(	obj->can_merge_fast(*it->first) && it->first->can_merge_fast(*obj) &&
							obj->can_merge(*it->first) && it->first->can_merge(*obj))
							candidates[it->first].push_back(Candidate(obj, std::min(area_in-area_union, area_map-area_union)/std::max(area_in,area_map) ));
						else
							candidates_neg[it->first].push_back(Candidate(obj, std::min(area_in-area_union, area_map-area_union)/std::max(area_in,area_map) ));
					}
					else
						unknown++;
					
				}

#ifdef DEBUG_
				std::cout<<"merge candidates "<<candidates.size()<<" / "<<candidates_neg.size()<<" / "<<unknown<<std::endl;
				if(candidates.size()>1)
					system("read x");
#endif

				size_t bad_ones=0;
				for(std::map<Object::Ptr, std::vector<Candidate> >::iterator it=candidates.begin(); it!=candidates.end(); it++) {
#ifdef DEBUG_
					std::cout<<"    candidates "<<it->second.size()<<std::endl;
#endif
					for(size_t i=0; i<it->second.size(); i++) {
						if(!it->first->merge(*it->second[i].p, it->second[i].relation)) {
							scene_->erase(it->first);
							++bad_ones;
							break;
						}
					}
				}	
#ifdef DEBUG_
				std::cout<<"Xmerge candidates "<<candidates.size()<<" / "<<candidates_neg.size()<<" / "<<unknown<<" / "<<bad_ones<<std::endl;
#endif
				
				if(candidates.size()==bad_ones && candidates_neg.size()==0)
					scene_->add(*it_obj);
				
			CAST_TO_END;
	}
}

bool Context::fitModel(const Object &model, double &score_coverage, double &score_matching)
{
	score_coverage = 0;
	score_matching = 0;
	
	if(ctxt2d_)
		return ctxt2d_->fitModel(model, score_coverage, score_matching);
	
	return false;
}

bool Context2D::fitModel(const Object &model, double &score_coverage, double &score_matching)
{	
	score_coverage = 0;
	score_matching = 0;
	
	const Plane *plane_model = dynamic_cast<const Plane*>(&model);
	if(plane_model) {
	
	//project model in viewport space
	std::vector<Plane_Polygon::Ptr> polys;
	plane_model->project(projector_, polys);
	double area_in = 0;
	for(size_t j=0; j<polys.size(); j++)
		area_in += boost::geometry::area(*polys[j]);
	
	//get bounding box of projected model for query
	box bb;
	if(polys.size()>0)
		bb=ProjectedPolygon::get_box(polys[0]);
	for(size_t j=1; j<polys.size(); j++)
		boost::geometry::expand(bb, ProjectedPolygon::get_box(polys[j]));
	
	//query
	std::vector<value> result_n;
	rtree_.query(boost::geometry::index::intersects(bb), std::back_inserter(result_n));
	
	for(size_t k=0; k<result_n.size(); k++)
	{
		double area_map = boost::geometry::area(*result_n[k].second->poly_), area_union=0;
		for(size_t j=0; j<polys.size(); j++) {
			std::vector<Plane_Polygon> u=*polys[j] & *result_n[k].second->poly_;
			for(size_t n=0; n<u.size(); n++)
				area_union += boost::geometry::area(u[n]);
		}
			
#ifdef DEBUG_
		std::cout<<"areas "<<100*area_union/std::min(area_in,area_map)<<"% "<<100*area_union/std::max(area_in,area_map)<<"% "<<area_in<<" "<<area_map<<" "<<area_union<<std::endl;

		{std::ofstream svg("/tmp/m1.svg");
		boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);
		
		for(size_t j=0; j<polys.size(); j++)
			polys[j]->save_as_svg(mapper);
		}
		{std::ofstream svg("/tmp/m2.svg");
		boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);
		
		result_n[k].second->poly_->save_as_svg(mapper);
		}
#endif

		assert(area_union<=area_in+0.00001);
		assert(area_union<=area_map+0.00001);
		
		//decide wether model is in front, behind or the same as perceived surfaces
		//case: same
		if(	plane_model->can_merge_fast(*result_n[k].second->obj_) && result_n[k].second->obj_->can_merge_fast(*plane_model) &&
				plane_model->can_merge(*result_n[k].second->obj_) && result_n[k].second->obj_->can_merge(*plane_model) ) {
			score_matching += area_union/area_in;
			score_coverage += area_union/area_in;
		}
		//case: behind
		else if(result_n[k].second->obj_->in_front_of(*plane_model))
			score_coverage += area_union/area_in;
	}
		
	assert(score_matching<=1);
	assert(score_coverage<=1);
	assert(score_matching<=score_coverage);
	
	return true;
	}
	
	return false;
}
