#include <cob_3d_mapping_geometry_map_v2/types/plane.h>
#include <cob_3d_mapping_geometry_map_v2/visualization/vis_obj.h>

using namespace cob_3d_geometry_map;


void Plane_Polygon::visualization(const std::string &name, std::vector<boost::shared_ptr<Visualization::Object> > &objs, const Eigen::Affine3f &pose)
{
	std::list<TPPLPoly> polys,result;
	
	internal_add_triangulation(boundary_, polys, false, TPPL_CCW);
	for(size_t i=0; i<holes_.size(); i++)
		internal_add_triangulation(holes_[i], polys, true, TPPL_CW);

	TPPLPartition pp;
	pp.Triangulate_EC(&polys,&result);

	TPPLPoint p1,p2,p3;
	Eigen::Vector3f v1,v2,v3;

	Visualization::Mesh::Ptr mesh( new Visualization::Mesh(name) );
	
	for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
		assert(it->GetNumPoints()==3);

		p1 = it->GetPoint(0); p2 = it->GetPoint(1); p3 = it->GetPoint(2);
		
		mesh->add(
			pose*Eigen::Vector3f(p1.x, p1.y, 0.f),
			pose*Eigen::Vector3f(p2.x, p2.y, 0.f),
			pose*Eigen::Vector3f(p3.x, p3.y, 0.f)
		);
	}
	
	objs.push_back( mesh );
	
	for(size_t j=0; j<boundary_.size(); j++)
		objs.push_back( Visualization::Object::Ptr( new Visualization::Sphere(name+": bandwith", pose*Eigen::Vector3f(boundary_[j].pos(0), boundary_[j].pos(1), 0.f), boundary_[j].var) ) );
	for(size_t i=0; i<holes_.size(); i++)
		for(size_t j=0; j<holes_[i].size(); j++)
			objs.push_back( Visualization::Object::Ptr( new Visualization::Sphere(name+": bandwith", pose*Eigen::Vector3f(holes_[i][j].pos(0), holes_[i][j].pos(1), 0.f), holes_[i][j].var) ) );
}

//convert from ros msg
Plane_Point::Plane_Point(const cob_3d_mapping_msgs::Point2D &pt) :
 pos(pt.x, pt.y), tex(pt.tex_x, pt.tex_y), var(pt.var)
{
}

Plane_Polygon::Plane_Polygon(const std::vector<cob_3d_mapping_msgs::Polygon> &poly)
{
	if(poly.size()<1) return;
	
	boundary_.from(poly[0]);
	holes_.insert(holes_.begin(), poly.begin()+1, poly.end());
}

Plane::Plane(const ContextPtr &ctxt, const cob_3d_mapping_msgs::Plane &inp) :
	ObjectVolume(ctxt)
{
	Eigen::Affine3d pose;
	tf::poseMsgToEigen(inp.pose, pose);
	pose_ = cast(pose);
	
	polygons_.push_back(Plane_Polygon::Ptr(new Plane_Polygon(inp.polygons)));
	
	//bb_.setEmpty();
	//bb_.extend(vec3)
}

void Plane_Polygon::internal_add_triangulation(const Plane_Ring &inp, std::list<TPPLPoly> &polys, const bool hole, const int orientation)
{
	TPPLPoly poly;

	poly.Init(inp.size());
	poly.SetHole(hole);

	for(size_t j=0; j<inp.size(); j++) {
	  poly[j].x = inp[j].pos(0);
	  poly[j].y = inp[j].pos(1);
	}
	poly.SetOrientation(orientation);
	polys.push_back(poly);
}
	
void Plane::visualization(std::vector<boost::shared_ptr<Visualization::Object> > &objs)
{
	Parent::visualization(objs);
	
	for(size_t i=0; i<polygons_.size(); i++)
		polygons_[i]->visualization(vis_name("Plane"), objs, cast(pose_));
}

