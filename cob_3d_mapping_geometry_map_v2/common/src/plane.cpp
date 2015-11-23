#include <cob_3d_mapping_geometry_map_v2/types/plane.h>
#include <cob_3d_mapping_geometry_map_v2/types/impl/cutter.hpp>
#include <cob_3d_mapping_geometry_map_v2/visualization/vis_obj.h>
#include <boost/geometry/io/svg/svg_mapper.hpp>

using namespace cob_3d_geometry_map;


void Plane_Polygon::visualization(const std::string &name, std::vector<boost::shared_ptr<Visualization::Object> > &objs, const Eigen::Affine3f &pose)
{
	save_as_svg("/tmp/debug.svg");
	
	std::list<TPPLPoly> polys,result;
	
	internal_add_triangulation(boundary_, polys, false);
	for(size_t i=0; i<holes_.size(); i++)
		internal_add_triangulation(holes_[i], polys, true);

	TPPLPartition pp;
	int r = pp.Triangulate_EC(&polys,&result);
	assert(r);

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
	
	mesh->color().head<3>() = color_;
	objs.push_back( mesh );
	
	for(size_t j=0; j<boundary_.size(); j++)
		objs.push_back( Visualization::Object::Ptr( new Visualization::Sphere(name+": bandwith", pose*Eigen::Vector3f(boundary_[j]->pos(0), boundary_[j]->pos(1), 0.f), boundary_[j]->var) ) );
	for(size_t i=0; i<holes_.size(); i++)
		for(size_t j=0; j<holes_[i].size(); j++)
			objs.push_back( Visualization::Object::Ptr( new Visualization::Sphere(name+": bandwith", pose*Eigen::Vector3f(holes_[i][j]->pos(0), holes_[i][j]->pos(1), 0.f), holes_[i][j]->var) ) );
}

//convert from ros msg
Plane_Point::Plane_Point(const cob_3d_mapping_msgs::Point2D &pt) :
 pos(pt.x, pt.y), tex(pt.tex_x, pt.tex_y), var(pt.var)
{
}

Plane_Polygon::Plane_Polygon(const std::vector<cob_3d_mapping_msgs::Polygon> &poly, const Eigen::Vector3f &color) :
	color_(color)
{
	if(poly.size()<1) return;
	
	boundary_.from(poly[0]);
	for(size_t i=1; i<poly.size(); i++) {
		bool ok=true;
		for(size_t j=0; j<poly[i].points.size() && ok; j++)
			ok &= boost::geometry::within(Plane_Point(poly[i].points[j]), *this);
		if(ok) holes_.push_back(poly[i]);
		else ROS_WARN("hole in polygon is invalid as its outside");
	}
}

void Plane_Polygon::save_as_svg(const std::string &fn, const bool check) const
{	
	{
		std::ofstream svg(fn.c_str());
		boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);

		save_as_svg(mapper);
	}
	
	assert( !check||boost::geometry::is_valid(*this) );
	assert( !check||!boost::geometry::intersects(*this) );
}

void Plane_Polygon::save_as_svg(boost::geometry::svg_mapper<Plane_Point::Ptr> &mapper, const int color_b, const int color_g) const
{
	char buf[512];
	sprintf(buf, "fill:none;stroke:rgb(0,%d,%d);stroke-width:5", color_g, color_b);
    // Add geometries such that all these geometries fit on the map
    mapper.add(boundary_);
    mapper.map(boundary_, buf);
    
	for(size_t i=0; i<holes_.size(); i++) {
		mapper.add(holes_[i]);
		sprintf(buf, "fill:none;stroke:rgb(255,%d,%d);stroke-width:5", color_g, color_b);
		mapper.map(holes_[i], buf);
	}
}

Plane::Plane(const ContextPtr &ctxt, const cob_3d_mapping_msgs::Plane &inp) :
	ObjectVolume(ctxt)
{
	Eigen::Affine3d pose;
	tf::poseMsgToEigen(inp.pose, pose);
	pose_ = cast(pose);
	
	polygons_.push_back(Plane_Polygon::Ptr(new Plane_Polygon(inp.polygons, Eigen::Vector3f(inp.color.r,inp.color.g,inp.color.b) )));
	boost::geometry::correct(*polygons_.back());
	
	buildBB();
}

void Plane::buildBB()
{
	bb_.setEmpty();
	
	for(size_t i=0; i<polygons_.size(); i++)
		polygons_[i]->buildBB(bb_);
	
	if(polygons_.size()>0) {
		pose_.loc_h_ = polygons_.back()->max_var();
		pose_.ori_h_ = std::atan2(pose_.loc_h_, (bb_.max()-bb_.min()).norm()/2);
		
	}
}

void Plane_Polygon::buildBB(ObjectVolume::TBB &bb)
{		
	//we only consider boundary here
	for(size_t j=0; j<boundary_.size(); j++) {
		Eigen::Vector3f pt = Eigen::Vector3f(boundary_[j]->pos(0), boundary_[j]->pos(1), 0.f);
		
		std::cout<<"v "<<boundary_[j]->var<<std::endl;
		
		bb.extend(pt + boundary_[j]->var*Eigen::Vector3f::UnitX());
		bb.extend(pt - boundary_[j]->var*Eigen::Vector3f::UnitX());
		bb.extend(pt + boundary_[j]->var*Eigen::Vector3f::UnitY());
		bb.extend(pt - boundary_[j]->var*Eigen::Vector3f::UnitY());
		bb.extend(pt + boundary_[j]->var*Eigen::Vector3f::UnitZ());
		bb.extend(pt - boundary_[j]->var*Eigen::Vector3f::UnitZ());
	}
}

void Plane_Polygon::internal_add_triangulation(const Plane_Ring &inp, std::list<TPPLPoly> &polys, const bool hole)
{
	//std::cout<<"order "<<hole<<" "<<boost::geometry::point_order<Plane_Ring>::value<<std::endl;
	//std::cout<< boost::geometry::dsv(inp) <<std::endl;
	
	TPPLPoly poly;

	poly.Init(inp.size()-1);

	for(size_t j=0; j<inp.size()-1; j++) {
	  poly[j].x = inp[j]->pos(0);
	  poly[j].y = inp[j]->pos(1);
	}
	
	poly.SetHole(hole);
	poly.SetOrientation(hole?TPPL_CW:TPPL_CCW);	//handles orientation by itself by inverting list
	
	polys.push_back(poly);
}
	
void Plane::visualization(std::vector<boost::shared_ptr<Visualization::Object> > &objs)
{
	Parent::visualization(objs);
	
	for(size_t i=0; i<polygons_.size(); i++)
		polygons_[i]->visualization(vis_name("Plane"), objs, cast(pose_));
}

double Plane::polygon_distance(const Plane_Point &pt) const {
	double d=-1;
	for(size_t i=0; i<polygons_.size() && d!=0; i++)
		d = std::min(d, polygons_[i]->polygon_distance(pt));
	return d;
}

double Plane_Polygon::polygon_distance(const Plane_Point &pt) const {
	if(boost::geometry::within(pt, *this))
		return 0;
	return boost::geometry::distance(pt, *this);
}
	

bool Plane::snap(const Plane &other)
{
	assert(this!=&other);
	return cut(VolumeCut_Snapper(other));
}

bool Plane::cut(const VolumeCut &cutter)
{
	for(size_t i=0; i<polygons_.size(); i++) {
		if(polygons_[i]->cut(cutter, pose_)) {
			polygons_.erase(polygons_.begin()+i);
			--i;
		}
	}
	buildBB();
	
	return polygons_.size()==0;
}

bool Plane_Polygon::cut(const VolumeCut &cutter, const nuklei::kernel::se3 &pose)
{
	if(cut(cutter, pose, boundary_))
		return true; //now empty
	
	for(size_t i=0; i<holes_.size(); i++)
		if(cut(cutter, pose, holes_[i])) {
			holes_.erase(holes_.begin()+i);
			--i;
		}
		
	for(size_t i=0; i<holes_.size(); i++) {
		if(!boost::geometry::within(holes_[i], boundary_)) {
			boost::geometry::difference(boundary_, holes_[i], boundary_);
			holes_.erase(holes_.begin()+i);
			--i;
		}
	}
	
	/*std::cout<<"valid: "<<boost::geometry::is_valid(boundary_)<<std::endl;
	for(size_t i=0; i<holes_.size(); i++)
		std::cout<<"valid: "<<boost::geometry::is_valid( holes_[i])<<std::endl;
	std::vector<boost::shared_ptr<Visualization::Object> > objs;
	visualization("safd", objs, cast(pose));
	assert(boost::geometry::is_valid(boundary_));*/
		
	return !boost::geometry::is_valid(boundary_);
}

bool Plane_Polygon::cut(const VolumeCut &cutter, const nuklei::kernel::se3 &pose, Plane_Ring &inp)
{
	ROS_INFO("cut--------------");
	//save_as_svg("/tmp/debug_bef.svg");
	inp.erase(inp.end()-1);
	
	std::vector<bool> changes(inp.size(), false);
	size_t cnt=0;
	for(size_t i=0; i<inp.size(); i++) {
		//inp[i]->var += 0.01f;
		nuklei::kernel::r3 pt = to3Dvar(inp[i], pose);
		if(cutter.cut(pt)) {
			changes[i]= true;
			++cnt;
		}
	}
	
	if(cnt==inp.size()) return true;
	
	for(size_t i=0; i<inp.size(); i++) {
		const size_t n  = (i+inp.size()-1)%inp.size();
		const size_t n2  = (i+1)%inp.size();
		
		if(changes[i] && changes[n] && changes[n2]) {
			ROS_INFO("cutting out point %d/%d", (int)i, (int)inp.size());
			inp.erase(inp.begin()+i);
			changes.erase(changes.begin()+i);	//TODO: optimize
			--i;
		}
		else if(changes[i]) {
			inp[i]->pos = to2D(cutter.cutting_point(to3Dvar(inp[i], pose)), pose);
			ROS_INFO("update point %d", (int)n);
		}
	}
	
	if(inp.size()>0) {
		inp.push_back(inp.front());
		/*std::cout<<"valid: "<<boost::geometry::is_valid(inp)<<std::endl;
		std::vector<boost::shared_ptr<Visualization::Object> > objs;
		visualization("safd", objs, cast(pose));
		assert(boost::geometry::is_valid(inp));*/
	}
		
	return inp.size()<4;
}

double area2(const Plane_Point::Vector2 &a, const Plane_Point::Vector2 &b) {
	return std::pow(a(0)*b(1)-a(1)*b(0), 2);
}

void Plane_Ring::simplify_by_area(const double min_area2) {
	erase(end()-1);
	
	std::cout<<"simplify_by_area: "<<size()<<" -> ";
	for(size_t i=0; i<size(); i++) {
		const size_t ind_1 = (i+1)%size();
		const size_t ind_2 = (i+2)%size();
		
		const double area = area2( (*this)[ind_1]->pos-(*this)[i]->pos, (*this)[ind_2]->pos-(*this)[ind_1]->pos);
		
		if(area<=min_area2) {
			erase(begin()+ind_1);
			if(ind_1<i) break;
		}
	}
	std::cout<<size()<<std::endl;
	
	if(size()>0)
		push_back(front());	//close
}

bool Plane_Polygon::simplify_by_area(const double min_area) {
	boundary_.simplify_by_area(min_area*min_area);
	for(size_t i=0; i<holes_.size(); i++) {
		holes_[i].simplify_by_area(min_area*min_area);
		if(holes_[i].size()<4) {
			holes_.erase(holes_.begin()+i);
			--i;
			continue;
		}
	}
	
	return boundary_.size()>3;	//4 points for triangle (with closing point)
}

void Plane_Polygon::clone(const Plane_Polygon &o, const Projector &proj, const nuklei::kernel::se3 &pose) {
	boundary_.clone(o.boundary_, proj, pose);
	holes_.resize(o.holes_.size());
	for(size_t i=0; i<holes_.size(); i++)
		holes_[i].clone(o.holes_[i], proj, pose);
}

void Plane::project(const Projector &proj, std::vector<Plane_Polygon::Ptr> &res) const
{
	res.resize(polygons_.size());
	for(size_t i=0; i<polygons_.size(); i++) {
		res[i].reset(new Plane_Polygon(*polygons_[i], proj, pose()));
		boost::geometry::correct(*res[i]);
		res[i]->save_as_svg("/tmp/proj.svg");
	}
}

std::vector<Plane_Polygon> Plane_Polygon::operator-(const Plane_Polygon &o) const
{	
		save_as_svg("/tmp/in1.svg");
		o.save_as_svg("/tmp/in2.svg");
		
		std::cout<<"-i1 "<<holes_.size()<<std::endl;
		std::cout<<"-i2 "<<o.holes_.size()<<std::endl;
		
	std::vector<Plane_Polygon> rs;
	boost::geometry::difference(*this,o, rs);
	
	{	
		std::cout<<"-size "<<rs.size()<<std::endl;
		
		std::ofstream svg("/tmp/op.svg");
		boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);

		// Add geometries such that all these geometries fit on the map
		
		for(size_t i=0; i<rs.size(); i++) {
			std::cout<<"-o "<<rs[i].holes_.size()<<std::endl;
			std::cout<<"-size "<<rs[i].boundary_.size()<<std::endl;
			mapper.add(rs[i].boundary_);
			char buf[512]; sprintf(buf, "fill:none;stroke:rgb(%d,0,0);stroke-width:5", (int)i*100);
			mapper.map(rs[i].boundary_, buf);
		}
	}
	
	for(size_t i=0; i<rs.size(); i++) {
		const bool sim_res = rs[i].simplify_by_area(0.000001); //remove double lines
		
		if(!sim_res || !boost::geometry::is_valid(rs[i]) || boost::geometry::intersects(rs[i]) ) {
			ROS_ERROR("invalid result from op-");
			rs[i].save_as_svg("/tmp/res.svg", false);
	  if(sim_res) system("read x");
			rs.erase(rs.begin()+i);
			--i; continue;
		}
		rs[i].save_as_svg("/tmp/res.svg");
		
		rs[i].color_ = color_;
		rs[i].imgs_ = imgs_; //TODO: check for o
	}
	
	return rs;
}

std::vector<Plane_Polygon> Plane_Polygon::operator+(const Plane_Polygon &o) const
{
	std::vector<Plane_Polygon> rs;
	boost::geometry::union_(*this,o, rs);
	
	save_as_svg("/tmp/in1.svg");
	o.save_as_svg("/tmp/in2.svg");
	
	std::cout<<"+i1 "<<holes_.size()<<std::endl;
	std::cout<<"+i2 "<<o.holes_.size()<<std::endl;
	std::cout<<"+size "<<rs.size()<<std::endl;
	
    std::ofstream svg("/tmp/op.svg");
    boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);

    // Add geometries such that all these geometries fit on the map
    
	for(size_t i=0; i<rs.size(); i++) {
		std::cout<<"+o "<<rs[i].holes_.size()<<std::endl;
		std::cout<<"+size "<<rs[i].boundary_.size()<<std::endl;
		mapper.add(rs[i].boundary_);
		char buf[512]; sprintf(buf, "fill:none;stroke:rgb(%d,0,0);stroke-width:5", (int)i*100);
		mapper.map(rs[i].boundary_, buf);
	}
	
	//assert(rs.size()>=1);
	
	for(size_t i=0; i<rs.size(); i++) {
		rs[i].color_ = color_;
		rs[i].imgs_ = imgs_; //TODO: check for o
		
		assert( boost::geometry::is_valid(rs[i]) );
	}
	
	return rs;
}

std::vector<Plane_Polygon> Plane_Polygon::operator&(const Plane_Polygon &o) const
{
	std::cout<<"&i1 "<<holes_.size()<<std::endl;
	std::cout<<"&i2 "<<o.holes_.size()<<std::endl;
	
	save_as_svg("/tmp/in1.svg");
	o.save_as_svg("/tmp/in2.svg");
    {
		std::ofstream svg("/tmp/in_both.svg");
		boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);
		save_as_svg(mapper);
		o.save_as_svg(mapper);
	}
	
	std::vector<Plane_Polygon> rs;
	//if( boost::geometry::overlaps(o, *this) )
		boost::geometry::intersection(*this,o, rs);
	
	std::cout<<"&size "<<rs.size()<<std::endl;
	
    {
		std::ofstream svg("/tmp/op.svg");
		boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);

		// Add geometries such that all these geometries fit on the map
		
		for(size_t i=0; i<rs.size(); i++) {
			std::cout<<"&o "<<rs[i].holes_.size()<<std::endl;
			std::cout<<"&size "<<rs[i].boundary_.size()<<std::endl;
			mapper.add(rs[i].boundary_);
			char buf[512]; sprintf(buf, "fill:none;stroke:rgb(%d,0,0);stroke-width:5", (int)i*100);
			mapper.map(rs[i].boundary_, buf);
		}
		
	}
	
	for(size_t i=0; i<rs.size(); i++) {
		if(!boost::geometry::is_valid(rs[i])) {
			ROS_ERROR("invalid result from op&");
	  system("read x");
			//rs[i].save_as_svg("/tmp/res.svg");
			rs.erase(rs.begin()+i);
			--i; continue;
		}
		
		rs[i].color_ = color_;
		rs[i].imgs_ = imgs_; //TODO: check for o
	}
	
	return rs;
}

bool Plane::can_merge_fast(const Object &o) const {
	Plane const *plane = dynamic_cast<Plane const*>(&o);
	if(plane) {
		const nuklei::kernel::r3xs2 params1 = plane_params();
		const nuklei::kernel::r3xs2 params2 = plane->plane_params();
		nuklei::coord_pair dist( std::abs( (params1.loc_-params2.loc_).Dot(params1.dir_) ), 1-std::abs(params2.dir_.Dot(params1.dir_)) );
		
		std::cout<<"can_merge_fast "<<dist.first<<" "<<dist.second<<" -- "<<params1.loc_h_+params2.loc_h_<<" "<<params1.dir_h_+params2.dir_h_<<std::endl;
		
		if(dist.first<=params1.loc_h_+params2.loc_h_ && dist.second<=params1.dir_h_+params2.dir_h_)
			return true;
	}
	
	return false;
}

bool Plane::can_merge(const Object &o) const {
	assert( can_merge_fast(o) );
	
	return true;
}

bool Plane::merge(const Object &o, const double relation) {
	Plane const *plane = dynamic_cast<Plane const*>(&o);
	if(plane) {
		Plane copy = *this;
		polygons_.clear();
		
		//1. get new pose
		pose_ = copy.pose_.linearInterpolation(plane->pose_, (plane->pose_.ori_h_+plane->pose_.loc_h_)/(copy.pose_.ori_h_+copy.pose_.loc_h_ + plane->pose_.ori_h_+plane->pose_.loc_h_) );	//TODO: weightning
		
		std::cout<<"THIS\n"<<normal_eigen().transpose()<<"\n"<<offset_eigen().transpose()<<std::endl;
		std::cout<<"COPY\n"<<copy.normal_eigen().transpose()<<"\n"<<copy.offset_eigen().transpose()<<std::endl;
		std::cout<<"OTHER\n"<<plane->normal_eigen().transpose()<<"\n"<<plane->offset_eigen().transpose()<<std::endl;
		
		//2. project to new pose
		//3. split into 3 parts
		std::ofstream svg2("/tmp/merged_bef.svg");
		boost::geometry::svg_mapper<Plane_Point::Ptr> mapper2(svg2, 800, 800);
		
		for(size_t i=0; i<copy.polygons_.size(); i++) {
			for(size_t j=0; j<plane->polygons_.size(); j++)
				if(relation<0.5)
					complex_projection(this, &copy, plane, i, j);
				else
					simple_projection(this, &copy, plane, i, j);
			copy.polygons_[i]->save_as_svg(mapper2);
		}
				
		std::ofstream svg("/tmp/merged.svg");
		boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);

		for(size_t i=0; i<polygons_.size(); i++) {
			char buf[128]; sprintf(buf, "/tmp/merged%d.svg", (int)i); 
			polygons_[i]->save_as_svg(mapper);
			polygons_[i]->save_as_svg(buf);
		}
		
		buildBB();
		
		return simplify_by_area(0.02*0.02);
	}
	
	return true;
}

class Projector_Plane : public Projector {
	Plane* plane_;
	
public:
	Projector_Plane(Plane* plane) : plane_(plane) {
	}
	
	virtual Plane_Point::Vector2 operator()(const nuklei_wmf::Vector3<double> &pt3) const {
		return Plane_Polygon::to2D(pt3, plane_->pose());
	}
	
	virtual Plane_Polygon operator()(const Plane &plane_other, const Plane_Polygon &poly) const {
		return Plane_Polygon(poly, *this, plane_other.pose());
	}
};

void Plane::complex_projection(Plane* plane_out, const Plane* plane_in1, const Plane* plane_in2, const size_t ind1, const size_t ind2)
{
	assert(plane_in1);
	assert(plane_in2);
	
	static int n=0;
	char buf[256];
	
	sprintf(buf, "/tmp/merge_%d_%d_%d_%d.svg", n++, plane_in1->id(), plane_in2->id(), plane_out->id());
	std::ofstream svg(buf);
	boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);
	
	plane_in1->save_as_svg(mapper, 100);
	plane_in2->save_as_svg(mapper, 200);
	
	Projector_Plane projector(plane_out);
	
	//split into 3 parts
	std::vector<Plane_Polygon> p_org   = projector(*plane_in2, *plane_in2->polygons_[ind2]) - projector(*plane_in1, *plane_in1->polygons_[ind1]);
	std::vector<Plane_Polygon> p_new   = projector(*plane_in1, *plane_in1->polygons_[ind1]) - projector(*plane_in2, *plane_in2->polygons_[ind2]);
	std::vector<Plane_Polygon> p_union = projector(*plane_in2, *plane_in2->polygons_[ind2]) & projector(*plane_in1, *plane_in1->polygons_[ind1]);
	
	plane_out->insert(plane_in2, p_org);
	plane_out->insert(plane_in1, p_new);
	
	//split union by dividing line
	
	const Eigen::Vector3f line_normal = plane_in1->normal_eigen().cross(plane_in2->normal_eigen());
	const Eigen::Vector3f proj_normal = line_normal.cross(plane_in2->normal_eigen());
	Eigen::Vector3f p = plane_in2->offset_eigen()-plane_in1->offset_eigen();
	p = (-plane_in1->normal_eigen().dot(p)/plane_in1->normal_eigen().dot(proj_normal)) * proj_normal + p + plane_in1->offset_eigen();
	
	if(proj_normal.squaredNorm()>0.001f) {
	
		std::cout<<"n1 "<<proj_normal.transpose()<<std::endl;
		std::cout<<"n2 "<<plane_in2->normal_eigen().transpose()<<std::endl;
		std::cout<<"n3 "<<plane_in1->normal_eigen().transpose()<<std::endl;
		
		std::cout<<"dots "
		<< (p-plane_in1->offset_eigen()).dot(plane_in1->normal_eigen()) <<" "
		<< (p-plane_in2->offset_eigen()).dot(plane_in2->normal_eigen()) <<" "
		<< (p+line_normal-plane_in1->offset_eigen()).dot(plane_in1->normal_eigen()) <<" "
		<< (p+line_normal-plane_in2->offset_eigen()).dot(plane_in2->normal_eigen()) <<" "
		<<std::endl;
		
		assert(p(0)==p(0));
		assert( std::abs( (p-plane_in1->offset_eigen()).dot(plane_in1->normal_eigen()) )<0.001f );
		assert( std::abs( (p-plane_in2->offset_eigen()).dot(plane_in2->normal_eigen()) )<0.001f );
		assert( std::abs( (p+line_normal-plane_in1->offset_eigen()).dot(plane_in1->normal_eigen()) )<0.001f );
		assert( std::abs( (p+line_normal-plane_in2->offset_eigen()).dot(plane_in2->normal_eigen()) )<0.001f );
		
		Plane_Point::Vector2 p1 = projector(cast(p));
		Plane_Point::Vector2 p2 = projector(cast((Eigen::Vector3f)(p+line_normal)));
		
		Plane_Point::Vector2 p_test = projector(cast((Eigen::Vector3f)(p+proj_normal)));
		
		std::cout<<"p1 "<<p1.transpose()<<std::endl;
		std::cout<<"p2 "<<p2.transpose()<<std::endl;
		
		assert(p1!=p2);
		
		typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
		typedef boost::geometry::model::box<point_t> box;
		
		box tmp_box, tmp_box2;
		boost::geometry::envelope(projector(*plane_in1, *plane_in1->polygons_[ind1]), tmp_box);
		boost::geometry::envelope(projector(*plane_in2, *plane_in2->polygons_[ind2]), tmp_box2);
		boost::geometry::expand(tmp_box, tmp_box2);
		
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
			
		for(size_t l=0; l<cut1.boundary().size(); l++) {
			std::cout<<"e1 "<<cast(Plane_Polygon::to3Dvar(cut1.boundary()[l], plane_in1->pose()).loc_).transpose()<<std::endl;
			std::cout<<"e1 "<<cast(Plane_Polygon::to3Dvar(cut1.boundary()[l], plane_in2->pose()).loc_).transpose()<<std::endl;
		}
		
		std::cout<<"e2 "<<cast(Plane_Polygon::to3Dvar(cut2.boundary().back(), plane_in1->pose()).loc_).transpose()<<std::endl;
		std::cout<<"e2 "<<cast(Plane_Polygon::to3Dvar(cut2.boundary().back(), plane_in2->pose()).loc_).transpose()<<std::endl;
		
		if( 
			(p_test-p1).dot(edge3_1-p1) * 
			( (p+proj_normal)(2) - Plane_Polygon::to3Dvar(Plane_Polygon::to2D(cast((Eigen::Vector3f)(p+proj_normal)), plane_out->pose()),plane_out->pose()).loc_.Z())
			< 0
		 ) {						
			std::swap(cut1.boundary().back(), cut2.boundary().back());
		}
		
		boost::geometry::correct(cut1);
		boost::geometry::correct(cut2);
		//boost::geometry::simplify(cut1, cut1, 0.05);
		//boost::geometry::simplify(cut2, cut2, 0.05);
		
		cut1.save_as_svg("/tmp/cut1.svg");					
		cut2.save_as_svg("/tmp/cut2.svg");
		
		for(size_t k=0; k<p_union.size(); k++) {
			plane_out->insert(plane_in1, p_union[k]-cut1 );
			plane_out->insert(plane_in2, p_union[k]-cut2 );
		}
	}
	else {
		if( (plane_in2->offset_eigen()-plane_in1->offset_eigen()).dot(Eigen::Vector3f::UnitZ())>0 )
			plane_out->insert(plane_in1, p_union);
		else
			plane_out->insert(plane_in2, p_union);
	}
	
	plane_out->save_as_svg(mapper);
}

void Plane::save_as_svg(const std::string &fn) const
{	
	{
		std::ofstream svg(fn.c_str());
		boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);

		save_as_svg(mapper);
	}
	
	for(size_t i=0; i<polygons_.size(); i++) {
		assert( boost::geometry::is_valid(*polygons_[i]) );
		assert( !boost::geometry::intersects(*polygons_[i]) );
	}
}

void Plane::save_as_svg(boost::geometry::svg_mapper<Plane_Point::Ptr> &mapper, const int color) const
{	
	char buf[128];
	sprintf(buf, "id: %d", id());
//	if(polygons_.size()>0)
//		mapper.text(boost::geometry::return_centroid<Plane_Point>(*polygons_[0]), buf, "fill:none;stroke:rgb(0,255,0);stroke-width:5");
	for(size_t i=0; i<polygons_.size(); i++)
		polygons_[i]->save_as_svg(mapper, (i*77)%256, color);
}

bool Plane::simplify_by_area(const double min_area) {
	bool ok=false;
	for(size_t i=0; i<polygons_.size(); i++) {
		const bool r = polygons_[i]->simplify_by_area(min_area);
		if(!r) {
			polygons_.erase(polygons_.begin()+i);
			--i;
		}
		ok |= r;
	}
	return ok;
}


void Plane::simple_projection(Plane* plane_out, const Plane* plane_in1, const Plane* plane_in2, const size_t ind1, const size_t ind2)
{
	assert(plane_in1);
	assert(plane_in2);
	
	static int n=0;
	char buf[256];
	
	sprintf(buf, "/tmp/merge_%d_%d_%d_%d.svg", n++, plane_in1->id(), plane_in2->id(), plane_out->id());
	std::ofstream svg(buf);
	boost::geometry::svg_mapper<Plane_Point::Ptr> mapper(svg, 800, 800);
	
	plane_in1->save_as_svg(mapper, 100);
	plane_in2->save_as_svg(mapper, 200);
	
	Projector_Plane projector(plane_out);
	
	plane_out->insert(NULL, projector(*plane_in2, *plane_in2->polygons_[ind2]) + projector(*plane_in1, *plane_in1->polygons_[ind1]));
	
	
	plane_out->save_as_svg(mapper);
}
