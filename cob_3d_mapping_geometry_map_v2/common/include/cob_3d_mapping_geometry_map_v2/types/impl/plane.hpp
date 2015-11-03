
//convert from ros msg
Plane_Point::Plane_Point(const cob_3d_mapping_msgs::Point2D &pt) :
 pos(pt.x, pt.y), tex(pt.tex_x, pt.tex_y)
{
	std::copy(pt.var.begin(), pt.var.end(), var_.data());
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
	Eigen::Affine3d _pose;
	Eigen::Affine3f pose;
	tf::poseMsgToEigen(inp.pose, _pose);
	pose = _pose.cast<float>();
	
	normal_ = pose.rotation()*Eigen::Vector3f::UnitZ();
	offset_ = pose.translation();
	
	polygons_.push_back(Plane_Polygon(inp.polygons));
	
	//bb_.setEmpty();
	//bb_.extend(vec3)
}

#if 0
{

      TPPLPartition pp;
      std::list<TPPLPoly> polys,result;

      //std::cout << "id: " << new_message->id << std::endl;
      //std::cout << new_message->centroid << std::endl;
      //fill polys
      for(size_t i=0; i<boundary_.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ> pc;
        TPPLPoly poly;

        poly.Init(boundary_.size());
        poly.SetHole(false);

        for(size_t j=0; j<boundary_.size(); j++) {
          poly[j].x = segments_[i][j](0);
          poly[j].y = segments_[i][j](1);
        }
        if(i!=0)
          poly.SetOrientation(TPPL_CW);
        else
          poly.SetOrientation(TPPL_CCW);

        polys.push_back(poly);
      }

      pp.Triangulate_EC(&polys,&result);

      TPPLPoint p1,p2,p3;
      Eigen::Vector3f v1,v2,v3;

      float a=0.f;
      for(std::list<TPPLPoly>::iterator it=result.begin(); it!=result.end(); it++) {
        if(it->GetNumPoints()!=3) continue;

        p1 = it->GetPoint(0);
        p2 = it->GetPoint(1);
        p3 = it->GetPoint(2);

        v1(0) = p1.x;
        v1(1) = p1.y;
        v2(0) = p2.x;
        v2(1) = p2.y;
        v3(0) = p3.x;
        v3(1) = p3.y;

        v1 = project2world(v1.head<2>());
        v2 = project2world(v2.head<2>());
        v3 = project2world(v3.head<2>());

        a += (v2-v1).cross(v3-v1).norm();
      }

}

/*
geometry_msgs/Pose pose
float32 weight
std_msgs/ColorRGBA color
  float32 r
  float32 g
  float32 b
  float32 a
*/
#endif
