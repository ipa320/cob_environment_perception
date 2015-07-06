
#include "cob_3d_mapping_geometry_map/geometry_map.h"
#include <pcl/io/pcd_io.h>

using namespace cob_3d_mapping;

bool GeometryMapEntry_Polygon::isMergeCandidate(const GeometryMapEntry::Ptr &o) const {
	if(!o || o->getType()!=getType()) return false;
	return polygon_->hasSimilarParametersWith( ((GeometryMapEntry_Polygon*)o.get())->polygon_ ) && polygon_->isIntersectedWith( ((GeometryMapEntry_Polygon*)o.get())->polygon_ );
}

bool sort_polygons_by_area(const Polygon::Ptr &p1, const Polygon::Ptr &p2) {
	return p1->computeArea3d()>p2->computeArea3d()*std::pow(1.05f, (float)(p2->frame_stamp_-p1->frame_stamp_));
}

bool GeometryMapEntry_Polygon::merge(const GeometryMapEntry::Ptr &o) {
	if(!o || o->getType()!=getType()) return false;
	
	if( !polygon_->hasSimilarColorWith( ((GeometryMapEntry_Polygon*)o.get())->polygon_ ) )
		return false;
	
	std::vector<Polygon::Ptr> tmp;
	tmp.push_back( ((GeometryMapEntry_Polygon*)o.get())->polygon_ );
	polygon_->merge( tmp );

#if 0	
	if(((GeometryMapEntry_Polygon*)o.get())->polygon_->merged_>5) {
		last_polygons_.insert(last_polygons_.begin(), ((GeometryMapEntry_Polygon*)o.get())->last_polygons_.begin(), ((GeometryMapEntry_Polygon*)o.get())->last_polygons_.end() );
		last_polygons_.push_back( ((GeometryMapEntry_Polygon*)o.get())->polygon_org_ );
	} else {	
		last_polygons_.push_back( ((GeometryMapEntry_Polygon*)o.get())->polygon_ );
	}
	
	//for(size_t i=0; i<((GeometryMapEntry_Polygon*)o.get())->last_polygons_.size(); i++)
	//	last_polygons_.push_back( Polygon::Ptr(new Polygon( *((GeometryMapEntry_Polygon*)o.get())->last_polygons_[i] )));
	//last_polygons_.push_back( Polygon::Ptr(new Polygon( *((GeometryMapEntry_Polygon*)o.get())->polygon_ )));
	
	const int history = 40;
	ROS_INFO("history %zu %d", last_polygons_.size(), ((GeometryMapEntry_Polygon*)o.get())->polygon_->merged_);
	if(last_polygons_.size()>history) { //history size
		if(rand()%2) {
			//descending order
			std::sort(last_polygons_.begin(), last_polygons_.end(), sort_polygons_by_area );
			//for(size_t i=0; i<last_polygons_.size(); i++) ROS_INFO("%f %f %d", last_polygons_[i]->computeArea3d(),std::pow(1.1f, -(float)last_polygons_[i]->frame_stamp_), last_polygons_[i]->frame_stamp_);
			//ROS_INFO("------");
			last_polygons_.erase(last_polygons_.begin()+(history/2), last_polygons_.end());
		
			/*polygon_org_->merge_settings_ = polygon_->merge_settings_;
			*polygon_ = *polygon_org_;
			polygon_->assignWeight ();
			
			//polygon_->merge222(last_polygons_);
			
			for(size_t i=0; i<last_polygons_.size(); i++) {
				std::vector<Polygon::Ptr> tmp;
				tmp.push_back( polygon_ );
				Polygon t = *last_polygons_[i];
				if(i) t.merge( tmp );
				*polygon_ = t;
			}
			
			{
				std::vector<Polygon::Ptr> tmp;
				tmp.push_back( polygon_ );
				Polygon t = *polygon_org_;
				t.merge( tmp );
				*polygon_ = t;
			}*/
			
			*polygon_ = *polygon_org_;
			for(size_t i=0; i<last_polygons_.size(); i++) {
				if( !polygon_->hasSimilarParametersWith( last_polygons_[i] ) || !polygon_->isIntersectedWith( last_polygons_[i] ) )
					continue;
				std::vector<Polygon::Ptr> tmp;
				tmp.push_back( last_polygons_[i] );
				polygon_->merge( tmp );
			}
		} else {
			last_polygons_.clear();
			std::vector<Polygon::Ptr> tmp;
			tmp.push_back( ((GeometryMapEntry_Polygon*)o.get())->polygon_ );
			polygon_->merge( tmp );
			*polygon_org_ = *polygon_;
		}
	}
	else {
		std::vector<Polygon::Ptr> tmp;
		tmp.push_back( ((GeometryMapEntry_Polygon*)o.get())->polygon_ );
		polygon_->merge( tmp );
	}
#endif
	
	return true;
}

void GeometryMapEntry_Polygon::setMergeSettings(const cob_3d_mapping::MergeConfig &limits) {
	polygon_->merge_settings_ = limits;
	polygon_->assignWeight ();
	
	//polygon_org_->merge_settings_ = limits;
	//polygon_org_->assignWeight ();
}

bool GeometryMapEntry_Polygon::needsCleaning(const int frame_counter, const bool persistent) const {
	return (polygon_->merged_ <= 1 || !persistent)
	 && (frame_counter - 5) > (int)polygon_->frame_stamp_ && (int)polygon_->frame_stamp_ > 1;
}

void GeometryMapEntry_Polygon::save(const std::string &path) const
{
  std::ofstream plane_file;
  plane_file.open ((path+".pl").c_str());
  plane_file << polygon_->normal_ (0) << " " << polygon_->normal_ (1) << " " << polygon_->normal_ (2) << " " << polygon_->d_;
  plane_file.close ();
  
  for (int i = 0; i < (int)polygon_->contours_.size (); i++)
  {
    pcl::PointCloud<pcl::PointXYZ> pc;
    for (int j = 0; j < (int)polygon_->contours_[i].size (); j++)
    {
      pcl::PointXYZ pt;
      pt.getVector3fMap () = Eigen::Vector3f (polygon_->contours_[i][j] (0), polygon_->contours_[i][j] (1), 0);
      pc.points.push_back (pt);
    }
    
	std::stringstream ss;
	ss<<path<<"_"<<i<<".pcd";
    pcl::io::savePCDFileASCII (ss.str(), pc);
  }
}

void GeometryMapEntry_Polygon::colorize()
{
	//coloring for polygon
	if (polygon_->color_[3] == 1.0f)
	  return;
	if (fabs (polygon_->normal_[2]) < 0.1) //plane is vertical
	{
	  polygon_->color_[0] = 0.75;
	  polygon_->color_[1] = 0.75;
	  polygon_->color_[2] = 0;
	  polygon_->color_[3] = 0.8;
	}
	else if (fabs (polygon_->normal_[0]) < 0.12 && fabs (polygon_->normal_[1]) < 0.12
		&& fabs (polygon_->normal_[2]) > 0.9) //plane is horizontal
	{
	  polygon_->color_[0] = 0;
	  polygon_->color_[1] = 0.5;
	  polygon_->color_[2] = 0;
	  polygon_->color_[3] = 0.8;
	}
	else
	{
	  polygon_->color_[0] = 0.75;
	  polygon_->color_[1] = 0.75;
	  polygon_->color_[2] = 0.75;
	  polygon_->color_[3] = 0.8;
	}
}

bool GeometryMapEntry_Cylinder::isMergeCandidate(const GeometryMapEntry::Ptr &o) const {
	if(!o || o->getType()!=getType()) return false;
	
	const Cylinder &c_map = *((GeometryMapEntry_Cylinder*)o.get())->cylinder_;
	Eigen::Vector3f connection = c_map.pose_.translation() - cylinder_->pose_.translation();

	// Test for geometrical attributes of cylinders
	if(fabs(c_map.sym_axis_.dot(cylinder_->sym_axis_)) > cylinder_->merge_settings_.angle_thresh && (fabs(c_map.r_ - cylinder_->r_) < (0.1 ))) //TODO: add param
	{
		// Test for spatial attributes of cylinders
		if( connection.norm() < (c_map.r_ + 0.1) || fabs(c_map.sym_axis_.dot(connection.normalized())) > cylinder_->merge_settings_.angle_thresh )
		{
		  if(cylinder_->isIntersectedWith( ((GeometryMapEntry_Cylinder*)o.get())->cylinder_ ))
			return true;
		}
	}
      
	return false;
}

bool GeometryMapEntry_Cylinder::merge(const GeometryMapEntry::Ptr &o) {
	if(!o || o->getType()!=getType()) return false;
	std::vector<Polygon::Ptr> tmp; tmp.push_back(((GeometryMapEntry_Cylinder*)o.get())->cylinder_);
	cylinder_->merge(tmp);
	return true;
}

void GeometryMapEntry_Cylinder::setMergeSettings(const cob_3d_mapping::MergeConfig &limits) {
	cylinder_->merge_settings_ = limits;
	cylinder_->assignWeight ();
}

void GeometryMapEntry_Cylinder::colorize()
{
	//coloring for cylinder
    if (cylinder_->color_[3] == 1.0f)
      return;
    if (fabs (cylinder_->normal_[0]) < 0.1 && fabs (cylinder_->normal_[1]) < 0.1) //cylinder is vertical
    {
      cylinder_->color_[0] = 0.5;
      cylinder_->color_[1] = 0.5;
      cylinder_->color_[2] = 0;
      cylinder_->color_[3] = 1;
    }
    else if (fabs (cylinder_->normal_[2]) < 0.12) //plane is horizontal
    {
      cylinder_->color_[0] = 0;
      cylinder_->color_[1] = 0.5;
      cylinder_->color_[2] = 0;
      cylinder_->color_[3] = 1;
    }
    else
    {
      cylinder_->color_[0] = 1;
      cylinder_->color_[1] = 1;
      cylinder_->color_[2] = 1;
      cylinder_->color_[3] = 1;
    }
}

bool GeometryMapEntry_Polygon::checkVisibility(const Eigen::Affine3f &T, const Eigen::Vector3f &camera_params, const Eigen::Vector3f &Z) {
	float area = 0;
	//ROS_INFO("z-dot: %f", Z.dot(T.inverse()*polygon_->computeCentroid()));
	if( Z.dot(T.inverse()*polygon_->computeCentroid())>0 ||1) {
		Polygon clip = *polygon_;
		std::vector<std::vector<Eigen::Vector3f> > contours_3d(1);
		
		Eigen::Affine3f pose_in_camera = T.inverse()*polygon_->pose_;
		Eigen::Vector3f tmp = camera_params;
		Eigen::Vector3f tr = pose_in_camera.rotation().inverse()*pose_in_camera.translation();
		contours_3d.back().push_back(T *( (tr(2)/(pose_in_camera.rotation().inverse()*tmp)(2))*tmp ));
		//std::cout<<tr(2)<<" "<<(pose_in_camera.rotation().inverse()*tmp)(2)<<" "<<((tr(2)/(pose_in_camera.rotation().inverse()*tmp)(2)))<<std::endl;
		tmp(1)*=-1;
		contours_3d.back().push_back(T *( (tr(2)/(pose_in_camera.rotation().inverse()*tmp)(2))*tmp ));
		//std::cout<<tr(2)<<" "<<(pose_in_camera.rotation().inverse()*tmp)(2)<<" "<<((tr(2)/(pose_in_camera.rotation().inverse()*tmp)(2)))<<std::endl;
		tmp(0)*=-1;
		contours_3d.back().push_back(T *( (tr(2)/(pose_in_camera.rotation().inverse()*tmp)(2))*tmp ));
		//std::cout<<tr(2)<<" "<<(pose_in_camera.rotation().inverse()*tmp)(2)<<" "<<((tr(2)/(pose_in_camera.rotation().inverse()*tmp)(2)))<<std::endl;
		tmp(1)*=-1;
		contours_3d.back().push_back(T *( (tr(2)/(pose_in_camera.rotation().inverse()*tmp)(2))*tmp ));
		//std::cout<<tr(2)<<" "<<(pose_in_camera.rotation().inverse()*tmp)(2)<<" "<<((tr(2)/(pose_in_camera.rotation().inverse()*tmp)(2)))<<std::endl;
		
		for(int i=0; i<4; i++) {
			if( (T.inverse()*contours_3d.back()[i])(2)<0 ) {
				if( (T.inverse()*contours_3d.back()[(i+1)%4])(2)>0 )
					contours_3d.back()[i] = 101*contours_3d.back()[(i+1)%4]-100*contours_3d.back()[i];
				else if( (T.inverse()*contours_3d.back()[(i+3)%4])(2)>0 )
					contours_3d.back()[i] = 101*contours_3d.back()[(i+3)%4]-100*contours_3d.back()[i];
			}
		}
		
	  /*std::cout<<(pose_in_camera).matrix()<<std::endl;
	  for(int i=0; i<4; i++) std::cout<<contours_3d[0][i].transpose()<<std::endl;
	  for(int i=0; i<4; i++) std::cout<<(polygon_->pose_.inverse()*contours_3d[0][i]).transpose()<<std::endl;
	  std::cout<<(T.inverse()*polygon_->computeCentroid()).transpose()<<std::endl;*/
	  
		clip.setContours3D(contours_3d);
		area = clip.getOverlap(*polygon_);
		//polygon_->setContours3D(contours_3d);
		
		/*if(area>=std::min(0.7*0.7, 0.7*polygon_->computeArea3d()))
			polygon_->color_[0] = 1;
		else
			polygon_->color_[0] = 0;*/
		//if(area>0.5*polygon_->computeArea3d())
	}
	
	/*if(area>=std::min(0.5*0.5, 0.3*polygon_->computeArea3d()))
		polygon_->color_[0] = 1;
	else
		polygon_->color_[0] = 0;*/
	
	//ROS_INFO("area: %f of %f", area, polygon_->computeArea3d());
	return area>=std::min(0.5*0.5, 0.3*polygon_->computeArea3d());
}
