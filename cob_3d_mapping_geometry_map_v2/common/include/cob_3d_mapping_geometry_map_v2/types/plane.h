#pragma once

#include "image.h"

//boost polygon, geometry
#include <boost/polygon/polygon.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

#include <libpolypartition/polypartition.h>

//ros
#include <eigen_conversions/eigen_msg.h>

//ros message types
#include <cob_3d_mapping_msgs/Plane.h>


namespace cob_3d_geometry_map {
	
class Plane_Point {
public:
    typedef float coordinate_type;
    typedef Eigen::Matrix<coordinate_type, 2,1> Vector2;
    
    Vector2 pos, tex;
    Eigen::Matrix3f var_;
    
    Plane_Point() {}
    Plane_Point(const cob_3d_mapping_msgs::Point2D &);
    
private:
	
};

struct Plane_Ring : std::deque<Plane_Point>
{
	Plane_Ring() {}
	
	Plane_Ring(const cob_3d_mapping_msgs::Polygon &poly) {
		insert(begin(), poly.points.begin(), poly.points.end());
	}
	
	Plane_Ring &from(const cob_3d_mapping_msgs::Polygon &poly) {
		clear();
		insert(begin(), poly.points.begin(), poly.points.end());
		return *this;
	}
};

/*
 * a polygon consists of:
 *  - points
 *  - texture ids
 */
class Plane_Polygon {
public:
	typedef std::vector<Plane_Ring> THoles;
	
private:
	Plane_Ring boundary_;
	THoles holes_;
	std::vector<Image::Ptr> imgs_;
	
public:
	Plane_Polygon(const std::vector<cob_3d_mapping_msgs::Polygon> &);

	const Plane_Ring &boundary() const {return boundary_;}
	Plane_Ring &boundary() {return boundary_;}
	
	const THoles &holes() const {return holes_;}
	THoles &holes() {return holes_;}
};

/*
 * a plane consists of:
 *  - polygons
 *  - a pose to captured view point
 *  - plane parameters (normal + offset)
 */
class Plane : public ObjectVolume {
	std::vector<Plane_Polygon> polygons_;
	Eigen::Vector3f offset_, normal_;
	
public:

	Plane(const ContextPtr &ctxt, const cob_3d_mapping_msgs::Plane &);
};

}

#include "impl/plane_boost_traits.h"

namespace cob_3d_geometry_map {
	#include "impl/plane.hpp"
}

