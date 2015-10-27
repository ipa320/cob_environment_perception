#pragma once

#include "object.h"
#include <boost/polygon/polygon.hpp>


namespace cob_3d_geometry_map {
	
class Point {
public:
    typedef float coordinate_type;
    
    coordinate_type x, y;
    coordinate_type tex_x[2], tex_y[2];
    Eigen::Matrix3f var_;
    
private:
	
};

}

//First we register it as a point with boost polygon
namespace boost { namespace polygon {
    template <>
    struct geometry_concept<cob_3d_geometry_map::Point> { typedef point_concept type; };
 
    
    //Then we specialize the gtl point traits for our point type
    template <>
    struct point_traits<cob_3d_geometry_map::Point> {
        typedef Point::coordinate_type coordinate_type;
    
        static inline coordinate_type get(const cob_3d_geometry_map::Point& point, orientation_2d orient) {
            if(orient == HORIZONTAL)
                return point.x;
            return point.y;
        }
    };
    
    template <>
    struct point_mutable_traits<cob_3d_geometry_map::Point> {
        typedef Point::coordinate_type coordinate_type;

        static inline void set(cob_3d_geometry_map::Point& point, orientation_2d orient, coordinate_type value) {
            if(orient == HORIZONTAL)
                point.x = value;
            else
            point.y = value;
        }
        static inline cob_3d_geometry_map::Point construct(coordinate_type x_value, coordinate_type y_value) {
            cob_3d_geometry_map::Point retval;
            retval.x = x_value;
            retval.y = y_value; 
            return retval;
        }
    };
} }

namespace cob_3d_geometry_map {
/*
 * a polygon consists of:
 *  - points
 *  - texture ids
 */
class Polygon {
	typedef std::list<CPoint> Polygon;
	
	Polygon poly_;
	Image::Ptr img_[2];
};

/*
 * a plane consists of:
 *  - polygons
 *  - a pose to captured view point
 *  - plane parameters (normal + offset)
 */
class Plane : public Object {
	std::vector<Polygon> polygons_;
	Eigen::Vector3f offset_, normal_;
};

}

