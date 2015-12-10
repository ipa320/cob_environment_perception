
// We can conveniently use macro's to register point and ring
//that's so cool
BOOST_GEOMETRY_REGISTER_POINT_2D(cob_3d_geometry_map::Plane_Point::Ptr, cob_3d_geometry_map::Plane_Point::coordinate_type, cs::cartesian, get()->pos(0), get()->pos(1))
BOOST_GEOMETRY_REGISTER_POINT_2D(cob_3d_geometry_map::Plane_Point, cob_3d_geometry_map::Plane_Point::coordinate_type, cs::cartesian, pos(0), pos(1))
BOOST_GEOMETRY_REGISTER_RING(cob_3d_geometry_map::Plane_Ring)



// There is currently no registration macro for polygons
// and besides that a boost::array<T,N> in a macro would
// be very specific, so we show it "by hand":
namespace boost { namespace geometry { namespace traits
{

template<> struct tag<cob_3d_geometry_map::Plane_Polygon> { typedef polygon_tag type; };
template<> struct ring_const_type<cob_3d_geometry_map::Plane_Polygon> { typedef cob_3d_geometry_map::Plane_Ring const& type; };
template<> struct ring_mutable_type<cob_3d_geometry_map::Plane_Polygon> { typedef cob_3d_geometry_map::Plane_Ring& type; };

template<> struct interior_const_type<cob_3d_geometry_map::Plane_Polygon>
{
    typedef cob_3d_geometry_map::Plane_Polygon::THoles const& type;
};

template<> struct interior_mutable_type<cob_3d_geometry_map::Plane_Polygon>
{
    typedef cob_3d_geometry_map::Plane_Polygon::THoles& type;
};

template<> struct exterior_ring<cob_3d_geometry_map::Plane_Polygon>
{
    static cob_3d_geometry_map::Plane_Ring& get(cob_3d_geometry_map::Plane_Polygon& p)
    {
        return p.boundary();
    }

    static cob_3d_geometry_map::Plane_Ring const& get(cob_3d_geometry_map::Plane_Polygon const& p)
    {
        return p.boundary();
    }
};

template<> struct interior_rings<cob_3d_geometry_map::Plane_Polygon>
{
    typedef cob_3d_geometry_map::Plane_Polygon::THoles holes_type;

    static holes_type& get(cob_3d_geometry_map::Plane_Polygon& p)
    {
        return p.holes();
    }

    static holes_type const& get(cob_3d_geometry_map::Plane_Polygon const& p)
    {
        return p.holes();
    }
};

template<typename coordinate_type> struct intersection_interpolation<cob_3d_geometry_map::Plane_Point::Ptr, coordinate_type>
{	
	typedef cob_3d_geometry_map::Plane_Point::Ptr Point;
	
	static void interpolate(Point &point, const Point &p1, const Point &p2, const coordinate_type relation)
	{
		point->var = relation*p2->var + (1-relation)*p1->var;
	}
};

template<typename coordinate_type> struct intersection_interpolation<cob_3d_geometry_map::Plane_Point, coordinate_type>
{	
	typedef cob_3d_geometry_map::Plane_Point Point;
	
	static void interpolate(Point &point, const Point &p1, const Point &p2, const coordinate_type relation)
	{
		point.var = relation*p2.var + (1-relation)*p1.var;
	}
};

}}} // namespace boost::geometry::traits
