#pragma once

namespace cob_3d_geometry_map {
	
class VolumeCut_Snapper : public VolumeCut {
	Plane plane_;
	nuklei::kernel::se3 inv_;
public:
	VolumeCut_Snapper(const Plane &plane) : plane_(plane) {
		inv_ = plane.pose().inverseTransformation();
	}
	
	virtual bool cut(const nuklei::kernel::r3 &pt) const {
		nuklei::kernel::r3 pt_in_plane = pt.transformedWith(inv_);
		std::cout<<"pt in plane "<<cast(pt_in_plane.loc_).transpose()<<" +- "<<pt.loc_h_<<"+"<<plane_.bb_in_pose().exteriorDistance(cast(pt_in_plane.loc_))<<std::endl;
		if(plane_.bb_in_pose().exteriorDistance(cast(pt_in_plane.loc_)) <= pt.loc_h_) {
			//inside?
			return true;//plane_.polygon_distance(Plane_Point(pt_in_plane.loc_.X(), pt_in_plane.loc_.Y())) <= pt.loc_h_;
		}
		return false;
	}
	
	virtual nuklei::kernel::r3 cutting_point(const nuklei::kernel::r3 &pt) const {
		nuklei::kernel::r3 r = pt.transformedWith(inv_);
		r.loc_.Z() = 0;
		r = r.transformedWith(plane_.pose());
		r.loc_h_ = pt.loc_h_;
		std::cout<<"cutting point "<<cast(pt.loc_).transpose()<<" --- "<<cast(r.loc_).transpose()<<std::endl;
		return r;
	}
};


class Intersection_Volume_Viewport : public Intersection_Volume {
	Eigen::Vector3f origin_;
	Eigen::Matrix3f proj_;
	ObjectVolume::TBB bb_;
	float far_clipping_;
	
	inline bool inside(const Eigen::Vector3f &pt) const {
		if(pt(2)<=0 || pt(2)>=far_clipping_) return false;
		Eigen::Vector2f tmp = (proj_*pt).head<2>();
		return std::min(tmp(0),tmp(1))>=0 && std::max(tmp(0),tmp(1))<=1;
	}
	
	inline bool intersects(const Eigen::Vector3f &pti, const Eigen::Vector3f &pta) const {
		if((pti(2)<=0&&pta(2)<=0) || (pti(2)>=far_clipping_&&pta(2)>=far_clipping_)) {
			std::cout<<"Intersection_Volume_Viewport: int1"<<std::endl;
			return false;
		}
		
		Eigen::Vector2f ti = (proj_*pti).head<2>();
		Eigen::Vector2f ta = (proj_*pta).head<2>();
		
		assert(ti(0)<=ta(0));
		assert(ti(1)<=ta(1));
		
		if(!(ti(0)<=1 && ta(0)>=0 && ti(1)<=1 && ta(1)>=0)) {
			std::cout<<"Intersection_Volume_Viewport: int2\n"<<(proj_*pti).transpose()<<" -- "<<(proj_*pta).transpose()<<std::endl;
		}
		
		return ti(0)<=1 && ta(0)>=0 && ti(1)<=1 && ta(1)>=0;
	}
	
	inline bool contains(const Eigen::Vector3f &pti, const Eigen::Vector3f &pta) const {
		if(pti(2)<=0 || pta(2)>=far_clipping_) return false;
		
		Eigen::Vector2f ti = (proj_*pti).head<2>();
		Eigen::Vector2f ta = (proj_*pta).head<2>();
		
		return ti(0)>=0 && ta(0)<=1 && ti(1)>=0 && ta(1)<=1;
	}
	
public:
	Intersection_Volume_Viewport(
		const Eigen::Vector3f &origin,
		const Eigen::Matrix3f &proj,
		const Eigen::Matrix3f &proj_inv,
		const float far_clipping
		) :
		origin_(origin), proj_(proj), far_clipping_(far_clipping)
	{
		bb_.extend(origin);
		bb_.extend(origin+far_clipping*proj_inv*Eigen::Vector3f::UnitZ());
		bb_.extend(origin+far_clipping*proj_inv*Eigen::Vector3f(1,1,1));
	}
	
	virtual ObjectVolume::TBB bb() const {
		return bb_;
	}
	
	virtual bool intersects(const Object *obj) const {
		/*const Plane *plane = dynamic_cast<const Plane*>(obj);
		if(plane) {
		}*/
		
		const ObjectVolume *volume = dynamic_cast<const ObjectVolume*>(obj);
		if(volume) {
			ObjectVolume::TBB bb = volume->bb_in_context();
			return intersects(bb.min(), bb.max());
		}
		
		const Object3D *point = dynamic_cast<const Object3D*>(obj);
		if(point)
			return inside(cast(point->pose().loc_));
		
		return false;
	}
	
	virtual bool contains(const Object *obj) const {
		const ObjectVolume *volume = dynamic_cast<const ObjectVolume*>(obj);
		if(volume) {
			ObjectVolume::TBB bb = volume->bb_in_context();
			return contains(bb.min(), bb.max());
		}
		
		const Object3D *point = dynamic_cast<const Object3D*>(obj);
		if(point)
			return inside(cast(point->pose().loc_));
		
		return false;
	}
};


}
