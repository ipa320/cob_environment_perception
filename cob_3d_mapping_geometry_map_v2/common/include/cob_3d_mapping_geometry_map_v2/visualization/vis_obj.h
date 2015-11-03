#pragma once


namespace cob_3d_geometry_map {
	
	namespace Visualization {
		
		class Marker;
		
		class Object {
		public:
			typedef boost::shared_ptr<Object> Ptr;
			
			virtual void serialize(Marker &stream) = 0;
		};
		
		class Mesh : public Object {
			struct Tri {
				int ind[3];
			};
			std::vector<Eigen::Vector3f> verts_;
			std::vector<Tri> indices_;
			
			virtual void serialize(Marker &stream);
		};
		
		class Sphere : public Object {
			float r_;
			Eigen::Vector3f pos_;
			
		public:
			Sphere(const Eigen::Vector3f &p, const float r) :
				r_(r), pos_(p)
			{}
			
			virtual void serialize(Marker &stream);
		};
		
	}
	
}
