#pragma once

#include <Eigen/Core>
#include <cob_3d_visualization/simple_marker.h>


namespace cob_3d_geometry_map {
	
	namespace Visualization {
		
		class Marker;
		
		class Object {
		protected:
			Eigen::Vector4f color_;
			std::string name_;
			
			void _serialize(cob_3d_visualization::RvizMarker &scene);
			
		public:
			typedef boost::shared_ptr<Object> Ptr;
			
			Object(const std::string &name) : color_(1.f, 1.f, 1.f, 1.f), name_(name) {}
			
			const Eigen::Vector4f &color() const {return color_;}
			Eigen::Vector4f &color() {return color_;}
			
			const std::string &name() const {return name_;}
			std::string &name() {return name_;}
			
			virtual void serialize(Marker &stream) = 0;
		};
		
		class Mesh : public Object {
			struct Tri {
				int ind[3];
			};
			std::vector<Eigen::Vector3f> verts_;
			std::vector<Tri> indices_;
			
		public:
			typedef boost::shared_ptr<Mesh> Ptr;
			
			Mesh(const std::string &name) : Object(name)
			{}
			
			virtual void serialize(Marker &stream);
			
			void add(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3) {
				Tri t;
				t.ind[0] = (int)verts_.size();
				t.ind[1] = t.ind[0]+1;
				t.ind[2] = t.ind[0]+2;
				indices_.push_back(t);
				verts_.push_back(p1); verts_.push_back(p2); verts_.push_back(p3);
			}
		};
		
		class Sphere : public Object {
			float r_;
			Eigen::Vector3f pos_;
			
		public:
			Sphere(const std::string &name, const Eigen::Vector3f &p, const float r) :
				Object(name), r_(r), pos_(p)
			{}
			
			virtual void serialize(Marker &stream);
		};
		
	}
	
}
