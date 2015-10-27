#pragma once

namespace cob_3d_geometry_map {
	
class Object {
public:
	typedef int TID;
	
	static TID generate_id();
	
private:
	TID id_;
public:

	Object(): id_(generate_id())
	{
	}
	
	inline TID id() const {return id_;}
};

}
