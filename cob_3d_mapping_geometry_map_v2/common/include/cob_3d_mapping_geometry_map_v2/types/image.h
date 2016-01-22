#pragma once

#include "object.h"
#include <string>


//! interfaces of cob_3d_geometry_map (v2)
namespace cob_3d_geometry_map {
	
class FileImage : public Object {
public:
	typedef boost::shared_ptr<FileImage> Ptr;
	
	enum TImgType {TYPE_COLOR, TYPE_DEPTH};
	
private:
	std::string filename_;
	int width_, height_;
	TImgType type_;
	float resolution_;	//<depth resolution of depth image
	
public:

	const std::string &filename() const {return filename_;}
};

typedef FileImage Image;

}
