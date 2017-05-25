#pragma once

#include "object.h"
#include <string>
#include <sensor_msgs/Image.h>


//! interfaces of cob_3d_geometry_map (v2)
namespace cob_3d_geometry_map {
	
class Image : public Object {
public:
	typedef boost::shared_ptr<Image> Ptr;
	
	enum TImgType {TYPE_NONE, TYPE_COLOR, TYPE_DEPTH};
	
private:
	int width_, height_;
	TImgType type_;
	float resolution_;	//<depth resolution of depth image

public:

	Image(const ContextPtr &ctxt) : Object(ctxt), width_(0), height_(0), type_(TYPE_NONE), resolution_(0) {}
	Image(const ContextPtr &ctxt, const TImgType &type) : Object(ctxt), width_(0), height_(0), type_(type), resolution_(0) {}

	//getters
	inline int width() const {return width_;}
	inline int height() const {return height_;}
	inline TImgType type() const {return type_;}	//< type of image (color, depth, ...)
	inline float resolution() const {return resolution_;}
};

	
class ImageFile : public Image {
public:
	typedef boost::shared_ptr<ImageFile> Ptr;
	
private:
	std::string filename_;
	
public:

	ImageFile(const ContextPtr &ctxt);
	ImageFile(const ContextPtr &ctxt, const TImgType &type);

	const std::string &filename() const {return filename_;}
	
	static Ptr get(const ContextPtr &ctxt, const sensor_msgs::ImageConstPtr &img, const TImgType &type);	//< convert sensor msg to Image object
};

//typedef FileImage Image;

}
