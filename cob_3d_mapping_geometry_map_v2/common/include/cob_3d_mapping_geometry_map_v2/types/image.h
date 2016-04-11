#pragma once

#include "object.h"
#include <string>


//! interfaces of cob_3d_geometry_map (v2)
namespace cob_3d_geometry_map {
	
class Image : public Object {
public:
	typedef boost::shared_ptr<Image> Ptr;
	
	enum TImgType {TYPE_COLOR, TYPE_DEPTH};
	
private:
	int width_, height_;
	TImgType type_;
	float resolution_;	//<depth resolution of depth image

public:

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

	const std::string &filename() const {return filename_;}
	
	static Ptr get(const sensor_msgs::ImageConstPtr &img);	//< convert sensor msg to Image object
};

//typedef FileImage Image;

}

void SaveImageAsPPM( const sensor_msgs::ImageConstPtr& msg, const char* filename )
{
  if ( msg->encoding != "rgb8" )
  {
    return;  // Can only handle the rgb8 encoding
  }

  FILE* file = fopen( filename, "w" );

  fprintf( file, "P3\n" );
  fprintf( file, "%i %i\n", msg->width, msg->height );
  fprintf( file, "255\n" );

  for ( uint32_t y = 0; y < msg->height; y++ )
  {
    for ( uint32_t x = 0; x < msg->width; x++ )
    {
      // Get indices for the pixel components
      uint32_t redByteIdx = y*msg->step + 3*x;
      uint32_t greenByteIdx = redByteIdx + 1;
      uint32_t blueByteIdx = redByteIdx + 2;

      fprintf( file, "%i %i %i ", 
        msg->data[ redByteIdx ], 
        msg->data[ greenByteIdx ], 
        msg->data[ blueByteIdx ] );
    }
    fprintf( file, "\n" );
  }

  fclose( file );
}
