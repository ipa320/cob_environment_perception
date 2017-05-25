#include <cob_3d_mapping_geometry_map_v2/types/image.h>
#include <boost/filesystem.hpp>

using namespace cob_3d_geometry_map;


ImageFile::ImageFile(const ContextPtr &ctxt) :
 Image(ctxt), filename_(boost::filesystem::unique_path().native())
{}

ImageFile::ImageFile(const ContextPtr &ctxt, const TImgType &type) :
 Image(ctxt, type), filename_(boost::filesystem::unique_path().native())
{}

ImageFile::Ptr ImageFile::get(const ContextPtr &ctxt, const sensor_msgs::ImageConstPtr &msg, const TImgType &type)
{
  ImageFile::Ptr img;
  
  if ( msg->encoding != "rgb8" )
  {
    return img;  // Can only handle the rgb8 encoding
  }

  img.reset(new ImageFile(ctxt, type));
  
  FILE* file = fopen( img->filename().c_str(), "w" );

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
  
  return img;
}
