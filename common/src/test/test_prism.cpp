/*
 * test_prism.cpp
 *
 *  Created on: 02.09.2011
 *      Author: goa
 */

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/common/centroid.h"
#include "pcl/common/eigen.h"
#include <pcl/pcl_base.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

typedef pcl::PointXYZ Point;



bool
isXYPointIn2DXYPolygon (const Point &point, const pcl::PointCloud<Point> &polygon)
{
  bool in_poly = false;
  double x1, x2, y1, y2;

  int nr_poly_points = polygon.points.size ();
  double xold = polygon.points[nr_poly_points - 1].x;
  double yold = polygon.points[nr_poly_points - 1].y;
  for (int i = 0; i < nr_poly_points; i++)
  {
    double xnew = polygon.points[i].x;
    double ynew = polygon.points[i].y;
    if (xnew > xold)
    {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    }
    else
    {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }

    if ( (xnew < point.x) == (point.x <= xold) && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1) )
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }
  return (in_poly);
}

bool
isPointIn2DPolygon (const Point &point, const pcl::PointCloud<Point> &polygon)
{
  // Compute the plane coefficients
  Eigen::Vector4f model_coefficients;
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;

  // Estimate the XYZ centroid
  pcl::compute3DCentroid (polygon, xyz_centroid);

  // Compute the 3x3 covariance matrix
  computeCovarianceMatrix (polygon, xyz_centroid, covariance_matrix);

  // Compute the model coefficients
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

  model_coefficients[0] = eigen_vectors (0, 0);
  model_coefficients[1] = eigen_vectors (1, 0);
  model_coefficients[2] = eigen_vectors (2, 0);
  model_coefficients[3] = 0;

  // Hessian form (D = nc . p_plane (centroid here) + p)
  model_coefficients[3] = -1 * model_coefficients.dot (xyz_centroid);

  float distance_to_plane = model_coefficients[0] * point.x +
                            model_coefficients[1] * point.y +
                            model_coefficients[2] * point.z +
                            model_coefficients[3];
  Point ppoint;
  // Calculate the projection of the point on the plane
  ppoint.x = point.x - distance_to_plane * model_coefficients[0];
  ppoint.y = point.y - distance_to_plane * model_coefficients[1];
  ppoint.z = point.z - distance_to_plane * model_coefficients[2];

  // Create a X-Y projected representation for within bounds polygonal checking
  int k0, k1, k2;
  // Determine the best plane to project points onto
  k0 = (fabs (model_coefficients[0] ) > fabs (model_coefficients[1])) ? 0  : 1;
  k0 = (fabs (model_coefficients[k0]) > fabs (model_coefficients[2])) ? k0 : 2;
  k1 = (k0 + 1) % 3;
  k2 = (k0 + 2) % 3;
  // Project the convex hull
  pcl::PointCloud<Point> xy_polygon;
  xy_polygon.points.resize (polygon.points.size ());
  for (size_t i = 0; i < polygon.points.size (); ++i)
  {
    Eigen::Vector4f pt (polygon.points[i].x, polygon.points[i].y, polygon.points[i].z, 0);
    xy_polygon.points[i].x = pt[k1];
    xy_polygon.points[i].y = pt[k2];
    xy_polygon.points[i].z = 0;
  }
  Point xy_point;
  xy_point.z = 0;
  Eigen::Vector4f pt (ppoint.x, ppoint.y, ppoint.z, 0);
  xy_point.x = pt[k1];
  xy_point.y = pt[k2];

  return (isXYPointIn2DXYPolygon (xy_point, xy_polygon));
}

void
segment (pcl::PointCloud<Point>::Ptr& input_, pcl::PointCloud<Point>::Ptr& planar_hull_, std::vector<int>& indices_, pcl::PointIndices &output)
{
  int min_pts_hull_ = 3;
  output.header = input_->header;

  /*if (!initCompute ())
  {
    output.indices.clear ();
    return;
  }*/

  if ((int)planar_hull_->points.size () < min_pts_hull_)
  {
    //PCL_ERROR ("[pcl::%s::segment] Not enough points (%lu) in the hull!\n", getClassName ().c_str (), (unsigned long)planar_hull_->points.size ());
    output.indices.clear ();
    return;
  }

  // Compute the plane coefficients
  Eigen::Vector4f model_coefficients;
  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
  Eigen::Vector4f xyz_centroid;

  // Estimate the XYZ centroid
  compute3DCentroid (*planar_hull_, xyz_centroid);

  // Compute the 3x3 covariance matrix
  computeCovarianceMatrix (*planar_hull_, xyz_centroid, covariance_matrix);

  // Compute the model coefficients
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

  model_coefficients[0] = eigen_vectors (0, 0);
  model_coefficients[1] = eigen_vectors (1, 0);
  model_coefficients[2] = eigen_vectors (2, 0);
  model_coefficients[3] = 0;

  // Hessian form (D = nc . p_plane (centroid here) + p)
  model_coefficients[3] = -1 * model_coefficients.dot (xyz_centroid);

  // Need to flip the plane normal towards the viewpoint
  double vpx_=0, vpy_=0, vpz_=0;
  Eigen::Vector4f vp (vpx_, vpy_, vpz_, 0);
  // See if we need to flip any plane normals
  vp -= planar_hull_->points[0].getVector4fMap ();
  vp[3] = 0;
  // Dot product between the (viewpoint - point) and the plane normal
  float cos_theta = vp.dot (model_coefficients);
  // Flip the plane normal
  if (cos_theta < 0)
  {
    model_coefficients *= -1;
    model_coefficients[3] = 0;
    // Hessian form (D = nc . p_plane (centroid here) + p)
    model_coefficients[3] = -1 * (model_coefficients.dot (planar_hull_->points[0].getVector4fMap ()));
  }

  // Project all points
  pcl::PointCloud<Point> projected_points;
  pcl::SampleConsensusModelPlane<Point> sacmodel (input_);
  sacmodel.projectPoints (indices_, model_coefficients, projected_points, false);

  // Create a X-Y projected representation for within bounds polygonal checking
  int k0, k1, k2;
  // Determine the best plane to project points onto
  k0 = (fabs (model_coefficients[0] ) > fabs (model_coefficients[1])) ? 0  : 1;
  k0 = (fabs (model_coefficients[k0]) > fabs (model_coefficients[2])) ? k0 : 2;
  k1 = (k0 + 1) % 3;
  k2 = (k0 + 2) % 3;
  // Project the convex hull
  pcl::PointCloud<Point> polygon;
  polygon.points.resize (planar_hull_->points.size ());
  for (size_t i = 0; i < planar_hull_->points.size (); ++i)
  {
    Eigen::Vector4f pt (planar_hull_->points[i].x, planar_hull_->points[i].y, planar_hull_->points[i].z, 0);
    polygon.points[i].x = pt[k1];
    polygon.points[i].y = pt[k2];
    polygon.points[i].z = 0;
  }

  Point pt_xy;
  pt_xy.z = 0;

  double height_limit_min_ = -0.1, height_limit_max_ = 0.1;
  output.indices.resize (indices_.size ());
  int l = 0;
  for (size_t i = 0; i < projected_points.points.size (); ++i)
  {
    // Check the distance to the user imposed limits from the table planar model
    double distance = pointToPlaneDistanceSigned (input_->points[(indices_)[i]], model_coefficients);
    if (distance < height_limit_min_ || distance > height_limit_max_)
      continue;

    // Check what points are inside the hull
    Eigen::Vector4f pt (projected_points.points[i].x,
                         projected_points.points[i].y,
                         projected_points.points[i].z, 0);
    pt_xy.x = pt[k1];
    pt_xy.y = pt[k2];

    if (!isXYPointIn2DXYPolygon (pt_xy, polygon))
      continue;

    output.indices[l++] = (indices_)[i];
  }
  output.indices.resize (l);

  //deinitCompute ();
}



int main(int argc, char** argv)
{

    pcl::PointCloud<Point> pc;
    pcl::io::loadPCDFile("/home/goa/tmp/table_objects/pc.pcd", pc);
    pcl::PointCloud<Point> hull;
    pcl::io::loadPCDFile("/home/goa/tmp/table_objects/hull.pcd", hull);
    pcl::PointCloud<Point>::Ptr pc_ptr = pc.makeShared();
    pcl::PointCloud<Point>::Ptr hull_ptr = hull.makeShared();
    pcl::PointIndices output;

    pcl::ExtractPolygonalPrismData<Point> prism;
    prism.setHeightLimits(-0.1, 0.1);
    // ---[ Get the objects on top of the table
    pcl::PointIndices roi_indices;
    prism.setInputCloud(pc_ptr);
    prism.setInputPlanarHull(hull_ptr);
    prism.segment(output);

    /*std::vector<int> indices;
    for(unsigned int i=0; i<pc.size(); i++)
      indices.push_back(i);
    segment (pc_ptr, hull_ptr, indices, output);*/
    std::cout << output.indices.size() << std::endl;
    pcl::ExtractIndices<Point> extract_roi;
    pcl::PointCloud<Point> pc_roi;
    extract_roi.setInputCloud (pc_ptr);
    extract_roi.setIndices (boost::make_shared<const pcl::PointIndices> (output));
    extract_roi.filter (pc_roi);
    pcl::io::savePCDFile("/home/goa/tmp/table_objects/roi.pcd", pc_roi);
    /*Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    pc.points.push_back(p);
    p.x = 1;
    pc.points.push_back(p);
    p.z = 1;
    pc.points.push_back(p);
    p.x = 0;
    pc.points.push_back(p);
    p.x = 0.5;
    p.z = 0.5;
    bool isIn = isPointIn2DPolygon (p, pc);
    std::cout << isIn << std::endl;*/

    std::cout << "done" << std::endl;
    return 0;
}
