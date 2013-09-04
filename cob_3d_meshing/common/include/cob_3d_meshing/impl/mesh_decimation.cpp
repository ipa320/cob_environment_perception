
//== INCLUDES ==================================================================

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>





namespace OpenMesh {
namespace Decimater {


template<typename PointT, typename NormalT, typename LabelT>
void MeshSimplification<PointT,NormalT,LabelT>::initializeMesh(
  const PointCloudConstPtr& input,
  const LabelCloudConstPtr& labels,
  const NormalCloudConstPtr& normals,
  const std::map<int,Eigen::Vector4f>& params)
{
  int rows = input->height - 1; // last row
  int cols = input->width - 1; // last column
  int row_offset;
  // [h]orizontal, [v]ertical, [l]eft, [r]ight edge check
  std::vector<std::vector<bool> > h(rows+1, std::vector<bool>(cols,true));
  std::vector<std::vector<bool> > v(rows, std::vector<bool>(cols+1,true));
  std::vector<std::vector<bool> > l(rows, std::vector<bool>(cols,true));
  std::vector<std::vector<bool> > r(rows, std::vector<bool>(cols,true));
  std::vector<std::vector<Mesh::VertexHandle> >
    vh(rows+1, std::vector<Mesh::VertexHandle>(cols+1)); // vertex handles

  /*
   * +--+--+   p00  h00  p01  h01  p02
   * |  |  |   v00 lr00  v01 lr01  v02
   * +--+--+   p10  h10  p11  h11  p12
   * |  |  |   v10 lr10  v11 lr11  v12
   * +--+--+   p20  h20  p21  h21  p22
   */

  // corners
  h.front().front() = v.front().front() = r.front().front() = false;
  h.front().back()  = v.front().back()  = l.front().back()  = false;
  h.back().front() = v.back().front() = l.back().front() = false;
  h.back().back()  = v.back().back()  = r.back().back()  = false;

  // first and last row
  for(int x = 1; x<cols; ++x)
  {
    h.front()[x-1] = false;
    h.front()[x  ] = false;
    v.front()[x  ] = false;
    l.front()[x-1] = false;
    r.front()[x  ] = false;
    h.back ()[x-1] = false;
    h.back ()[x  ] = false;
    v.back ()[x  ] = false;
    r.back ()[x-1] = false;
    l.back ()[x  ] = false;
  }

  for(int y = 1; y<rows; ++y)
  {
    // left column and right column
    h[y  ].front() = false;
    v[y-1].front() = false;
    v[y  ].front() = false;
    l[y-1].front() = false;
    r[y  ].front() = false;
    h[y  ].back()  = false;
    v[y-1].back()  = false;
    v[y  ].back()  = false;
    r[y-1].back()  = false;
    l[y  ].back()  = false;

    row_offset = y*(cols+1);
    // iterate remaining
    for(int x=1; x<cols; ++x)
    {
      const PointT* p = &(*input)[row_offset+x];
      if( p->z != p->z )
      {
        v[y-1][x  ] = false;
        v[y  ][x  ] = false;
        h[y  ][x-1] = false;
        h[y  ][x  ] = false;
        l[y-1][x  ] = false;
        l[y  ][x-1] = false;
        r[y-1][x-1] = false;
        r[y  ][x  ] = false;
      }
      else
      {
        vh[y][x] = mesh_->add_vertex(Mesh::Point(p->x, p->y, p->z));
        /*Eigen::Vector4f nd;
        if(params.find((*labels)[row_offset+x].label) != params.end())
          nd = params.find((*labels)[row_offset+x].label)->second;
        else
        {
          Eigen::Vector3f n = (*normals)[row_offset+x].getNormalVector3fMap();
          float d = p->getVector3fMap().dot(n);
          nd = Eigen::Vector4f(n(0),n(1),n(2),d);
        }

        mesh_->property(p_normals_,vh[y][x]) = PropNormalT(nd(0), nd(1), nd(2), nd(3));
        mesh_->property(p_labels_, vh[y][x]) = (*labels)[row_offset+x].label;*/
      }
    }
  }

  // iterate h and v to check if edge is valid
  typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
    ::const_iterator pii = input->points.begin();
  typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
    ::const_iterator pij = pii + 1; // right
  typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
    ::const_iterator pji = pii + 1 + cols; // below
  typename std::vector<PointT, Eigen::aligned_allocator_indirection<PointT> >
    ::const_iterator pjj = pji + 1; // below right

  for(int y=0; y<rows; ++y)
  {
    for(int x=0; x<cols; ++x)
    {
      // check horizontal and vertical
      if (h[y][x])
        h[y][x] = isNeighbor(pii->getVector3fMap(), pij->getVector3fMap());
      if (v[y][x])
        v[y][x] = isNeighbor(pii->getVector3fMap(), pji->getVector3fMap());

      // check diagonal
      unsigned char status = (l[y][x] << 1) | r[y][x];
      switch(status)
      {
      case 0b00:
        break;
      case 0b01:
        r[y][x] = isNeighbor(pii->getVector3fMap(), pjj->getVector3fMap());
        break;
      case 0b10:
        l[y][x] = isNeighbor(pij->getVector3fMap(), pji->getVector3fMap());
        break;
      case 0b11:
        if( (pij->z - pji->z) > (pii->z - pjj->z) )
        {
          r[y][x] = false;
          l[y][x] = isNeighbor(pij->getVector3fMap(), pji->getVector3fMap());
        }
        else
        {
          l[y][x] = false;
          r[y][x] = isNeighbor(pii->getVector3fMap(), pjj->getVector3fMap());
        }
        break;
      }
      ++pii; ++pij; ++pji; ++pjj;
    }
    // skip the last column
    // note that in the very last iteration, pjj points beyond end()
    ++pii; ++pij; ++pji; ++pjj;
  }

  for(int y=0; y<rows; ++y)
  {
    for(int x=0; x<cols; ++x)
    {
      /* ii-ji-ij | ij-ji-jj | ii-jj-ij | ii-ji-jj
       *  +-+     |    +     |  +-+     |  +
       *  |/      |   /|     |   \|     |  |\
       *  +       |  +-+     |    +     |  +-+     */
      if(l[y][x])
      {
        if (h[y  ][x] && v[y][x  ])
          mesh_->add_face(vh[y][x  ], vh[y+1][x], vh[y  ][x+1]);
        if (h[y+1][x] && v[y][x+1])
          mesh_->add_face(vh[y][x+1], vh[y+1][x], vh[y+1][x+1]);
      }
      else if (r[y][x])
      {
        if (h[y][x] && v[y][x+1])
          mesh_->add_face(vh[y][x], vh[y+1][x+1], vh[y][x+1]);
        if (v[y][x] && h[y+1][x])
          mesh_->add_face(vh[y][x], vh[y+1][x], vh[y+1][x+1]);
      }
    }
  }
}

template<typename PointT, typename NormalT, typename LabelT>
void MeshSimplification<PointT,NormalT,LabelT>::decimate()
{
  dec_->initialize();
  dec_->module(mod_).set_binary(false);
  dec_->decimate_to(n_vertices_);
  mesh_->garbage_collection();
  std::cout << "Vertices: " << dec_->mesh().n_vertices() << "\n"
            << "Faces: " << dec_->mesh().n_faces() << "\n"
            << "Edges: " << dec_->mesh().n_edges() << std::endl;
}

