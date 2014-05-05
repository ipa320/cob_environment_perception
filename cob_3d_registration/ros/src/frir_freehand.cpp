/*
 * frir_freehand.cpp
 *
 *  Created on: 25.06.2012
 *      Author: josh
 */




#define _FN1_ "/home/josh/tmp/1305031229.729077.png.img.pcd"
#define _FN2_ "/home/josh/tmp/1305031229.764869.png.img.pcd"

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/pca.h>
#include <pcl/registration/registration.h>
#include <boost/timer.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/distances.h>
#include <pcl/registration/icp.h>

#include "pcl/registration/registration.h"
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_registration.h"

#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"

#include <pcl/common/transformation_from_correspondences.h>
#include <tf/transform_broadcaster.h>

//#define USE_LM
#define USE_TF 1

#define DEBUG_SWITCH_ 0

#ifdef USE_LM
#include <pcl/registration/transformation_estimation_lm.h>
#endif

//#define RECONF_
#define MAP_
#define RGB_
#define WHOLE_DEPTH_
#define DIS_TREE_
#define OPT_SIZE_ 2

#define USE_SMART_GRID_ 0
#define TESTING_ 0

bool g_step = false;



class PrecisionStopWatch {
  struct timeval precisionClock;

public:
  PrecisionStopWatch() {
    gettimeofday(&precisionClock, NULL);
  };

  void precisionStart() {
    gettimeofday(&precisionClock, NULL);
  };

  double precisionStop() {
    struct timeval precisionClockEnd;
    gettimeofday(&precisionClockEnd, NULL);
    double d= ((double)precisionClockEnd.tv_sec + 1.0e-6 * (double)precisionClockEnd.tv_usec) - ((double)precisionClock.tv_sec + 1.0e-6 * (double)precisionClock.tv_usec);
    precisionStart();
    return d;
  };
};


std_msgs::Header header_;
ros::Publisher marker_pub_;
ros::Publisher point_cloud_pub_,map_pub_;              //publisher for map

inline bool makes_sense(const float f) {return f>0.2f && f<6.f;}

#define MY_POINT pcl::PointXYZRGB

struct SORT_S {
  float dis;
  int ind;

  bool operator() (const SORT_S &i, const SORT_S &j) const { return (i.dis<j.dis);}
};

struct SORT_S2 {
  float dis;
  int ind;

  bool operator() (const SORT_S2 &i, const SORT_S2 &j) const { return (i.dis<j.dis);}
};

struct COR_S {
  int ind_o, ind_n;
  float dis;
};

inline Eigen::Vector3f Null3f() {
  Eigen::Vector3f r;
  r(0)=r(1)=r(2)=0;
  return r;
}

inline void getDis(const Eigen::Vector3f &C, const Eigen::Vector3f &S, float &dT, float &dA) {
  dT = std::abs(C.norm()-S.norm());
  dA = acosf(C.dot(S)/(C.norm()*S.norm()));
}

template <typename Point>
Eigen::Vector3f getMidO(const pcl::PointCloud<Point> &pc, const std::vector<int> &weight, const std::vector<COR_S> &cors, const bool sm=true, const float max_dis=10000000) {
  Eigen::Vector3f m = Null3f();
  float n=0;

  for(int i=0; i<cors.size(); i++) {
    Eigen::Vector3f v;

    //if(weight[i]<1)
    //  continue;

    float w=1/(float)(weight[cors[i].ind_o]);
    v=pc[cors[i].ind_o].getVector3fMap();

    if(sm && v.squaredNorm()<max_dis) {
      m+=v*w;
      n+=w;
    }
    else if(!sm && v.squaredNorm()>=max_dis) {
      m+=v*w;
      n+=w;
    }
  }

  ROS_WARN("w %f",n);
  if(n) m/=n;
  return m;
}

template <typename Point>
Eigen::Vector3f getMidN(const pcl::PointCloud<Point> &pc, const std::vector<int> &weight, const bool sm, const float max_dis, const pcl::PointCloud<Point> &old, const std::vector<COR_S> &cors) {
  Eigen::Vector3f m = Null3f();
  float n=0;

  for(int i=0; i<cors.size(); i++) {
    Eigen::Vector3f v;

    //if(weight[i]<1)
    //  continue;

    float w=1/(float)(weight[cors[i].ind_o]);
    v=pc[cors[i].ind_n].getVector3fMap()*w;

    if(sm && old[cors[i].ind_o].getVector3fMap().squaredNorm()<max_dis) {
      m+=v;
      n+=w;
    }
    else if(!sm && old[cors[i].ind_o].getVector3fMap().squaredNorm()>=max_dis) {
      m+=v;
      n+=w;
    }
  }

  ROS_WARN("w %f",n);
  if(n) m/=n;
  return m;
}

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/correspondence_estimation.h>
void visualize(const pcl::PointCloud<pcl::PointXYZ> &pc_old, const pcl::PointCloud<pcl::PointXYZ> &pc_new, const std::vector<COR_S> &cors, Eigen::Vector3f mo=Null3f(),Eigen::Vector3f mn=Null3f(),Eigen::Vector3f mot=Null3f())
{
  std::cout<<"show cloud? y/n";
  if(getchar()!='y')
    return;

  /* pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

  //blocks until the cloud is actually rendered
  pcl::registration::Correspondences cc;
  char buffer[128];
  for(int i=0; i<cors.size(); i++) {
    pcl::registration::Correspondence corr;
    corr.indexQuery=cors[i].ind_o;
    corr.indexMatch=cors[i].ind_n;
    cc.push_back(corr);

    sprintf(buffer,"l%d",i);
    viewer.addLine(pc_old[corr.indexQuery], pc_new[corr.indexMatch], 0,0,255, buffer);
  }
  pcl::visualization::PointCloudColorHandlerCustom<Point> single_color1(pc_old.makeShared(), 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<Point> single_color2(pc_new.makeShared(), 255, 0, 0);
  viewer.addPointCloud<Point>(pc_old.makeShared(),single_color1, "p1");
  viewer.addPointCloud<Point>(pc_new.makeShared(),single_color2, "p2");

  viewer.setBackgroundColor (10, 10, 10);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "p1");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "p2");
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();*/

  pcl::visualization::CloudViewer viewer("Cloud Viewer");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p1,p2,p3,p4,p5,p6;
  p1.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  p2.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  p3.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  p4.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  p5.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  p6.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(int i=0; i<pc_old.size(); i++) {
    pcl::PointXYZRGB p;
    p.x=pc_old[i].x;
    p.y=pc_old[i].y;
    p.z=pc_old[i].z;
    p.r=255;
    p.g=p.b=0;
    p1->push_back(p);
  }
  p1->width=p1->size();
  p1->height=1;
  for(int i=0; i<pc_new.size(); i++) {
    pcl::PointXYZRGB p;
    p.x=pc_new[i].x;
    p.y=pc_new[i].y;
    p.z=pc_new[i].z;
    p.g=255;
    p.b=p.b=0;
    p2->push_back(p);
  }
  p2->width=p2->size();
  p2->height=1;

  Eigen::Vector3f X,v;
  X(0)=1;
  X(1)=X(2)=0;

  for(int i=0; i<cors.size(); i++) {
    float dx=pc_old[cors[i].ind_o].x-pc_new[cors[i].ind_n].x;
    float dy=pc_old[cors[i].ind_o].y-pc_new[cors[i].ind_n].y;
    float dz=pc_old[cors[i].ind_o].z-pc_new[cors[i].ind_n].z;
    v=pc_new[cors[i].ind_n].getVector3fMap()-pc_old[cors[i].ind_o].getVector3fMap();
    for(int j=1; j<10; j++) {
      pcl::PointXYZRGB p;
      p.x=pc_new[cors[i].ind_n].x+dx*j/10.;
      p.y=pc_new[cors[i].ind_n].y+dy*j/10.;
      p.z=pc_new[cors[i].ind_n].z+dz*j/10.;

      p.b=255;
      p.r=p.g=0;
      if(v.dot(X)>0) {
      }
      else {
        p.r=255;
      }
      p3->push_back(p);
    }
  }
  p3->width=p3->size();
  p3->height=1;

  /*for(int i=0; i<50; i++) {
    pcl::PointXYZRGB p;
    p.x=sinf(M_PI*2*i/50.)*0.1+mo(0);
    p.y=cosf(M_PI*2*i/50.)*0.1+mo(1);
    p.z=mo(2);
    p.r=128;p.b=p.g=255;
    p4->push_back(p);
  }
  p4->width=p2->size();
  p4->height=1;
  for(int i=0; i<50; i++) {
    pcl::PointXYZRGB p;
    p.x=sinf(M_PI*2*i/50.)*0.1+mn(0);
    p.y=cosf(M_PI*2*i/50.)*0.1+mn(1);
    p.z=mn(2);
    p.r=p.b=p.g=255;
    p5->push_back(p);
  }
  p5->width=p2->size();
  p5->height=1;
  for(int i=0; i<50; i++) {
    pcl::PointXYZRGB p;
    p.x=sinf(M_PI*2*i/50.)*0.1+mot(0);
    p.y=cosf(M_PI*2*i/50.)*0.1+mot(1);
    p.z=mot(2);
    p.r=p.b=128;p.g=255;
    p6->push_back(p);
  }
  p6->width=p2->size();
  p6->height=1;*/

  //viewer.runOnVisualizationThreadOnce (viewerOneOff);

  //blocks until the cloud is actually rendered
  viewer.showCloud(p1,"p1");
  viewer.showCloud(p2,"p2");
  if(p3->size()>0) viewer.showCloud(p3,"p3");
  /*viewer.showCloud(p4,"p4");
  viewer.showCloud(p5,"p5");
  viewer.showCloud(p6,"p6");*/

  while (!viewer.wasStopped ())
  {
    usleep(1000);
  }

}
void visualize(const pcl::PointCloud<pcl::PointXYZ> &pc_old, const pcl::PointCloud<pcl::PointXYZ> &pc_new, const std::vector<COR_S> &cors,
               const std::vector<int> &weight_o, const std::vector<int> &weight_n)
{
  std::cout<<"show cloud? y/n/w";
  char c=getchar();
  if(c!='y'&&c!='w')
    return;

  pcl::visualization::CloudViewer viewer("Cloud Viewer");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p1,p2,p3,p4,p5,p6;
  p1.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  p2.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  p3.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(int i=0; i<pc_old.size(); i++) {
    pcl::PointXYZRGB p;
    p.x=pc_old[i].x;
    p.y=pc_old[i].y;
    p.z=pc_old[i].z;
    p.r=255;
    p.g=p.b=weight_o[i]*5;
    p1->push_back(p);
  }
  p1->width=p1->size();
  p1->height=1;
  for(int i=0; i<pc_new.size(); i++) {
    pcl::PointXYZRGB p;
    p.x=pc_new[i].x;
    p.y=pc_new[i].y;
    p.z=pc_new[i].z;
    p.g=255;
    p.b=p.b=weight_n[i]*5;
    p2->push_back(p);
  }
  p2->width=p2->size();
  p2->height=1;

  Eigen::Vector3f X,v;
  X(0)=1;
  X(1)=X(2)=0;

  for(int i=0; i<cors.size(); i++) {
    float dx=pc_old[cors[i].ind_o].x-pc_new[cors[i].ind_n].x;
    float dy=pc_old[cors[i].ind_o].y-pc_new[cors[i].ind_n].y;
    float dz=pc_old[cors[i].ind_o].z-pc_new[cors[i].ind_n].z;
    v=pc_new[cors[i].ind_n].getVector3fMap()-pc_old[cors[i].ind_o].getVector3fMap();
    for(int j=1; j<10; j++) {
      pcl::PointXYZRGB p;
      p.x=pc_new[cors[i].ind_n].x+dx*j/10.;
      p.y=pc_new[cors[i].ind_n].y+dy*j/10.;
      p.z=pc_new[cors[i].ind_n].z+dz*j/10.;

      p.b=255;
      p.r=p.g=0;
      if(v.dot(X)>0) {
      }
      else {
        p.r=255;
      }
      p3->push_back(p);
    }
  }
  p3->width=p3->size();
  p3->height=1;
  //blocks until the cloud is actually rendered
  viewer.showCloud(p1,"p1");
  viewer.showCloud(p2,"p2");
  if(p3->size()>0&&c!='w') viewer.showCloud(p3,"p3");

  while (!viewer.wasStopped ())
  {
    usleep(1000);
  }

}
void visualize(const pcl::PointCloud<pcl::PointXYZRGB> &pc_old)
{
  std::cout<<"show cloud? y/n";
  if(getchar()!='y')
    return;
  pcl::visualization::CloudViewer viewer("Cloud Viewer");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p1,p2,p3,p4,p5,p6;
  p1.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(int i=0; i<pc_old.size(); i++) {
    p1->push_back(pc_old[i]);
  }
  p1->width=p1->size();
  p1->height=1;

  viewer.showCloud(p1,"p1");

  while (!viewer.wasStopped ())
  {
    usleep(1000);
  }

}

template <typename Point>
Eigen::Vector3f getAxis(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new, const std::vector<COR_S> &cors, const bool sm, const float max_dis, const Eigen::Vector3f &rel_old, const Eigen::Vector3f &rel_new) {
  Eigen::Vector3f m = Null3f();

  for(int i=0; i<cors.size(); i++) {
    Eigen::Vector3f v,vp;

    v =pc_old[cors[i].ind_o].getVector3fMap();
    vp=pc_new[cors[i].ind_n].getVector3fMap();

    if((sm && v.squaredNorm()<max_dis)||(!sm && v.squaredNorm()>=max_dis)) {
      v -=rel_old;
      vp-=rel_new;
      v.normalize();
      vp.normalize();
      v=vp-v;
      v.normalize();
      m+=v;
    }
  }

  m.normalize();
  return m;
}


template <typename Point>
Eigen::Vector3f getAxis3(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new,
                         const std::vector<COR_S> &cors, const float max_dis,
                         const Eigen::Vector3f &rel_old, const Eigen::Vector3f &rel_new
                         , std::vector<int> &weight_o,std::vector<int> &weight_n, const Eigen::Vector3f &X2) {
  Eigen::Vector3f m[4];
  int g[4]={};

  m[0] = Null3f();
  m[1] = Null3f();
  m[2] = Null3f();
  m[3] = Null3f();

  Eigen::Vector3f X,Y;

  X(0)=1;
  X(1)=X(2)=0;
  Y(1)=1;
  Y(2)=Y(0)=0;

  X=rel_old.cross(X);
  Y=rel_old.cross(Y);

  for(int i=0; i<cors.size(); i++) {
    Eigen::Vector3f v,vp;

    v =pc_old[cors[i].ind_o].getVector3fMap();
    vp=pc_new[cors[i].ind_n].getVector3fMap();

    v -=rel_old;
    vp-=rel_new;

    int q=0;
    //q+=v(0)>0?1:0;
    //q+=v(1)<0?2:0;

    //q+=v(0)>rel_new(0)?1:0;
    //q+=v(1)<rel_new(1)?2:0;

    q+=v.dot(X)>0?1:0;
    q+=v.dot(Y)<0?2:0;

    //v.normalize();
    //vp.normalize();
    v=vp-v;
    v.normalize();
    //v/=v.squaredNorm();

    if(pc_old[cors[i].ind_o].getVector3fMap().squaredNorm()<rel_old.squaredNorm()) v=-v;

    if(v.dot(X2)<0) {
      //ROS_ERROR(".");
      continue;
    }

    //m[q]+=v / (float)(weight_o[cors[i].ind_o]*weight_n[cors[i].ind_n]); //weight by I
    m[q]+=v;// / (float)(weight_o[cors[i].ind_o]); //weight by I

    g[q]++;//=1.f / (float)(weight_o[cors[i].ind_o]); //weight by I
  }

  m[0]/=g[0];
  m[1]/=g[1];
  m[2]/=g[2];
  m[3]/=g[3];

  std::cout<<"m[0]\n"<<m[0]<<"\n";
  std::cout<<"m[1]\n"<<m[1]<<"\n";
  std::cout<<"m[2]\n"<<m[2]<<"\n";
  std::cout<<"m[3]\n"<<m[3]<<"\n";
  std::cout<<"m[0] "<<m[0].norm()<<"\n";
  std::cout<<"m[1] "<<m[1].norm()<<"\n";
  std::cout<<"m[2] "<<m[2].norm()<<"\n";
  std::cout<<"m[3] "<<m[3].norm()<<"\n";

  /*m[0].normalize();
  m[1].normalize();
  m[2].normalize();
  m[3].normalize();*/

  /*std::cout<<"m[0]\n"<<m[0]<<"\n";
  std::cout<<"m[1]\n"<<m[1]<<"\n";
  std::cout<<"m[2]\n"<<m[2]<<"\n";
  std::cout<<"m[3]\n"<<m[3]<<"\n";*/

  std::cout<<"a x\n"<<(m[0]+m[1]).cross(m[2]+m[3])<<"\n";
  std::cout<<"a y\n"<<(m[0]+m[2]).cross(m[1]+m[3])<<"\n";
  std::cout<<"a z\n"<<(m[0]-m[3]).cross(m[1]-m[3])<<"\n";

  Eigen::Vector3f axis, axis2;

  Eigen::Matrix3f aX, aY, aZ;

  aY=aZ=aX=aX.Identity();

  aX(0,0)=0;
  aY(1,1)=0;
  aZ(2,2)=0;

  Eigen::Vector3f ax,ay,az;
  ax=(m[0]+m[1]).cross(m[2]+m[3]);
  ay=(m[0]+m[2]).cross(m[1]+m[3]);
  az=(m[0]-m[3]).cross(m[1]-m[3]);

  ax.normalize();
  ay.normalize();
  az.normalize();

  axis(0) = (aX*(m[0]+m[1])).cross(aX*(m[2]+m[3])).norm() / std::abs((m[0]+m[1])(0)+(m[2]+m[3])(0));
  axis(1) = (aY*(m[0]+m[2])).cross(aY*(m[1]+m[3])).norm() / std::abs((m[0]+m[2])(1)+(m[1]+m[3])(1));
  axis(2) = (aZ*(m[0]-m[3])).cross(aZ*(m[1]-m[2])).norm() / std::abs((m[0]-m[3])(2)+(m[1]-m[2])(2));

  axis(0) = (aX*(m[0]+m[1])).cross(aX*(m[2]+m[3])).norm() * (std::abs(ax(0))+std::abs(ay(0))+std::abs(az(0)));
  axis(1) = (aY*(m[0]+m[2])).cross(aY*(m[1]+m[3])).norm() * (std::abs(ax(1))+std::abs(ay(1))+std::abs(az(1)));
  axis(2) = (aZ*(m[0]-m[3])).cross(aZ*(m[1]-m[2])).norm() * (std::abs(ax(2))+std::abs(ay(2))+std::abs(az(2)));

  axis(0) = (std::abs(ax(0))+std::abs(ay(0))+std::abs(az(0))) ;
  axis(1) = (std::abs(ax(1))+std::abs(ay(1))+std::abs(az(1))) ;
  axis(2) = (std::abs(ax(2))+std::abs(ay(2))+std::abs(az(2))) ;

  axis(0) = std::abs( ((m[0]+m[1])).cross((m[2]+m[3]))(0) );
  axis(1) = std::abs( ((m[0]+m[2])).cross((m[1]+m[3]))(1) );
  axis(2) = std::abs( ((m[0]-m[3])).cross((m[1]-m[2]))(2) );

  axis(0) = std::abs( ax(0) );
  axis(1) = std::abs( ay(1) );
  axis(2) = std::abs( az(2) );

  axis.normalize();

  if( m[0](1)+m[1](1)+m[2](1)+m[3](1) > 0)
    axis(0) = -axis(0);
  if( m[0](0)+m[1](0)+m[2](0)+m[3](0) < 0)
    axis(1) = -axis(1);
  if( m[0](0)*m[0](1)+m[3](0)*m[3](1) - m[1](0)*m[1](1)-m[2](0)*m[2](1)
      < 0)
    axis(2) = -axis(2);

  axis2=axis;
  std::cout<<"axis\n"<<axis<<"\n";

  aX(0,0)=0;
  aY(1,1)=0;
  aZ(2,2)=0;

  axis(0) = -(aX*(m[0]+m[1])).cross(aX*(m[2]+m[3]))(0);
  axis(1) = -(aY*(m[0]+m[2])).cross(aY*(m[1]+m[3]))(1);
  axis(2) = -(aZ*(m[0]-m[3])).cross(aZ*(m[1]-m[3]))(2);
  axis.normalize();

  std::cout<<"axis\n"<<axis<<"\n";

  return axis2;
}
template <typename Point>
Eigen::Vector3f getAxis2(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new,
                         std::vector<COR_S> &cors, const float max_dis,
                         const Eigen::Vector3f &rel_old, const Eigen::Vector3f &rel_new
                         , std::vector<int> &weight_o,std::vector<int> &weight_n) {
  Eigen::Vector3f m[4];
  int g[4]={};

  m[0] = Null3f();
  m[1] = Null3f();
  m[2] = Null3f();
  m[3] = Null3f();

  Eigen::Vector3f X,Y;

  X(0)=1;
  X(1)=X(2)=0;
  Y(1)=1;
  Y(2)=Y(0)=0;

  Eigen::Vector3f X2=X;

  X=rel_old.cross(X);
  Y=rel_old.cross(Y);

  float bin[100]={};
  float bing[100]={};

  static float border=2;
  //border/=2;

  pcl::PointCloud<pcl::PointXYZ> pc[4];
  pcl::PointCloud<pcl::PointXYZRGB> pc2[4];
  for(int i=0; i<cors.size(); i++) {
    Eigen::Vector3f v,vp;

    v =pc_old[cors[i].ind_o].getVector3fMap();
    vp=pc_new[cors[i].ind_n].getVector3fMap();

    v -=rel_old;
    vp-=rel_new;

    int q=0;
    //q+=v(0)>0?1:0;
    //q+=v(1)<0?2:0;

    //q+=v(0)>rel_new(0)?1:0;
    //q+=v(1)<rel_new(1)?2:0;

    q+=v.dot(X)>0?1:0;
    q+=v.dot(Y)<0?2:0;

    //v.normalize();
    //vp.normalize();
    v=vp-v;
    v.normalize();
    //v/=v.squaredNorm();

    if(pc_old[cors[i].ind_o].getVector3fMap().squaredNorm()<rel_old.squaredNorm()) v=-v;

    /*if(v.dot(X2)<0) {
      //ROS_ERROR(".");
      continue;
    }*/

    pcl::PointXYZ p;
    p.x=v(0);
    p.y=v(1);
    p.z=v(2);
    pc[q].points.push_back(p);
    pcl::PointXYZRGB p2;
    p2.x=v(0);
    p2.y=v(1);
    p2.z=10*(pc_new[cors[i].ind_n].getVector3fMap()-rel_new-(pc_old[cors[i].ind_o].getVector3fMap()-rel_old)).norm()/(pc_old[cors[i].ind_o].getVector3fMap()-rel_old).norm();

    p2.x = (pc_new[cors[i].ind_n].getVector3fMap()-rel_new).norm()/(pc_old[cors[i].ind_o].getVector3fMap()-rel_old).norm();
    p2.z = 0;(pc_new[cors[i].ind_n].getVector3fMap()-rel_new-(pc_old[cors[i].ind_o].getVector3fMap()-rel_old)).norm()/(pc_old[cors[i].ind_o].getVector3fMap()-rel_old).norm();
    p2.y = 0;(rel_new-rel_old).norm()/(pc_new[cors[i].ind_n].getVector3fMap()-pc_old[cors[i].ind_o].getVector3fMap()).norm();

    if(p2.x<1)
      p2.x=1/p2.x;
    //if(p2.y<1)
    //  p2.y=1/p2.y;
    p2.z = p2.z/sqrtf(p2.x*p2.x+p2.y*p2.y)+1;


    //std::cout<<p2<<"\n";
    p2.r=p2.b=p2.g=108;

    /*if(p2.x>border) {

      weight_o[cors[i].ind_o]--;
      weight_n[cors[i].ind_n]--;

      std::swap(cors[i],cors.back());
      cors.resize(cors.size()-1);

      --i;
      continue;
    }*/
    bin[(int)((p2.z-1)*60)]++;
    if(cors[i].ind_o==cors[i].ind_n) {
      p2.r=255;
      bing[(int)((p2.z-1)*60)]++;
    }

    p2.x=p2.y=0;

    p2.x = (pc_new[cors[i].ind_n].getVector3fMap()-rel_new).norm()/(pc_old[cors[i].ind_o].getVector3fMap()-rel_old).norm();
    p2.z = 0;//(pc_new[cors[i].ind_n].getVector3fMap()-rel_new-(pc_old[cors[i].ind_o].getVector3fMap()-rel_old)).norm()/(pc_old[cors[i].ind_o].getVector3fMap()-rel_old).norm();
    p2.y = 0;(rel_new-rel_old).norm()/(pc_new[cors[i].ind_n].getVector3fMap()-pc_old[cors[i].ind_o].getVector3fMap()).norm();


    if(p2.x<1)
      p2.x=1/p2.x;
    //if(p2.y<1)
    //  p2.y=1/p2.y;

    p2.x -= 1;

    //if(std::abs(p2.y-1)<0.1&&std::abs(p2.x-1)<0.1)
    pc2[0].points.push_back(p2);

    //m[q]+=v / (float)(weight_o[cors[i].ind_o]*weight_n[cors[i].ind_n]); //weight by I
    m[q]+=v;// / (float)(weight_o[cors[i].ind_o]); //weight by I

    g[q]++;//=1.f / (float)(weight_o[cors[i].ind_o]); //weight by I
  }

  int s=0;
  for(int i=0; i<20; i++) {
    s+=bin[i];
    std::cout<<s<<"\t";
  }
  std::cout<<"\n";
  for(int i=0; i<20; i++) {
    std::cout<<bin[i]*100/s<<"\t";
  }
  std::cout<<"\n";
  s=0;
  for(int i=0; i<20; i++) {
    s+=bing[i];
    std::cout<<s<<"\t";
  }
  std::cout<<"\n";
  for(int i=0; i<20; i++) {
    std::cout<<bing[i]*100/s<<"\t";
  }
  std::cout<<"\n";

  for(int i=0; i<1; i++) {
    pcl::PCA<pcl::PointXYZ> pca;
    pca.compute(pc[i]);

    std::cout<<pca.getEigenValues()<<"\n";
    std::cout<<pca.getEigenVectors()<<"\n";

    visualize(pc2[i]);
    /*
  for(int i=0;i<weight_o.size(); i++)
    if(weight_o[i]>0) {
      pcl::PointXYZ p;
      p.x=pc_old[i].x;
      p.y=pc_old[i].y;
      p.z=pc_old[i].z;
      pc.push_back(p);
    }
  for(int i=0;i<weight_n.size(); i++)
    if(weight_n[i]>0) {
      pcl::PointXYZ p;
      p.x=pc_new[i].x;
      p.y=pc_new[i].y;
      p.z=pc_new[i].z;
      pc2.push_back(p);
    }
  pcl::PCA<pcl::PointXYZ> pca2;
  pca.compute(pc);
  pca2.compute(pc2);

  std::cout<<pca.getEigenValues()<<"\n";
  std::cout<<pca.getEigenVectors()<<"\n";

  std::cout<<pca2.getEigenValues()<<"\n";
  std::cout<<pca2.getEigenVectors()<<"\n";

  std::cout<<(pca.getEigenVectors().inverse()*pca2.getEigenVectors())<<"\n";*/
  }

  m[0]/=g[0];
  m[1]/=g[1];
  m[2]/=g[2];
  m[3]/=g[3];

  /*pca.getEigenValues().col(0)(0) = std::abs(pca.getEigenValues().col(0)(0));
  pca.getEigenValues().col(0)(1) = std::abs(pca.getEigenValues().col(0)(1));
  pca.getEigenValues().col(0)(2) = std::abs(pca.getEigenValues().col(0)(2));

  for(int i=0; i<4; i++) {
    m[i](0)*=pca.getEigenValues().col(0)(0);
    m[i](1)*=pca.getEigenValues().col(0)(1);
    m[i](2)*=pca.getEigenValues().col(0)(2);
  }*/

  std::cout<<"m[0]\n"<<m[0]<<"\n";
  std::cout<<"m[1]\n"<<m[1]<<"\n";
  std::cout<<"m[2]\n"<<m[2]<<"\n";
  std::cout<<"m[3]\n"<<m[3]<<"\n";
  std::cout<<"m[0] "<<m[0].norm()<<"\n";
  std::cout<<"m[1] "<<m[1].norm()<<"\n";
  std::cout<<"m[2] "<<m[2].norm()<<"\n";
  std::cout<<"m[3] "<<m[3].norm()<<"\n";

  /*m[0].normalize();
  m[1].normalize();
  m[2].normalize();
  m[3].normalize();*/

  /*std::cout<<"m[0]\n"<<m[0]<<"\n";
  std::cout<<"m[1]\n"<<m[1]<<"\n";
  std::cout<<"m[2]\n"<<m[2]<<"\n";
  std::cout<<"m[3]\n"<<m[3]<<"\n";*/

  std::cout<<"a x\n"<<(m[0]+m[1]).cross(m[2]+m[3])<<"\n";
  std::cout<<"a y\n"<<(m[0]+m[2]).cross(m[1]+m[3])<<"\n";
  std::cout<<"a z\n"<<(m[0]-m[3]).cross(m[1]-m[3])<<"\n";

  Eigen::Vector3f axis, axis2;

  Eigen::Matrix3f aX, aY, aZ;

  aY=aZ=aX=aX.Identity();

  {
    axis(0) = ((m[0]-m[1])).cross((m[2]-m[3]))(0);
    axis(1) = ((m[0]-m[2])).cross((m[1]-m[3]))(1);
    axis(2) = ((m[0]-m[3])).cross((m[1]-m[2]))(2);

    axis.normalize();

    std::cout<<"axis\n"<<axis<<"\n";
  }
  aX(0,0)=0;
  aY(1,1)=0;
  aZ(2,2)=0;

  Eigen::Vector3f ax,ay,az;
  ax=(m[0]+m[1]).cross(m[2]+m[3]);
  ay=(m[0]+m[2]).cross(m[1]+m[3]);
  az=(m[0]-m[3]).cross(m[1]-m[3]);

  ax.normalize();
  ay.normalize();
  az.normalize();

  axis(0) = (aX*(m[0]+m[1])).cross(aX*(m[2]+m[3])).norm() / std::abs((m[0]+m[1])(0)+(m[2]+m[3])(0));
  axis(1) = (aY*(m[0]+m[2])).cross(aY*(m[1]+m[3])).norm() / std::abs((m[0]+m[2])(1)+(m[1]+m[3])(1));
  axis(2) = (aZ*(m[0]-m[3])).cross(aZ*(m[1]-m[2])).norm() / std::abs((m[0]-m[3])(2)+(m[1]-m[2])(2));

  axis(0) = (aX*(m[0]+m[1])).cross(aX*(m[2]+m[3])).norm() * (std::abs(ax(0))+std::abs(ay(0))+std::abs(az(0)));
  axis(1) = (aY*(m[0]+m[2])).cross(aY*(m[1]+m[3])).norm() * (std::abs(ax(1))+std::abs(ay(1))+std::abs(az(1)));
  axis(2) = (aZ*(m[0]-m[3])).cross(aZ*(m[1]-m[2])).norm() * (std::abs(ax(2))+std::abs(ay(2))+std::abs(az(2)));

  axis(0) = (std::abs(ax(0))+std::abs(ay(0))+std::abs(az(0))) ;
  axis(1) = (std::abs(ax(1))+std::abs(ay(1))+std::abs(az(1))) ;
  axis(2) = (std::abs(ax(2))+std::abs(ay(2))+std::abs(az(2))) ;

  axis(0) = std::abs( ((m[0]+m[1])).cross((m[2]+m[3]))(0) );
  axis(1) = std::abs( ((m[0]+m[2])).cross((m[1]+m[3]))(1) );
  axis(2) = std::abs( ((m[0]-m[3])).cross((m[1]-m[2]))(2) );

  axis(0) = std::abs( ax(0) );
  axis(1) = std::abs( ay(1) );
  axis(2) = std::abs( az(2) );

  axis.normalize();

  if( m[0](1)+m[1](1)+m[2](1)+m[3](1) > 0)
    axis(0) = -axis(0);
  if( m[0](0)+m[1](0)+m[2](0)+m[3](0) < 0)
    axis(1) = -axis(1);
  if( m[0](0)*m[0](1)+m[3](0)*m[3](1) - m[1](0)*m[1](1)-m[2](0)*m[2](1)
      < 0)
    axis(2) = -axis(2);

  axis2=axis;
  std::cout<<"axis\n"<<axis<<"\n";

  aX(0,0)=0;
  aY(1,1)=0;
  aZ(2,2)=0;

  axis(0) = -(aX*(m[0]+m[1])).cross(aX*(m[2]+m[3]))(0);
  axis(1) = -(aY*(m[0]+m[2])).cross(aY*(m[1]+m[3]))(1);
  axis(2) = -(aZ*(m[0]-m[3])).cross(aZ*(m[1]-m[3]))(2);
  axis.normalize();

  std::cout<<"axis\n"<<axis<<"\n";

  return getAxis3(pc_old, pc_new,
                  cors, max_dis,
                  rel_old, rel_new
                  , weight_o,weight_n, axis2);
  return axis2;
}

template <typename Point>
int remByAxis(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new, std::vector<COR_S> &cors, const Eigen::Vector3f &rel_old, const Eigen::Vector3f &rel_new, const Eigen::Vector3f &axis, const float add
              , std::vector<int> &weight_o,std::vector<int> &weight_n) {
  int rem=0,mis=0;

  for(int i=0; i<cors.size(); i++) {
    Eigen::Vector3f v,vp, d;

    v =pc_old[cors[i].ind_o].getVector3fMap();
    vp=pc_new[cors[i].ind_n].getVector3fMap();

    v -=rel_old;
    vp-=rel_new;

    d=v.cross(axis);
    v+=d/d.norm()*add;

    v=vp-v;

    float a=v.dot(d);
    if(pc_old[cors[i].ind_o].getVector3fMap().squaredNorm()<rel_old.squaredNorm()) a=-a;


    if(a>0) {
      rem++;

      if(cors[i].ind_o==cors[i].ind_n) {
        ROS_ERROR("%d",i);
        std::cout<<pc_old[cors[i].ind_o].getVector3fMap()<<"\n";
        std::cout<<pc_new[cors[i].ind_n].getVector3fMap()<<"\n";
        std::cout<<v<<"\n";
        std::cout<<v.cross(axis)<<"\n";

        /*weight_o[cors[i].ind_o]--;
        weight_n[cors[i].ind_n]--;

        std::swap(cors[i],cors.back());
        cors.resize(cors.size()-1);
        --i;*/

        mis++;
      }
    }

  }

  ROS_INFO("would remove %d of %d (%d)", rem, cors.size(), mis);

  return rem;
}

inline pcl::PointXYZ vec3TOxyz(const Eigen::Vector3f &v) {
  pcl::PointXYZ p;
  p.x=v(0);
  p.y=v(1);
  p.z=v(2);
  return p;
}

Eigen::Matrix4f getTF3(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3, const Eigen::Vector3f &p1n, const Eigen::Vector3f &p2n, const Eigen::Vector3f &p3n) {

  pcl::PointCloud<pcl::PointXYZ> t,s;

  t.push_back(vec3TOxyz(p1n));
  t.push_back(vec3TOxyz(p2n));
  t.push_back(vec3TOxyz(p3n));

  s.push_back(vec3TOxyz(p1));
  s.push_back(vec3TOxyz(p2));
  s.push_back(vec3TOxyz(p3));

  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  pcl::estimateRigidTransformationSVD(t,s, transf);

  return transf;
}

Eigen::Matrix4f getTF(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p1n, const Eigen::Vector3f &p2n) {
  Eigen::Quaternionf q;

  Eigen::Vector3f Z, axis, a,b;
  Z(0)=0;
  Z(1)=0;
  Z(2)=1;

  a=p1-p2;
  b=p1n-p2n;

  axis=(b-a).cross(Z);

  axis.normalize();

  a = a.cross(axis);
  b = b.cross(axis);

  a.normalize();
  b.normalize();

  float dir = a.dot(b-a);

  if(dir<0)
    axis*=-1;

  float angle = acos(a.dot(b));
  std::cout<<"angle "<<angle<<"\n";
  std::cout<<"a "<<a<<"\n";
  std::cout<<"b"<<b<<"\n";

  std::cout<<"dir"<<dir<<"\n";

  Eigen::AngleAxisf rot(angle, axis);

  std::cout<<"C1\n"<<axis<<"\n";
  std::cout<<"rot\n"<<rot.toRotationMatrix()<<"\n";

  Eigen::Matrix3f R=rot.toRotationMatrix();

  Eigen::Vector3f t=((p1n+p2n)-R*(p1+p2))*0.5f;

  std::cout<<"t\n"<<((p2n)-R*(p2))<<"\n";
  std::cout<<"t\n"<<((p1n)-R*(p1))<<"\n";

  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  for(int x=0; x<3; x++)
    for(int y=0; y<3; y++)
      transf(x,y) = R(x,y);
  transf.col(3).head<3>() = t;

  return transf;
}


float getTF2dis(const Eigen::Vector3f &axis, const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p1n, const Eigen::Vector3f &p2n) {
  Eigen::Quaternionf q;

  Eigen::Vector3f r, a,b;

  a=p1-p2;
  b=p1n-p2n;

  r = a.cross(axis);

  return acosf( r.dot(a-b)/(r.norm()*(b-a).norm()));
}

float getTF2dis2(const Eigen::Vector3f &axis, const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p1n, const Eigen::Vector3f &p2n) {
  Eigen::Quaternionf q;

  Eigen::Vector3f r, a,b;

  a=p1-p2;
  b=p1n-p2n;

  a = a.cross(axis);
  b = b.cross(axis);

  return asinf((a-b).norm()/a.norm());

  a.normalize();
  b.normalize();

  return acosf( a.dot(b) );
}

Eigen::Matrix4f getTF2(const Eigen::Vector3f &axis, const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p_s, const Eigen::Vector3f &p1n, const Eigen::Vector3f &p2n, const Eigen::Vector3f &pn_s) {
  Eigen::Quaternionf q;

  Eigen::Vector3f Z, a,b;
  Z(0)=0;
  Z(1)=0;
  Z(2)=1;

  a=p1-p2;
  b=p1n-p2n;

  a = a.cross(axis);
  b = b.cross(axis);

  a.normalize();
  b.normalize();

  /*if(dir<0)
    axis*=-1;*/

  float angle = acos(a.dot(b));
  std::cout<<"angle "<<angle<<"\n";
  std::cout<<"a "<<a<<"\n";
  std::cout<<"b"<<b<<"\n";

  std::cout<<"axis\n"<<axis<<"\n";

  Eigen::AngleAxisf rot(angle, axis);

  std::cout<<"C1\n"<<axis<<"\n";
  std::cout<<"rot\n"<<rot.toRotationMatrix()<<"\n";

  Eigen::Matrix3f R=rot.toRotationMatrix();

  Eigen::Vector3f t=pn_s-R*p_s;

  std::cout<<"pn_s\n"<<pn_s<<"\n";
  std::cout<<"R*p_s\n"<<(R*p_s)<<"\n";
  std::cout<<"t\n"<<((p2n)-R*(p2))<<"\n";
  std::cout<<"t\n"<<((p1n)-R*(p1))<<"\n";
  std::cout<<"t\n"<<(((p1n+p2n)-R*(p1+p2))*0.5f)<<"\n";

  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  for(int x=0; x<3; x++)
    for(int y=0; y<3; y++)
      transf(x,y) = R(x,y);
  transf.col(3).head<3>() = t;

  return transf;
}

template <typename Point>
Eigen::Matrix4f getTF2ex(const Eigen::Vector3f &axis,
                         const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new,
                         const std::vector<COR_S> &cors, const std::vector<int> &weight_o, const std::vector<int> &weight_n,
                         const Eigen::Vector3f &mid_old, const Eigen::Vector3f &mid_new
) {
  float avg_angel=0;
  float n=0;

  for(int i=0; i<cors.size(); i++) {

    float w=1/(float)weight_o[cors[i].ind_o];

    Eigen::Vector3f a,b;

    a=pc_old[cors[i].ind_o].getVector3fMap()-mid_old;
    b=pc_new[cors[i].ind_n].getVector3fMap()-mid_new;

    a = a.cross(axis);
    b = b.cross(axis);

    a.normalize();
    b.normalize();

    float angle = acos(a.dot(b));
    avg_angel+=w*angle;
    n+=w;

  }
  avg_angel/=n;

  Eigen::AngleAxisf rot(avg_angel, axis);
  Eigen::Matrix3f R=rot.toRotationMatrix();

  std::cout<<"C1\n"<<axis<<"\n";
  std::cout<<"rot\n"<<rot.toRotationMatrix()<<"\n";
  std::cout<<"angle "<<avg_angel<<"\n";

  Eigen::Vector3f t;
  t(0)=t(1)=t(2)=0;
  n=0;

  for(int i=0; i<cors.size(); i++) {

    float w=1/(float)weight_o[cors[i].ind_o];

    t+=w* (pc_new[cors[i].ind_n].getVector3fMap()-R*pc_old[cors[i].ind_o].getVector3fMap()-mid_old) ;
    n+=w;

  }
  t/=n;

  std::cout<<"t\n"<<t<<"\n";

  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  for(int x=0; x<3; x++)
    for(int y=0; y<3; y++)
      transf(x,y) = R(x,y);
  transf.col(3).head<3>() = t;

  return transf;
}

template <typename Point>
float avgError(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new, const std::vector<COR_S> &cors, const Eigen::Matrix4f &tf=Eigen::Matrix4f::Identity()) {
  float er=0.f;
  for(int i=0; i<cors.size(); i++) {
    er+=(tf*pc_old[cors[i].ind_o].getVector4fMap()-pc_new[cors[i].ind_n].getVector4fMap()).norm()/pc_old[cors[i].ind_o].getVector3fMap().norm();
  }
  return er/cors.size();
}

inline float vorz(float v) {
  return v;
}

template <typename Point>
int remError(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new,
             std::vector<COR_S> &cors, const Eigen::Matrix4f &tf, const float border_a, const float border_t, std::vector<int> &weight_o,std::vector<int> &weight_n,
             const Eigen::Vector3f &mid_old, const Eigen::Vector3f &mid_new) {

  std::vector<std::vector<int> > cs;

  for(int i=0; i<pc_new.size(); i++)
    weight_n[i]=0;
  for(int i=0; i<pc_old.size(); i++) {
    weight_o[i]=0;
    std::vector<int> s;
    for(int j=0; j<cors.size(); j++) {
      if(cors[j].ind_o==i)
        s.push_back(cors[j].ind_n);
    }
    cs.push_back(s);
  }

  pcl::PointCloud<Point> tpc,spc;
  cors.clear();
  for(int i=0; i<pc_old.size(); i++) {

    //if(i%100==0)
    //  std::cout<<".";

    std::vector<float> f1,f2,f3,f4;

    if(cs[i].size()<1) continue;

    int mi=0;
    float vmi=10000000000;

    for(int j=0; j<cs[i].size(); j++) {

      float t=0;
      float t2=0;
      Eigen::Vector3f v=(pc_old[i].getVector3fMap()-mid_old).cross(pc_new[cs[i][j]].getVector3fMap()-pc_old[i].getVector3fMap());
      Eigen::Vector3f v2=(pc_old[i].getVector3fMap()-mid_old).cross(v);
      v.normalize();
      v2.normalize();
      for(int k=0; k<pc_old.size(); k++) {
        t+=vorz((pc_old[k].getVector3fMap()-pc_old[i].getVector3fMap()).dot(v));
        t2+=vorz((pc_old[k].getVector3fMap()-pc_old[i].getVector3fMap()).dot(v2));
      }
      f1.push_back(t);
      f3.push_back(t2);

      v=(pc_new[cs[i][j]].getVector3fMap()-mid_new).cross(pc_new[cs[i][j]].getVector3fMap()-pc_old[i].getVector3fMap());
      v2=(pc_new[cs[i][j]].getVector3fMap()-mid_new).cross(v);
      t=0;
      t2=0;
      for(int k=0; k<pc_new.size(); k++) {
        t+=vorz((pc_new[k].getVector3fMap()-pc_new[cs[i][j]].getVector3fMap()).dot(v));
        t2+=vorz((pc_new[k].getVector3fMap()-pc_new[cs[i][j]].getVector3fMap()).dot(v2));
      }
      f2.push_back(t);
      f4.push_back(t2);

      //std::cout<<i<<" "<<cs[i][j]<<": "<<f1.back()<<", "<<f2.back()<<"\n";

      t=std::abs(f1.back()-f2.back())+std::abs(f3.back()-f4.back());

      if(t<vmi) {
        mi=j;
        vmi=t;
      }
    }

    if(cs[i][mi]!=i) {
      ROS_ERROR("%f %f", f1[mi], f2[mi]);
      for(int j=0; j<cs[i].size(); j++)
        std::cout<<i<<" "<<cs[i][j]<<": "<<f1[j]<<", "<<f2[j]<<" - "<<f3[j]<<", "<<f4[j]<<" = "<<
        std::abs( (f1[j]-f2[j])*(f3[j]-f4[j]) )
      <<"\n";
    }

    COR_S c;
    c.ind_o=i;
    c.ind_n=cs[i][mi];

    if(weight_n[c.ind_n]) continue;

    cors.push_back(c);

    weight_o[i]++;
    weight_n[c.ind_n]++;

    tpc.points.push_back(pc_old[i]);
    spc.points.push_back(pc_new[c.ind_n]);
  }


  /* Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  pcl::estimateRigidTransformationSVD(tpc,spc, transf);

  pcl::PointCloud<Point> tpc2;
  pcl::transformPointCloud(tpc,tpc2,tf);
  std::cout<<tf<<"\n";
  cors.clear();
  visualize(tpc2,pc_new, cors);*/

  return 10;
  int n=0;
  int sz=cors.size();
  float dA,dT;

  Eigen::Vector4f dir;
  dir(0)=dir(1)=dir(2)=dir(3)=0;

  float avg=0;
  for(int i=0; i<cors.size(); i++) {
    Eigen::Vector4f v=(tf*pc_old[cors[i].ind_o].getVector4fMap());

    float d=(v-pc_new[cors[i].ind_n].getVector4fMap()).squaredNorm();
    avg+=sqrtf(d);

    v-=pc_new[cors[i].ind_n].getVector4fMap();//testing;
    v.normalize();//testing;
    dir += v;//testing;
  }
  avg/=sz;
  ROS_INFO("AVG %f TMAX %f",avg, border_t);

  dir.normalize();
  std::cout<<"translation dir:\n"<<dir<<"\n";

  for(int i=0; i<cors.size(); i++) {

    Eigen::Vector4f v=(tf*pc_old[cors[i].ind_o].getVector4fMap());
    Eigen::Vector4f v2=pc_new[cors[i].ind_n].getVector4fMap();

    float dT = std::abs(v.norm()-v2.norm());
    float dA = acosf(v.dot(v2)/(v.norm()*v2.norm()));

    if( dA>border_a || dT>border_t || (v-v2).squaredNorm()>(pc_old[cors[i].ind_o].getVector4fMap()-v2).squaredNorm()+border_t )
      /*getDis( v.head<3>(),
            pc_new[cors[i].ind_n].getVector3fMap(),
            dT, dA);
    if(dT>border_t || dA>border_a)*/
      //if( (tf*pc_old[cors[i].ind_o].getVector4fMap()-pc_new[cors[i].ind_n].getVector4fMap()).norm()/pc_old[cors[i].ind_o].getVector3fMap().norm()>border )
    {
      //cors.erase(cors.begin()+i);

      if(cors[i].ind_o==cors[i].ind_n) {
        ROS_ERROR("%f %f",dT,dA);
      }

      weight_o[cors[i].ind_o]--;
      weight_n[cors[i].ind_n]--;

      std::swap(cors[i],cors.back());
      cors.resize(cors.size()-1);
      --i;
      ++n;
    }
  }

  ROS_INFO("AVG %f TMAX %f",avg, border_t);
  ROS_INFO("REMOVED %d",n);
  return n;
}

#define GET_MIDS() \
    \
    mid_old = getMidO(pc_old,weight_o,cors);\
    mid_new = getMidO(pc_new,weight_o,cors);\
    \
    mid_ob = getMidO(pc_old,weight_o,cors, true, mid_old.squaredNorm());\
    mid_nb = getMidN(pc_new,weight_o, true, mid_old.squaredNorm(), pc_old,cors);\
    \
    mid_oa = getMidO(pc_old,weight_o,cors, false, mid_old.squaredNorm());\
    mid_na = getMidN(pc_new,weight_o, false, mid_old.squaredNorm(), pc_old,cors);\
    \
    mid_old = 0.5f*(mid_ob+mid_oa);\
    mid_new = 0.5f*(mid_nb+mid_na);

template <typename Point>
Eigen::Matrix4f _findTF(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new, std::vector<COR_S> &cors, float border_a, float border_t, float border_a2,
                        std::vector<int> &weight_o, std::vector<int> &weight_n, const Eigen::Vector3f &prev_trans=Null3f()) {

  ROS_ERROR("tmax %f", border_t);
  ROS_ERROR("rmax %f", border_a);

  Eigen::Vector3f mid_old, mid_new, mid_ob, mid_nb, mid_oa, mid_na, axis;
  int rem = 0;

  GET_MIDS();

  std::cout<<"mid_old\n"<<mid_old<<"\n";
  std::cout<<"mid_new\n"<<mid_new<<"\n";
  std::cout<<"mid_ob\n"<<mid_ob<<"\n";
  std::cout<<"mid_oa\n"<<mid_oa<<"\n";
  std::cout<<"mid_nb\n"<<mid_nb<<"\n";
  std::cout<<"mid_na\n"<<mid_na<<"\n";
  visualize(pc_old,pc_new, cors, mid_old,mid_new);

  axis=getAxis2(pc_old, pc_new, cors, mid_old.squaredNorm(), mid_old, mid_new, weight_o, weight_n);



  //remByAxis(pc_old, pc_new, cors, mid_old, mid_new, axis, 2*axis2*border_t);
  //remByAxis(pc_old, pc_new, cors, ::Null3f(), ::Null3f(), axis, axis*border_t);
  if(0) {
    int remR=0, remF=0;
    float a_avg=0;
    for(int i=0; i<cors.size(); i++) {
      //float a=getTF2dis(axis, mid_old,pc_old[cors[i].ind_o].getVector3fMap(), mid_new,pc_new[cors[i].ind_n].getVector3fMap());
      //axis(0)=1;
      //axis(1)=0;
      //axis(2)=0;
      float a=getTF2dis(axis, Null3f(),pc_old[cors[i].ind_o].getVector3fMap(), prev_trans,pc_new[cors[i].ind_n].getVector3fMap());
      a_avg+=a;
    }
    a_avg/=cors.size();
    ROS_INFO("avg %f",a_avg);
    //a_avg=0;

    int num_a=0, num_b=0;
    int num_ar=0, num_br=0;
    for(int i=0; i<cors.size(); i++) {
      float a=getTF2dis(axis, Null3f(),pc_old[cors[i].ind_o].getVector3fMap(), prev_trans,pc_new[cors[i].ind_n].getVector3fMap());

      if(a>a_avg)
        num_a++;
      else
        num_b++;

      if(cors[i].ind_o==cors[i].ind_n) {
        if(a>a_avg)
          num_ar++;
        else
          num_br++;
      }

      if( (a>a_avg+border_a) && cors[i].ind_o==cors[i].ind_n) {
        ROS_ERROR("wrong remove %f",a);
        remF++;
      }

      if( a>a_avg+border_a ) {
        remR++;
        weight_o[cors[i].ind_o]--;
        weight_n[cors[i].ind_n]--;

        std::swap(cors[i],cors.back());
        cors.resize(cors.size()-1);
        --i;

        ++rem;
      }

    }
    ROS_INFO("suppose %d/%d (%d/%d)",num_b,num_a,num_br, num_ar);
    ROS_INFO("would rem %d/%d",remR,remF);
    visualize(pc_old,pc_new, cors, mid_old,mid_new);

    GET_MIDS();

    axis=getAxis2(pc_old, pc_new, cors, mid_old.squaredNorm(), mid_old, mid_new, weight_o, weight_n);

#if TESTING_
    getchar();
#endif
  }
  if(0){
    int remR=0, remF=0;
    float a_avg=0;
    for(int i=0; i<cors.size(); i++) {
      float a=getTF2dis2(axis, Null3f(),pc_old[cors[i].ind_o].getVector3fMap(), prev_trans,pc_new[cors[i].ind_n].getVector3fMap());
      a_avg+=a;
    }
    a_avg/=cors.size();
    ROS_INFO("avg %f",a_avg);
    //a_avg=0;

    int num_a=0, num_b=0;
    int num_ar=0, num_br=0;
    //while(!remR)
    {
      for(int i=0; i<cors.size(); i++) {
        float a=getTF2dis2(axis, Null3f(),pc_old[cors[i].ind_o].getVector3fMap(), prev_trans,pc_new[cors[i].ind_n].getVector3fMap());

        if(a>a_avg)
          num_a++;
        else
          num_b++;

        if(cors[i].ind_o==cors[i].ind_n) {
          if(a>a_avg)
            num_ar++;
          else
            num_br++;
        }

        bool b= a<a_avg-border_a || a>border_a*2+a_avg;

        if( b && /*a>=a_avg-border_a*2 && a>border_a*2+a_avg &&*/ cors[i].ind_o==cors[i].ind_n) {
          ROS_ERROR("wrong remove %f",a);
          remF++;
        }

        if( b /*a>=a_avg-border_a*2 && a>border_a*2+a_avg*/ ) {
          remR++;
          weight_o[cors[i].ind_o]--;
          weight_n[cors[i].ind_n]--;

          std::swap(cors[i],cors.back());
          cors.resize(cors.size()-1);
          --i;

          ++rem;
        }

      }
      //border_a*=0.5;
    }
    ROS_INFO("suppose %d/%d (%d/%d)",num_b,num_a,num_br, num_ar);
    ROS_INFO("would rem %d/%d",remR,remF);

    GET_MIDS();

    visualize(pc_old,pc_new, cors, mid_old,mid_new);

  }

  visualize(pc_old,pc_new, cors, mid_old,mid_new);

  //getTF2ex(axis, pc_old,pc_new, cors, weight_o,weight_n, mid_old,mid_new); //testing
  Eigen::Matrix4f tf=getTF2(axis, mid_oa,mid_ob,mid_old, mid_na,mid_nb,mid_new);
  std::cout<<tf<<"\n";

  if( (tf.col(3).head<3>()).squaredNorm() > 0.04) {
    tf.col(3).head<3>()=tf.col(3).head<3>()*0.2/tf.col(3).head<3>().norm();
  }

  float border;

  std::cout<<"mid_old\n"<<mid_old<<"\n";
  std::cout<<"mid_new\n"<<mid_new<<"\n";
  std::cout<<"mid_ob\n"<<mid_ob<<"\n";
  std::cout<<"mid_oa\n"<<mid_oa<<"\n";
  std::cout<<"mid_nb\n"<<mid_nb<<"\n";
  std::cout<<"mid_na\n"<<mid_na<<"\n";

  ROS_ERROR("above/below");
  visualize(pc_old,pc_new, cors, mid_ob,mid_nb);
  visualize(pc_old,pc_new, cors, mid_oa,mid_na);

  std::cout<<tf<<"\n";

#if TESTING_
  getchar();
#endif

  std::cout<<"error before "<<avgError(pc_old, pc_new, cors)<<"\n";
  std::cout<<"error after "<<(border=avgError(pc_old, pc_new, cors, tf))<<"\n";


  rem += remError(pc_old, pc_new, cors, tf, border_a, border_t, weight_o,weight_n, mid_old,mid_new);
  {
    Eigen::Vector4f mido;
    mido(0)=mid_old(0);mido(1)=mid_old(1);mido(2)=mid_old(2);
    mido(3)=0;
    visualize(pc_old,pc_new, cors, mid_old,mid_new, (tf*mido).head<3>());
  }
  //int rem=10;

  border_a*=0.5f;
  border_t*=0.5f;
  border_a2*=0.5f;

  /*const pcl::PointCloud<Point> &outer_pc=pc_old;
  {
    GET_MIDS();
    axis=getAxis2(pc_old, pc_new, cors, mid_old.squaredNorm(), mid_old, prev_trans, weight_o, weight_n);
    tf=getTF2(axis, mid_oa,mid_ob,mid_old, mid_na,mid_nb,mid_new);

    pcl::PointCloud<Point> pc_old=outer_pc;
    pcl::transformPointCloud(pc_old,pc_old,tf);
    GET_MIDS();

    axis=getAxis2(pc_old, pc_new, cors, mid_old.squaredNorm(), mid_old, prev_trans, weight_o, weight_n);

#if TESTING_
    ROS_ERROR("inverse check");
    getchar();
#endif
    rem+=remByAxis(pc_old, pc_new, cors, Null3f(), Null3f(), axis, border_t*0.5, weight_o,weight_n);
    visualize(pc_old,pc_new, cors, mid_old,mid_new);
  }*/


  GET_MIDS();
  axis=getAxis2(pc_old, pc_new, cors, mid_old.squaredNorm(), mid_old, mid_new, weight_o, weight_n);
  tf=getTF2(axis, mid_oa,mid_ob,mid_old, mid_na,mid_nb,mid_new);

  std::cout<<"FINAL TF\n"<<tf<<"\n";

  std::cout<<"removed "<<rem<<" ("<<cors.size()<<")\n";
  std::cout<<"error after2 "<<avgError(pc_old, pc_new, cors, tf)<<"\n";

  ROS_INFO("num %d",weight_o.size());
  ROS_INFO("num %d",weight_n.size());
  for(int i=0; i<weight_o.size(); i++)
    std::cout<<weight_o[i]<<" ";
  for(int i=0; i<weight_n.size(); i++)
    std::cout<<weight_n[i]<<" ";

  int cor=0;
  for(int i=0; i<cors.size(); i++)
    cor+=cors[i].ind_n==cors[i].ind_o?1:0;
  ROS_ERROR("correct %d/%d", cor, cors.size());

#if TESTING_
  getchar();
#endif

  if(rem>2&&border>0.01&&cors.size()>pc_old.size())
    return _findTF(pc_old,pc_new,cors, border_a, border_t, border_a2, weight_o, weight_n, tf.col(3).head<3>() );//0.5*(tf.col(3).head<3>()+prev_trans) );

  return tf;
}

int search_sorted_vector(const std::vector<SORT_S2> &tv, const float val) {
  int i=tv.size()-1;
  int step=tv.size()/2-1;
  while(i&&step&&i<tv.size()) {
    if(tv[i].dis>=val) {
      i-=step;
    }
    else if(tv[i].dis<val)
      i+=step;
    else
      break;
    step/=2;
  }
  return i;
}


template <typename Point>
Eigen::Matrix4f findTF_fast(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new,
                            const float rmax=0.1, const float tmax=0.1, Eigen::Matrix4f tf=Eigen::Matrix4f::Identity())
{
  std::vector<SORT_S2> tv;
  std::vector<COR_S> cors;

  for(int i=0; i<pc_new.size(); i++) {
    SORT_S2 s;
    s.dis=pc_new.points[i].getVector3fMap().norm();
    s.ind=i;
    tv.push_back(s);
  }

  SORT_S2 _s_;
  std::sort(tv.begin(),tv.end(), _s_);

  std::vector<int> weight_o, weight_n;

  weight_o.resize(pc_old.size(), 0);
  weight_n.resize(pc_new.size(), 0);

  float rmax_ = sinf(rmax);
  rmax_*=rmax_;

  weight_o.resize(pc_old.size(), 0);
  weight_n.resize(pc_new.size(), 0);

  COR_S c;
  for(int i=0; i<pc_old.size(); i++) {
    float t = pc_old.points[i].getVector3fMap().norm();
    Eigen::Vector3f vo = pc_old.points[i].getVector3fMap();

    int num=0;
    for(int j=search_sorted_vector(tv,t-tmax); j<tv.size(); j++) {

      float dT = std::abs(tv[j].dis-t);
      if( dT < tmax) {
        Eigen::Vector3f vn = pc_new.points[tv[j].ind].getVector3fMap();

        float dA = (((vn-vo).squaredNorm()/vo.squaredNorm()));

        if(dA>=rmax_) continue;

        if(num>0 &&
            (vn-vo)
            .dot
            (pc_new[cors.back().ind_n].getVector3fMap()-vo)<0. )
        {
          cors.resize(cors.size()-num);
          break;
        }

        ++num;

        c.ind_o = i;
        c.ind_n = tv[j].ind;
        cors.push_back(c);

        weight_o[i]++;
        weight_n[c.ind_n]++;
      }
      else if(tv[j].dis-t > tmax)
        break;
    }

  }

  pcl::TransformationFromCorrespondences transFromCorr;
  for(int i=0; i<cors.size(); i++) {
    transFromCorr.add(pc_old[cors[i].ind_o].getVector3fMap(), pc_new[cors[i].ind_n].getVector3fMap(), 1./std::min(weight_o[cors[i].ind_o],weight_n[cors[i].ind_n]));
  }
  Eigen::Matrix4f tf2 = transFromCorr.getTransformation().matrix();

  pcl::PointCloud<Point> tpc;
  pcl::transformPointCloud(pc_old,tpc,tf2);

  if(tmax<0.01)
    return tf*tf2;
  return findTF_fast(tpc,pc_new, rmax*0.5, tmax*0.5, tf*tf2);
}

template <typename Point>
Eigen::Matrix4f findTF(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new,
                       const float rmax=0.1, const float tmax=0.1, Eigen::Matrix4f tf=Eigen::Matrix4f::Identity())
{
  int dbg_rem1=0,dbg_rem2=0;

  std::vector<SORT_S2> tv;
  std::vector<COR_S> cors;

  for(int i=0; i<pc_new.size(); i++) {
    SORT_S2 s;
    s.dis=pc_new.points[i].getVector3fMap().norm();
    s.ind=i;
    tv.push_back(s);
  }

  SORT_S2 _s_;
  std::sort(tv.begin(),tv.end(), _s_);

  float avgT=0.f;
  /*int m=0;
  for(int i=0; i<pc_old.size(); i++) {
    float t = pc_old.points[i].getVector3fMap().norm();
    for(int j=0; j<tv.size(); j++) {
      if( std::abs(tv[j].dis-t) < tmax) {
        float dA;
        dA = acosf(pc_old.points[i].getVector3fMap().dot(pc_new.points[tv[j].ind].getVector3fMap())/(pc_old.points[i].getVector3fMap().norm()*pc_new.points[tv[j].ind].getVector3fMap().norm()));

        if(dA>=rmax) continue;

        avgT+=tv[j].dis-t;
        m++;

        if(i==0)
          std::cout<<j<<" ";
      }
    }
  }
  ROS_ERROR("NUM %d",m);
  avgT/=m;
  std::cout<<"avgT "<<avgT<<"\n";*/

  std::vector<int> weight_o, weight_n;

  weight_o.resize(pc_old.size(), 0);
  weight_n.resize(pc_new.size(), 0);

  for(int i=0; i<pc_old.size(); i++) {
    float t = pc_old.points[i].getVector3fMap().norm();

    float bestd;
    int num=0;
    for(int j=search_sorted_vector(tv,t-tmax); j<tv.size(); j++) {
      //float dt=tv[j].dis-t;
      //if( std::abs(dt) < tmax && std::abs(dt-avgT)<tmax/4) {
      //std::cout<<j<<" "<<tv[j].dis-t<<"\n";
      float dT=std::abs(tv[j].dis-t);
      if( dT < tmax) {
        float dA;
        //dA = acosf(pc_old.points[i].getVector3fMap().dot(pc_new.points[tv[j].ind].getVector3fMap())/(pc_old.points[i].getVector3fMap().norm()*pc_new.points[tv[j].ind].getVector3fMap().norm()));
        dA = (((pc_new.points[tv[j].ind].getVector3fMap()-pc_old.points[i].getVector3fMap()).norm()/pc_old.points[i].getVector3fMap().norm()));

        if(dA>=rmax) continue;
        //if((pc_old.points[i].getVector3fMap()-pc_new.points[tv[j].ind].getVector3fMap()).squaredNorm()==0)
        //  continue;

        Eigen::Vector3f X;
        X(0)=1;X(1)=0;X(2)=0;

        //if((pc_new.points[tv[j].ind].getVector3fMap()-pc_old.points[i].getVector3fMap()).dot(X)<0) continue;

        //        if(i!=tv[j].ind) continue;

        ++num;
        /*if(num>1 &&
            (pc_new[cors.back().ind_n].getVector3fMap()-pc_new[tv[j].ind].getVector3fMap()).squaredNorm()>0.02 )//if(num>200)
        {
          --num;
          weight_o[i]-=num;
          for(int i=0; i<num; i++) {
            weight_n[cors[cors.size()-1-i].ind_n]--;
          }
          cors.resize(cors.size()-num);
          break;
        }*/

        COR_S c;
        c.ind_o = i;
        c.ind_n = tv[j].ind;
        if(num==1){
          cors.push_back(c);
          bestd=dA*dA+dT*dT;
        }
        else if(bestd>dA*dA+dT*dT) {

          if(
              (pc_new[c.ind_n].getVector3fMap()-pc_old.points[i].getVector3fMap())
              .dot
              (pc_new[cors.back().ind_n].getVector3fMap()-pc_old.points[i].getVector3fMap())<0 )
          {
            cors.resize(cors.size()-1);
            break;
          }

          bestd=dA*dA+dT*dT;
          cors.back()=c;
        }

        weight_o[i]++;
        weight_n[c.ind_n]++;
      }
      else if(tv[j].dis-t > tmax)
        break;
    }
    /*
    int ok=0;
    for(int j=0; ok && j<num-1; j++) {
      Eigen::Vector3f v= pc_new[cors[cors.size()-j-1].ind_n].getVector3fMap()-pc_old.points[i].getVector3fMap();
      v.normalize();
      for(int k=j+1; k<num; k++) {
        Eigen::Vector3f v2= pc_new[cors[cors.size()-k-1].ind_n].getVector3fMap()-pc_old.points[i].getVector3fMap();
        v2.normalize();

        if(v2.dot(v)<-0) {
          ok++;
        }

      }
    }
    if(ok>num*(num+1)/16) {
      weight_o[i]-=num;
      for(int i=0; i<num; i++) {
        weight_n[cors[cors.size()-1-i].ind_n]--;
      }
      cors.resize(cors.size()-num);
    }
    /*else {
      for(int k=0; k<num-1; k++) {
        float a=(pc_new[cors[cors.size()-k-1].ind_n].getVector3fMap()-pc_old.points[i].getVector3fMap()).squaredNorm();
        float b=(pc_new[cors[cors.size()-num].ind_n].getVector3fMap()-pc_old.points[i].getVector3fMap()).squaredNorm();
        if(a<b)
          std::swap(cors[cors.size()-num],cors[cors.size()-k-1]);
      }
      --num;
        /*weight_o[i]-=num;
        for(int i=0; i<num; i++) {
          weight_n[cors[cors.size()-1-i].ind_n]--;
        }
        cors.resize(cors.size()-num);
    }*/

    //getchar();
  }

  pcl::PointCloud<Point> tpc;

  pcl::TransformationFromCorrespondences transFromCorr;
  for(int i=0; i<cors.size(); i++) {
    transFromCorr.add(pc_old[cors[i].ind_o].getVector3fMap(), pc_new[cors[i].ind_n].getVector3fMap(), 1./(weight_o[cors[i].ind_o]*weight_n[cors[i].ind_n]));
  }
  Eigen::Matrix4f tf2 = transFromCorr.getTransformation().matrix();
  std::cout<<tf*tf2<<"\n";
  //  visualize(pc_old,pc_new, cors);
  // pcl::transformPointCloud(pc_old,tpc,tf2);
  //cors.clear();
  //  visualize(tpc,pc_new, cors);

  PrecisionStopWatch psw;

  float rmax_ = sinf(rmax);
  rmax_*=rmax_;

  cors.clear();
  weight_o.resize(pc_old.size(), 0);
  weight_n.resize(pc_new.size(), 0);
  for(int i=0; i<pc_old.size(); i++) {
    float t = pc_old.points[i].getVector3fMap().norm();
    Eigen::Vector3f vo = pc_old.points[i].getVector3fMap();

    int num=0;
    float bestd;
    for(int j=search_sorted_vector(tv,t-tmax); j<tv.size(); j++) {

      float dT = std::abs(tv[j].dis-t);
      if( dT < tmax) {
        Eigen::Vector3f vn = pc_new.points[tv[j].ind].getVector3fMap();

        float dA;
        //dA = acosf(pc_old.points[i].getVector3fMap().dot(pc_new.points[tv[j].ind].getVector3fMap())/(pc_old.points[i].getVector3fMap().norm()*pc_new.points[tv[j].ind].getVector3fMap().norm()));
        dA = (((vn-vo).squaredNorm()
            /vo.squaredNorm()));

        if(dA>=rmax_) continue;

        /*
        if((tf2*pc_old.points[i].getVector4fMap()-pc_new.points[tv[j].ind].getVector4fMap()).squaredNorm()
            >=
            (pc_old.points[i].getVector3fMap()-pc_new.points[tv[j].ind].getVector3fMap()).squaredNorm()+tmax/2) {
          dbg_rem1++;
          continue;
        }

         */

        if(num>0 &&
            (vn-vo)
            .dot
            (pc_new[cors.back().ind_n].getVector3fMap()-vo)<0. )
        {
          cors.resize(cors.size()-num);
          dbg_rem2+=num;
          break;
        }
        ++num;

        COR_S c;
        c.ind_o = i;
        c.ind_n = tv[j].ind;
        //if(num==1) {
        bestd=dA*dA+dT*dT;
        cors.push_back(c);
        /*}
        else if(bestd>dA*dA+dT*dT) {

          if(
              (pc_new[c.ind_n].getVector3fMap()-pc_old.points[i].getVector3fMap())
              .dot
              (pc_new[cors.back().ind_n].getVector3fMap()-pc_old.points[i].getVector3fMap())<0. )
          {
            cors.resize(cors.size()-1);
            break;
          }

          bestd=dA*dA+dT*dT;
          cors.back()=c;
        }*/

        weight_o[i]++;
        weight_n[c.ind_n]++;
      }
      else if(tv[j].dis-t > tmax)
        break;
    }

    /*int ok=0;
    for(int j=0; j<num-1; j++) {
      Eigen::Vector3f v= pc_new[cors[cors.size()-j-1].ind_n].getVector3fMap()-pc_old.points[i].getVector3fMap();
      //v.normalize();
      for(int k=j+1; k<num; k++) {
        Eigen::Vector3f v2= pc_new[cors[cors.size()-k-1].ind_n].getVector3fMap()-pc_old.points[i].getVector3fMap();
        //v2.normalize();

        if(v2.dot(v)<0) {
          ok++;
        }

      }
    }
    if(ok>num*(num+1)/8) {
      dbg_rem2+=num;
      weight_o[i]-=num;
      for(int i=0; i<num; i++) {
        weight_n[cors[cors.size()-1-i].ind_n]--;
      }
      cors.resize(cors.size()-num);
    }*/

  }
  ROS_ERROR("took %f", psw.precisionStop());

  transFromCorr.reset();
  for(int i=0; i<cors.size(); i++) {
    transFromCorr.add(pc_old[cors[i].ind_o].getVector3fMap(), pc_new[cors[i].ind_n].getVector3fMap(), 1./std::min(weight_o[cors[i].ind_o],weight_n[cors[i].ind_n]));
  }
  tf2 = transFromCorr.getTransformation().matrix();
  ROS_ERROR("took %f", psw.precisionStop());

  std::cout<<tf*tf2<<"\n";
  // visualize(pc_old,pc_new, cors);
  pcl::transformPointCloud(pc_old,tpc,tf2);
  //cors.clear();
  std::cout<<"num "<<cors.size()<<"\n";
  std::cout<<"dbg_rem1 "<<dbg_rem1<<"\n";
  std::cout<<"dbg_rem2 "<<dbg_rem2<<"\n";
  visualize(tpc,pc_new, cors,weight_o,weight_n);


  {
    pcl::PointCloud<pcl::PointXYZRGB> pc1_,pc2_;
    pcl::io::loadPCDFile(_FN1_,pc1_);
    pcl::io::loadPCDFile(_FN2_,pc2_);

    pcl::transformPointCloud(pc1_,pc1_,tf*tf2);

    pcl::io::savePCDFileBinary("out/a.pcd",pc1_);
    pcl::io::savePCDFileBinary("out/b.pcd",pc2_);
  }

  if(tmax<0.01)
    return tf*tf2;
  return findTF(tpc,pc_new, rmax*0.5, tmax*0.5, tf*tf2);

  for(int i=0; i<weight_o.size(); i++) std::cout<<weight_o[i]<<" ";

  ROS_INFO("num %d",cors.size());

  tf=_findTF(pc_old, pc_new, cors, rmax,tmax,M_PI*0.5, weight_o,weight_n);
  ROS_ERROR("FINISHED!");
  visualize(pc_old,pc_new, cors);

  pcl::transformPointCloud(pc_old,tpc,tf);
  visualize(tpc,pc_new, cors);

  return tf;
}

template <typename Point>
class calcTF
{
  std::vector<SORT_S2> tv;
  const pcl::PointCloud<Point> &pc_old_;
  const pcl::PointCloud<Point> &pc_new_;
  const float rmax;
  const float tmax;
  const float mininfo;
  std::vector<float> dis_old, dis_new;
public:
  calcTF(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new):
    pc_old_(pc_old), pc_new_(pc_new),
    rmax(0.1), tmax(0.1), mininfo(5)
  {
    for(int i=0; i<pc_old.size(); i++) {
      float t = pc_old.points[i].getVector3fMap().norm();
      dis_old.push_back(t);
    }
    for(int i=0; i<pc_new.size(); i++) {
      float t = pc_new.points[i].getVector3fMap().norm();
      dis_new.push_back(t);
    }

    for(int i=0; i<pc_new.size(); i++) {
      SORT_S2 s;
      s.dis=dis_new[i];
      s.ind=i;
      tv.push_back(s);
    }

    SORT_S2 _s_;
    std::sort(tv.begin(),tv.end(), _s_);
  }

  void getWeights(const std::vector<int> &inds, std::vector<int> &weight_o, std::vector<int> &weight_n) {
    float __rmax = std::abs(cosf(rmax));
    weight_o.resize(pc_old_.size(), 0);
    weight_n.resize(pc_new_.size(), 0);
    for(int k=0; k<inds.size(); k++) {
      int i = inds[k];
      float t = dis_old[i];

      for(int j=search_sorted_vector(tv,t-tmax); j<tv.size(); j++) {

        if( std::abs(tv[j].dis-t) < tmax) {
          float dA;
          float t2 = dis_new[tv[j].ind];
          dA = std::abs(pc_old_.points[i].getVector3fMap().dot(pc_new_.points[tv[j].ind].getVector3fMap())/(t*t2));

          if(dA<__rmax) continue;
          if((pc_old_.points[i].getVector3fMap()-pc_new_.points[tv[j].ind].getVector3fMap()).squaredNorm()==0)
            continue;

          weight_o[i]++;
          weight_n[tv[j].ind]++;
        }
        else if(tv[j].dis-t > tmax)
          break;
      }
    }
  }

  Eigen::Matrix4f findCloud() {
    PrecisionStopWatch psw;

    pcl::PointCloud<Point> pc_o, pc_n;

    std::vector<int> weight_o, weight_n;
    std::vector<int> *pinds = new std::vector<int>();
    std::vector<int> &inds = *pinds;
    for(int i=0; i<pc_old_.size(); i++) inds.push_back(i);

    getWeights(inds,weight_o, weight_n);
    ROS_ERROR("took %f", psw.precisionStop());

    for(int i=0; i<inds.size(); i++) {
      if(weight_o[inds[i]]<mininfo) {
        std::swap(inds[i],inds[inds.size()-1]);
        inds.resize(inds.size()-1);
        pc_o.push_back(pc_old_[i]);
        i--;
      }
    }

    ROS_ERROR("took %f", psw.precisionStop());
    ROS_INFO("to check %d",inds.size());

    if(inds.size()>0) {
      boost::shared_ptr<pcl::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
      tree->setInputCloud (pc_old_.makeShared(),pcl::IndicesConstPtr(pinds));

      std::vector<bool> done;
      done.resize(inds.size(),false);
      std::vector<int> found;
      std::vector<float> k_sqr_distances;

      float __rmax = std::abs(cosf(rmax));
      //TODO: speed up   std::sort(inds.begin(),inds.end());
      for(int k=0; k<inds.size(); k++) {
        if(done[k]) continue;
        tree->radiusSearch(pc_old_[inds[k]],tmax*0.2f,found,k_sqr_distances);

        Point po;
        po.x=po.y=po.z=0;

        float s=0;
        float t=dis_old[inds[k]];
        for(size_t j=0; j<found.size(); j++) {
          int i=inds[found[j]];

          float dA;
          float t2 = dis_old[i];
          dA = std::abs(pc_old_.points[inds[k]].getVector3fMap().dot(pc_old_.points[i].getVector3fMap())/(t*t2));

          if(dA<__rmax || std::abs(t-t2)>tmax) continue;

          float w=1/(float)weight_o[i];
          s+=w;
          po.x+=pc_old_[i].x*w;
          po.y+=pc_old_[i].y*w;
          po.z+=pc_old_[i].z*w;

          //remove
          done[found[j]]=true;

        }

        po.x/=s;
        po.y/=s;
        po.z/=s;

        //po=pc_old[inds[k]];
        pc_o.push_back(po);

        /*weight_o.clear();
      weight_n.clear();
      getWeights(found,weight_o, weight_n);

      Point po,pn;
      po.x=po.y=po.z=0;
      pn=po;

      float s=0;
      for(int i=0; i<weight_n.size(); i++) {
        if(weight_n[i]<1) continue;

        float w=1/(float)weight_n[i];
        s+=w;
        pn.x+=pc_new[i].x*w;
        pn.y+=pc_new[i].y*w;
        pn.z+=pc_new[i].z*w;
      }
      pn.x/=s;
      pn.y/=s;
      pn.z/=s;
      ROS_ERROR("s %f",s);

      s=0;
      for(int j=0; j<found.size(); j++) {
        int i=found[j];
        if(weight_o[i]>0) {
          float w=1/(float)weight_o[i];
          s+=w;
          po.x+=pc_old[i].x*w;
          po.y+=pc_old[i].y*w;
          po.z+=pc_old[i].z*w;
        }
        //remove
        done[i]=true;

      }
      ROS_ERROR("s %f",s);

      po.x/=s;
      po.y/=s;
      po.z/=s;

      std::cout<<po<<" "<<pn<<"\n";
      pc_o.push_back(po);
      pc_n.push_back(pn);*/
      }
    }

    ROS_ERROR("took %f", psw.precisionStop());

    ROS_INFO("%d -> %d",pc_old_.size(), pc_o.size());

    //return findTF(pc_o,pc_new_);
    //return findTF(pc_old_,pc_new_);
    return ::findTF(pc_old_,pc_new_);
  }


  Eigen::Matrix4f findTF(const pcl::PointCloud<Point> &pco, const pcl::PointCloud<Point> &pcn)
  {
    std::vector<COR_S> cors;
    PrecisionStopWatch psw;

    dis_old.clear();
    for(int i=0; i<pco.size(); i++) {
      float t = pco.points[i].getVector3fMap().norm();
      dis_old.push_back(t);
    }

    std::vector<int> weight_o, weight_n;
    weight_o.resize(pco.size(),0);
    weight_n.resize(pcn.size(),0);

    float __rmax = std::abs(cosf(rmax));
    for(int i=0; i<pco.size(); i++) {
      float t = dis_old[i];

      for(int j=search_sorted_vector(tv,t-tmax); j<tv.size(); j++) {

        if( std::abs(tv[j].dis-t) < tmax) {
          float dA;
          float t2 = dis_new[tv[j].ind];
          dA = std::abs(pco.points[i].getVector3fMap().dot(pcn.points[tv[j].ind].getVector3fMap())/(t*t2));

          if(dA<__rmax) continue;
          if((pco.points[i].getVector3fMap()-pcn.points[tv[j].ind].getVector3fMap()).squaredNorm()==0)
            continue;

          COR_S c;
          c.ind_o = i;
          c.ind_n = tv[j].ind;
          //      c.dis = (pco.points[i].getVector3fMap()-pcn.points[tv[j].ind].getVector3fMap()).norm();
          cors.push_back(c);

          weight_o[i]++;
          weight_n[tv[j].ind]++;
        }
        else if(tv[j].dis-t > tmax)
          break;
      }

    }
    ROS_ERROR("took %f", psw.precisionStop());

    Eigen::Matrix4f tf=_findTF(pco, pcn, cors, rmax,tmax,M_PI*0.5, weight_o,weight_n);
    ROS_ERROR("took %f", psw.precisionStop());

    std::cout<<tf<<"\n";
    ROS_ERROR("FINISHED!");
    visualize(pco,pcn, cors);

    pcl::PointCloud<Point> tpc;
    pcl::transformPointCloud(pco,tpc,tf);
    cors.clear();
    visualize(tpc,pcn, cors);
    return tf;
  }

private:
  Eigen::Matrix4f _findTF(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new, std::vector<COR_S> &cors, float border_a, float border_t, float border_a2,
                          std::vector<int> &weight_o, std::vector<int> &weight_n, const Eigen::Vector3f &prev_trans=Null3f()) {

    Eigen::Vector3f mid_old, mid_new, mid_ob, mid_nb, mid_oa, mid_na, axis;
    int rem = 0;

    GET_MIDS();

    std::cout<<"mid_old\n"<<mid_old<<"\n";
    std::cout<<"mid_new\n"<<mid_new<<"\n";
    std::cout<<"mid_ob\n"<<mid_ob<<"\n";
    std::cout<<"mid_oa\n"<<mid_oa<<"\n";
    std::cout<<"mid_nb\n"<<mid_nb<<"\n";
    std::cout<<"mid_na\n"<<mid_na<<"\n";

    axis=getAxis2(pc_old, pc_new, cors, mid_old.squaredNorm(), mid_old, mid_new, weight_o, weight_n);
    std::cout<<"axis\n"<<axis<<"\n";
    Eigen::Matrix4f tf=getTF2(axis, mid_oa,mid_ob,mid_old, mid_na,mid_nb,mid_new);

    //std::cout<<tf<<"\n";

    if( (tf.col(3).head<3>()).squaredNorm() > 0.04) {
      tf.col(3).head<3>()=tf.col(3).head<3>()*0.2/tf.col(3).head<3>().norm();
    }

    bool bGoOn=false;
    do {
      std::cout<<"tmax: "<<border_t<<"\n";
      rem += remError(pc_old, pc_new, cors, tf, border_a, border_t, weight_o,weight_n);

      border_a*=0.5f;
      border_t*=0.5f;
      border_a2*=0.5f;

      bGoOn = border_t>0.01&&cors.size()>std::min(pc_new.size(),pc_old.size());
    } while(rem<1&&bGoOn);

    if(bGoOn)
      return _findTF(pc_old,pc_new,cors, border_a, border_t, border_a2, weight_o, weight_n, tf.col(3).head<3>() );//0.5*(tf.col(3).head<3>()+prev_trans) );

    GET_MIDS();
    axis=getAxis2(pc_old, pc_new, cors, mid_old.squaredNorm(), mid_old, mid_new, weight_o, weight_n);
    tf=getTF2(axis, mid_oa,mid_ob,mid_old, mid_na,mid_nb,mid_new);

    return tf;
  }

  int remError(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new,
               std::vector<COR_S> &cors, const Eigen::Matrix4f &tf, const float border_a, const float border_t, std::vector<int> &weight_o,std::vector<int> &weight_n) {
    int n=0;
    for(int i=0; i<cors.size(); i++) {
      Eigen::Vector4f v=(tf*pc_old[cors[i].ind_o].getVector4fMap());
      Eigen::Vector4f v2=pc_new[cors[i].ind_n].getVector4fMap();

      float dT = std::abs(v.norm()-v2.norm());
      float dA = acosf(v.dot(v2)/(v.norm()*v2.norm()));

      if( dA>border_a || dT>border_t ) {
        weight_o[cors[i].ind_o]--;
        weight_n[cors[i].ind_n]--;

        std::swap(cors[i],cors.back());
        cors.resize(cors.size()-1);
        --i;
        ++n;
      }
    }

    return n;
  }

  Eigen::Matrix4f getTF2(const Eigen::Vector3f &axis, const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p_s, const Eigen::Vector3f &p1n, const Eigen::Vector3f &p2n, const Eigen::Vector3f &pn_s) {
    Eigen::Quaternionf q;

    Eigen::Vector3f Z, a,b;
    Z(0)=0;
    Z(1)=0;
    Z(2)=1;

    a=p1-p2;
    b=p1n-p2n;

    a = a.cross(axis);
    b = b.cross(axis);

    a.normalize();
    b.normalize();

    float angle = acos(a.dot(b));

    Eigen::AngleAxisf rot(angle, axis);

    Eigen::Matrix3f R=rot.toRotationMatrix();

    Eigen::Vector3f t=pn_s-R*p_s;

    Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
    for(int x=0; x<3; x++)
      for(int y=0; y<3; y++)
        transf(x,y) = R(x,y);
    transf.col(3).head<3>() = t;

    return transf;
  }

  Eigen::Vector3f getAxis2(const pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new,
                           const std::vector<COR_S> &cors, const float max_dis,
                           const Eigen::Vector3f &rel_old, const Eigen::Vector3f &rel_new
                           , std::vector<int> &weight_o,std::vector<int> &weight_n) {
    Eigen::Vector3f m[4];
    int g[4]={};

    m[0] = Null3f();
    m[1] = Null3f();
    m[2] = Null3f();
    m[3] = Null3f();

    Eigen::Vector3f X,Y;

    X(0)=1;
    X(1)=X(2)=0;
    Y(1)=1;
    Y(2)=Y(0)=0;

    X=rel_old.cross(X);
    Y=rel_old.cross(Y);

    for(int i=0; i<cors.size(); i++) {
      Eigen::Vector3f v,vp;

      v =pc_old[cors[i].ind_o].getVector3fMap();
      vp=pc_new[cors[i].ind_n].getVector3fMap();

      v -=rel_old;
      vp-=rel_new;

      int q=0;

      q+=v.dot(X)>0?1:0;
      q+=v.dot(Y)<0?2:0;

      v=vp-v;
      //v.normalize();

      if(pc_old[cors[i].ind_o].getVector3fMap().squaredNorm()<rel_old.squaredNorm()) v=-v;

      //v/=cors[i].dis;
      m[q]+=v;//  / (float)(weight_o[cors[i].ind_o]); //weight by I
      g[q]++;//=1.f/ (float)(weight_o[cors[i].ind_o]); //weight by I

    }

    m[0]/=g[0];
    m[1]/=g[1];
    m[2]/=g[2];
    m[3]/=g[3];

    std::cout<<"m[0]\n"<<m[0]<<"\n";
    std::cout<<"m[1]\n"<<m[1]<<"\n";
    std::cout<<"m[2]\n"<<m[2]<<"\n";
    std::cout<<"m[3]\n"<<m[3]<<"\n";

    Eigen::Vector3f axis, axis2;

    /*Eigen::Matrix3f aX, aY, aZ;

    aY=aZ=aX=aX.Identity();

    aX(0,0)=0;
    aY(1,1)=0;
    aZ(2,2)=0;

    Eigen::Vector3f ax,ay,az;
    ax=(m[0]+m[1]).cross(m[2]+m[3]);
    ay=(m[0]+m[2]).cross(m[1]+m[3]);
    az=(m[0]-m[3]).cross(m[1]-m[3]);

    axis(0) = ( (aX*(m[0]+m[1])).cross(aX*(m[2]+m[3])).norm() * (std::abs(ax(0))+std::abs(ay(0))+std::abs(az(0))) );
    axis(1) = ( (aY*(m[0]+m[2])).cross(aY*(m[1]+m[3])).norm() * (std::abs(ax(1))+std::abs(ay(1))+std::abs(az(1))) );
    axis(2) = ( (aZ*(m[0]-m[3])).cross(aZ*(m[1]-m[2])).norm() * (std::abs(ax(2))+std::abs(ay(2))+std::abs(az(2))) );

    axis(0) = (std::abs(ax(0))+std::abs(ay(0))+std::abs(az(0))) ;
    axis(1) = (std::abs(ax(1))+std::abs(ay(1))+std::abs(az(1))) ;
    axis(2) = (std::abs(ax(2))+std::abs(ay(2))+std::abs(az(2))) ;*/

    Eigen::Vector3f ax,ay,az;
    ax=(m[0]+m[1]).cross(m[2]+m[3]);
    ay=(m[0]+m[2]).cross(m[1]+m[3]);
    az=(m[0]-m[3]).cross(m[1]-m[3]);

    ax.normalize();
    ay.normalize();
    az.normalize();

    axis(0) = std::abs( ax(0) );
    axis(1) = std::abs( ay(1) );
    axis(2) = std::abs( az(2) );

    axis.normalize();

    if( m[0](1)+m[1](1)+m[2](1)+m[3](1) > 0)
      axis(0) = -axis(0);
    if( m[0](0)+m[1](0)+m[2](0)+m[3](0) < 0)
      axis(1) = -axis(1);
    if( m[0](0)*m[0](1)+m[3](0)*m[3](1) - m[1](0)*m[1](1)-m[2](0)*m[2](1)
        < 0)
      axis(2) = -axis(2);

    return axis;
  }
};

Eigen::Matrix4f findCorr3_tf(const pcl::PointCloud<MY_POINT> &pc_old, const pcl::PointCloud<MY_POINT> &pc_new, std::vector<int> inds_old, std::vector<int> inds, const int i_sz_old, const int i_sz) {
  if(!i_sz_old||!i_sz)
    return Eigen::Matrix4f::Identity();

  Eigen::Vector3f mid_s1=pc_old.points[inds_old[0]].getVector3fMap();
  Eigen::Vector3f mid_t1=pc_new.points[inds[0]].getVector3fMap();
  float mi_s=10000,mi_t=100000, ma_s=0, ma_t=0;
  for(int i=1; i<i_sz_old; i++) {

    if(!pcl_isfinite(pc_old.points[inds_old[i]].z))
      ROS_ERROR("fuck");

    mid_s1+=pc_old.points[inds_old[i]].getVector3fMap();
    mi_s=std::min(mi_s,pc_old.points[inds_old[i]].z);
    ma_s=std::max(ma_s,pc_old.points[inds_old[i]].z);
  }
  for(int i=1; i<i_sz; i++) {

    if(!pcl_isfinite(pc_new.points[inds[i]].z))
      ROS_ERROR("fuck");

    mid_t1+=pc_new.points[inds[i]].getVector3fMap();
    mi_t=std::min(mi_t,pc_new.points[inds[i]].z);
    ma_t=std::max(ma_t,pc_new.points[inds[i]].z);
  }

  ROS_INFO("num %d %d", i_sz_old,i_sz);

  mid_s1/=i_sz_old;
  mid_t1/=i_sz;

  std::cout<<"mt1\n"<<mid_t1<<"\n";
  std::cout<<"ms1\n"<<mid_s1<<"\n";

  Eigen::Vector3f mid_s, mid_t, mid_s2, mid_t2;

  mid_s(0)=mid_s(1)=mid_s(2)=0;

  mid_t=mid_t2=mid_s2=mid_s;

  int n1=0,n2=0;
  for(int i=0; i<i_sz_old; i++) {
    if(inds_old[i]>=pc_old.size())
      ROS_INFO("oh man");
    //std::cout<<pc_old.points[inds_old[i]]<<"\n";
    //std::cout<<mid_s<<"\n";
    if(mid_s1.squaredNorm()<pc_old.points[inds_old[i]].getVector3fMap().squaredNorm()) {
      mid_s+=pc_old.points[inds_old[i]].getVector3fMap();
      n1++;
    }
    else {
      mid_s2+=pc_old.points[inds_old[i]].getVector3fMap();
      n2++;
    }
  }
  mid_s/=n1;
  mid_s2/=n2;

  std::cout<<"n1\n"<<n1<<"\n";
  std::cout<<"n2\n"<<n2<<"\n";
  std::cout<<"ms\n"<<mid_s<<"\n";
  std::cout<<"ms2\n"<<mid_s2<<"\n";

  if(!n1||!n2) return Eigen::Matrix4f::Identity();

  n1=0,n2=0;
  for(int i=0; i<i_sz; i++) {
    if(inds[i]>=pc_new.size())
      ROS_INFO("oh man");
    if(mid_s1.squaredNorm()<pc_new.points[inds[i]].getVector3fMap().squaredNorm()) {
      mid_t+=pc_new.points[inds[i]].getVector3fMap();
      n1++;
    }
    else {
      mid_t2+=pc_new.points[inds[i]].getVector3fMap();
      n2++;
    }
  }
  mid_t/=n1;
  mid_t2/=n2;

  Eigen::Matrix3f R;

  std::cout<<"mt\n"<<mid_t<<"\n";
  std::cout<<"mt2\n"<<mid_t2<<"\n";
  std::cout<<"n1\n"<<n1<<"\n";
  std::cout<<"n2\n"<<n2<<"\n";

  if(!n1||!n2) return Eigen::Matrix4f::Identity();

  Eigen::Quaternionf q;
  q.setFromTwoVectors(mid_t-mid_t2,mid_s-mid_s2);
  Eigen::Vector3f v;
  v(0)=v(1)=0;
  v(2)=1;
  v=q.toRotationMatrix()*v;
  ROS_INFO("v %f %f %f",v(0),v(1),v(2));

  R=q.toRotationMatrix();

  Eigen::Vector3f t;

  /*t=mid_s-R*mid_t;
  t+=mid_s2-R*mid_t2;

  t*=0.5;*/
  t=mid_t1-R*mid_s1;

  Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();
  for(int x=0; x<3; x++)
    for(int y=0; y<3; y++)
      transf(x,y) = R(x,y);
  transf.col(3).head<3>() = t;

  std::cout<<(mid_s1-R*mid_t1)<<"\n";
  std::cout<<(mid_t1-R*mid_s1)<<"\n";
  std::cout<<transf<<"\n";

  boost::shared_ptr<pcl::KdTree<MY_POINT> > tree (new pcl::KdTreeFLANN<MY_POINT>);
  tree->setInputCloud (pc_new.makeShared());
  boost::shared_ptr<pcl::KdTree<MY_POINT> > tree2 (new pcl::KdTreeFLANN<MY_POINT>);
  tree2->setInputCloud (pc_old.makeShared());

  float dis=0;
  int nn=0;
  for(int j=0; j<inds_old.size(); j++) {
    int i=inds_old[j];

    std::vector< int > k_indices;
    std::vector< float > k_distances;
    MY_POINT p=pc_old[i];
    Eigen::Vector4f v=transf.inverse()*pc_old[i].getVector4fMap();
    p.x=v(0);
    p.y=v(1);
    p.z=v(2);
    if(!tree->nearestKSearch(p,1,k_indices,k_distances))
    {
      //std::cout<<"- ";
      continue;
    }

    dis+= sqrtf(k_distances[0]);
    nn++;
  }
  dis/=nn;
  dis*=1.5;
  dis=std::max(dis,0.05f);

  for(int j=0; j<inds_old.size(); j++) {
    int i=inds_old[j];

    std::vector< int > k_indices;
    std::vector< float > k_distances;
    MY_POINT p=pc_old[i];
    Eigen::Vector4f v=pc_old[i].getVector4fMap();
    p.x=v(0);
    p.y=v(1);
    p.z=v(2);
    if(!tree->nearestKSearch(p,1,k_indices,k_distances))
    {
      //std::cout<<"- ";
      continue;
    }

    //k_distances[0]=(pc_old[i].getVector3fMap()-pc_new[k_indices[0]].getVector3fMap()).squaredNorm();
    if(sqrtf(k_distances[0])>10) {
      std::cout<<"\n"<<pc_old[i].getVector3fMap()<<"("<<k_distances.size()<<")\n";
      std::cout<<"\n"<<pc_new[k_indices[0]].getVector3fMap()<<"\n";
    }

    //std::cout<<sqrtf(k_distances[0])<<" ";
    k_distances[0] = (transf.inverse()*pc_old[i].getVector4fMap()-pc_new[k_indices[0]].getVector4fMap()).squaredNorm();
    if(sqrtf(k_distances[0])>dis) {
      inds_old.erase(inds_old.begin()+j);
      --j;
    }
  }

  for(int j=0; j<inds.size(); j++) {
    int i=inds[j];

    std::vector< int > k_indices;
    std::vector< float > k_distances;
    MY_POINT p=pc_new[i];
    Eigen::Vector4f v=pc_new[i].getVector4fMap();
    p.x=v(0);
    p.y=v(1);
    p.z=v(2);
    if(!tree2->nearestKSearch(p,1,k_indices,k_distances))
    {
      //std::cout<<"- ";
      continue;
    }

    k_distances[0] = (transf*pc_new[i].getVector4fMap()-pc_old[k_indices[0]].getVector4fMap()).squaredNorm();
    if(sqrtf(k_distances[0])>dis) {
      inds.erase(inds.begin()+j);
      --j;
    }
  }

  ROS_INFO("-->  %d %d %f", inds_old.size(), i_sz_old, dis);

  getchar();

  //pcl::transformPointCloud(pc_new,pc_new,transf);

  if( inds_old.size() != i_sz_old && dis>0.06)
    return findCorr3_tf(pc_new,pc_old, inds,inds_old, inds.size(),inds_old.size()).inverse();

  return transf;
}

class TestNode
{
#ifdef RGB_
  typedef pcl::PointXYZRGB Point;
#else
  typedef pcl::PointXYZ Point;
#endif

  static void eigen2point(Point &p, const Eigen::Vector3f &e) {
    p.x = e(0);
    p.y = e(1);
    p.z = e(2);
  }

  static void point2eigen(Eigen::Vector3f &e, const Point &p) {
    e(0) = p.x;
    e(1) = p.y;
    e(2) = p.z;
  }

  static void point2eigen(Eigen::VectorXf &e, const Point &p) {
    e(0) = p.x;
    e(1) = p.y;
    e(2) = p.z;
  }

  static void point2eigen(Eigen::Vector4f &e, const Point &p) {
    e(0) = p.x;
    e(1) = p.y;
    e(2) = 0;//p.z;
    e(3) = 0;
  }


  struct Polyline {
    std::vector<Eigen::Vector4f> pl;
    float area_;

    void findPolyline(pcl::PointCloud<Point> pc) {

      pcl::VoxelGrid<Point> voxel;
      voxel.setInputCloud(pc.makeShared());
      voxel.setLeafSize(0.01,0.01,0.01);
      voxel.filter(pc);

      // Create the filtering object
      pcl::StatisticalOutlierRemoval<Point> sor;
      sor.setInputCloud (pc.makeShared());
      sor.setMeanK (10);
      sor.setStddevMulThresh (0.025);
      sor.filter (pc);

      if(pc.points.size()<3)
        return;

      if(g_step) {
        sensor_msgs::PointCloud2 pc_out;
        pcl::toROSMsg(pc,pc_out);
        pc_out.header = header_;
        point_cloud_pub_.publish(pc_out);
      }

      Eigen::Vector4f mip, map, centroid;

      pcl::getMinMax3D(pc, mip, map);
      pcl::compute3DCentroid(pc, centroid);

      pl.insert(pl.end(), mip);
      pl.insert(pl.end(), map);

      std::vector<bool> blacklist;
      blacklist.push_back(false);
      blacklist.push_back(false);

      //addPoint(pc,rand()%pc.points.size(),0);
      //addPoint(pc,rand()%pc.points.size(),0);

      //find min dist. to line
      float max_dis, last_dis=10000;
      int iteration=0;
      //std::cout<<"pl: starting\n";
      do {

        //centroid(0)=centroid(1)=centroid(2)=centroid(3)=0;

        std::cout<<"pl: round "<<iteration<<"\n";
        for(int i=0; i<pl.size(); i++) {
          //centroid+=pl[i];
          std::cout<<"pl: \t"<<pl[i](0)<<", "<<pl[i](1)<<", "<<pl[i](2)<<"\n";
        }
        //centroid/=pl.size();
        max_dis=-1;

        float max_f=-1, avg = 0;
        int max_j, max_i=-1;

        std::vector<int> support(pl.size(),0);
        std::vector<int> not_support(pl.size(),0);
        std::vector<int> seg_i(pl.size(),-1);
        std::vector<float> seg_f(pl.size(),0.f);
        for(int i=0; i<pc.points.size(); i++) {
          Eigen::Vector4f v;
          point2eigen(v, pc.points[i]);

          float f=1000;
          int jm = -1;
          int found=0;
          for(int j=0; j<pl.size(); j++) {

            Eigen::Vector4f m,
            p = v-centroid,
            a=pl[j]-centroid,
            b=(pl[ (j+1)%pl.size() ])-centroid;
            //if( pl.size()>4 && p.norm()>((a+b)*0.5).norm()/2 ) continue;

            //p(2)=a(2)=b(2)=0;
            p.normalize();
            b.normalize();
            a.normalize();
            m=(a+b)/2;
            m.normalize();
            float c = (a-m).norm();
            //ROS_INFO("pl: %f %f", (m-p).norm(),c);
            if( pl.size()>2 && ((m-p).norm()>c || p.dot(m)<0 ) ) continue;
            found++;
            //ROS_INFO("pl: %f %f", (m-p).norm(),c);
            //if( a>c+b || b>c+a) continue;*/

            float t = pcl::sqrPointToLineDistance(v,pl[j],pl[ (j+1)%pl.size() ]-pl[j]);
            if(t<f||jm==-1)
            {

              /*if(SameSide(_4to3_(centroid),_4to3_(v), _4to3_(pl[j]), _4to3_(pl[ (j+1)%pl.size() ]))) {
                int num=0;
                for(int k=0; k<pc.points.size(); k++) {
                  Eigen::Vector4f t;
                  point2eigen(t, pc.points[k]);
                  if(PointInTriangle(t, pl[j], (pl[ (j+1)%pl.size() ]), v))
                    num++;
                }
                if(num>3) {
                  ROS_INFO("pl: not accepted as %d inliers",num);
                  continue;
                }
              }*/

              f=t;
              jm = j;
            }
          }

          avg+=f;

          if(jm!=-1) {
            if(f>0.05*0.05)
              not_support[jm]++;
            else
              support[jm]++;
            if(seg_f[jm]<f) {
              seg_f[jm]=f;
              seg_i[jm]=i;
            }
          }

          if(f>max_f && jm!=-1 && !blacklist[jm]) {
            max_dis = max_f;
            max_f=f;
            max_j=jm;
            max_i=i;
          }

        }

        avg/=pc.size();

        float pl_min=1000;int pl_i=-1;
        for(int i=0; i<pl.size(); i++) {
          std::cout<<"pl: \t"<<support[i]<<" "<<not_support[i]<<" -> "<<support[i]*100./not_support[i]<<"%\n";
          if(support[i]-10>not_support[i] && iteration>2)
            blacklist[i]=true;
          else
            blacklist[i]=false;

          Eigen::Vector4f
          a=pl[i],
          b=(pl[ (i+1)%pl.size() ]);
          float len = (a-b).norm();

          ROS_INFO("pl: %f %f %d %d", pl_min, (1+support[i])/(float)(not_support[i]+100)/len, seg_i[i],(int)blacklist[i]);
          if( pl_min> (1+support[i])/(float)(not_support[i]+100)/len && seg_i[i]!=-1 && !blacklist[i] ) {
            //if( pl_min<(a-b).norm() && seg_i[i]!=-1 && !blacklist[i] ) {
            pl_min=(1+support[i])/(float)(not_support[i]+100)/len;
            pl_i=i;
          }
        }

        if(pl_i!=-1 && iteration>2) {
          max_f = seg_f[pl_i];
          max_i = seg_i[pl_i];
          max_j = pl_i;
        }

        ROS_INFO("pl: dis %f %f -- %d %d", max_f, avg, max_j, pl_i);
        if(max_i!=-1) {
          /*if(last_dis<max_f && pl.size()>2) {
            //pc.points.erase(pc.points.begin()+max_i);
            blacklist[max_j]=true;
            continue;
          }
          else {
            for(int i=0; i<blacklist.size(); i++)
              blacklist[i]=false;
          }*/

          if( blacklist[max_j] ) continue;

          addPoint(pc,max_i,(max_j+1)%(pl.size()));
          blacklist.push_back(false);
          last_dis = max_f;
        }
        else
          break;

        /*if(iteration==0) {
          pl.insert(pl.end(), pl.front());
        }
        else if(iteration==1) {
          pl.erase(pl.begin());
          pl.erase(pl.begin());
        }*/
        if(iteration==1) {
          for(int j=0; j<pl.size(); j++) {
            if(pl[j]==mip||pl[j]==map) {
              pl.erase(pl.begin()+j);
              --j;
            }
          }
        }
        ++iteration;

        //resort();
        publishLineMarker(pl,-343);
        if(g_step&&getchar()=='q')
          exit(0);

        //if(iteration==1) break;

      } while(last_dis>0.01*0.01&&pl.size()<16&&pc.size()>0);//max_dis>0.01);

      if(g_step&&getchar()=='q')
        exit(0);

      {
        float avg=0;
        for(int i=0; i<pl.size();i++) {
          Eigen::Vector4f
          a=pl[i],
          b=(pl[ (i+1)%pl.size() ]);
          float len = (a-b).norm();
          avg+=len;
        }
        avg/=pl.size();

        for(int i=0; i<pl.size();i++) {
          Eigen::Vector4f
          a=pl[i],
          b=(pl[ (i+1)%pl.size() ]),
          c=(pl[ (i+2)%pl.size() ]);

          float len1 = (a-b).norm();
          float len2 = (c-b).norm();

          if(len1+len2<avg*2) {
            int r = (i+1)%pl.size();
            pl.erase(pl.begin()+r);
          }

        }

        float area=0;
        for(int i=0; i<pl.size();i++) {
          Eigen::Vector4f
          a=pl[i]-centroid,
          b=(pl[ (i+1)%pl.size() ])-centroid;

          area+=_4to3_(a).cross(_4to3_(b)).norm();
        }
        area*=100*100;
        ROS_INFO("area: %f", area);

        area_ = area;

      }

    }

    void show() {
      float max_alpha=-1;
      int offset=0;
      std::string r="";
      for(int j=0; j<pl.size(); j++) {
        Eigen::Vector4f a = pl[j]-pl[ (j+1)%pl.size() ];
        Eigen::Vector4f b = pl[ (j+1)%pl.size() ]-pl[ (j+2)%pl.size() ];
        if(alpha(a,b)>max_alpha) {
          max_alpha = alpha(a,b);
          offset=j;
        }
      }

      std::cout<<"pl-info: ";
      for(int i=0; i<pl.size(); i++) {
        int j = i+offset;

        Eigen::Vector4f a = pl[j%pl.size()]-pl[ (j+1)%pl.size() ];
        Eigen::Vector4f b = pl[ (j+1)%pl.size() ]-pl[ (j+2)%pl.size() ];

        std::cout<<alpha(a,b)<<" ";

        float t = alpha(a,b);
        if(t>1.7)
          r+="+";
        else if(t<1.3)
          r+="-";
        else
          r+="~";
      }
      std::cout<<r<<"\n";

    }

    float alpha(const Eigen::Vector4f &a, const Eigen::Vector4f &b) {
      return acosf(a.dot(b)/(a.norm()*b.norm()));
    }

    bool SameSide(const Eigen::Vector3f &p1, const Eigen::Vector3f  &p2, const Eigen::Vector3f  &a,const Eigen::Vector3f  &b) {
      Eigen::Vector3f cp1 = (b-a).cross(p1-a);
      Eigen::Vector3f cp2 = (b-a).cross(p2-a);
      if(cp1.dot(cp2) >= 0)
        return true;
      return false;
    }

    Eigen::Vector3f _4to3_(const Eigen::Vector4f &p) {
      Eigen::Vector3f r;
      r(0)=p(0);
      r(1)=p(1);
      r(2)=p(2);
      return r;
    }
    bool PointInTriangle(Eigen::Vector4f &pp, Eigen::Vector4f &aa,Eigen::Vector4f &bb,Eigen::Vector4f &cc) {
      Eigen::Vector3f p=_4to3_(pp);
      Eigen::Vector3f a=_4to3_(aa);
      Eigen::Vector3f b=_4to3_(bb);
      Eigen::Vector3f c=_4to3_(cc);

      if (SameSide(p,a, b,c) && SameSide(p,b, a,c)
          && SameSide(p,c, a,b) )
        return true;
      return false;
    }


    void resort() {
      std::vector<Eigen::Vector4f> t;
      t.push_back(pl[0]);
      pl.erase(pl.begin());

      while(pl.size()) {
        int w=0;
        for(int j=1; j<pl.size(); j++) {
          if( (pl[w]-t.back()).norm()>(pl[j]-t.back()).norm() )
            w=j;
        }

        t.push_back(pl[w]);
        pl.erase(pl.begin()+w);
      }

      pl = t;
    }

    void addPoint(pcl::PointCloud<Point> &pc, const int i, const int j) {
      Eigen::Vector4f v;
      point2eigen(v, pc.points[i]);
      pl.insert(pl.begin()+j, v);
      pc.points.erase(pc.points.begin()+i);
    }

  };

  struct DualFloat {
    float val[2];
  };

  Point movement_overall;
  Eigen::Vector3f movement_rotation, old_t_;
  Eigen::Matrix3f old_q_;

  ros::Subscriber settings_sub;
  bool show_labels_, show_markers_;
#ifdef RECONF_
  dynamic_reconfigure::Server<dynamic_tutorials::serverConfig> srv;
#endif

  float min_alpha_, max_alpha_;
  float min_thr_;
  int factor_;
  float min_thr_factor_, min_alpha_factor_, max_alpha_factor_;

public:
  // Constructor
  TestNode():show_labels_(true), show_markers_(true),
  min_alpha_(0.3), max_alpha_(0.5), min_thr_(-0.01),
  min_thr_factor_(0.005), min_alpha_factor_(0.03), max_alpha_factor_(0.03),
  factor_(2)
  {
#ifndef RGB_
    point_cloud_sub_ = n_.subscribe("/camera/depth/points", 1, &TestNode::pointCloudSubCallback, this);
#else
    point_cloud_sub_ = n_.subscribe("/camera/rgb/points", 1, &TestNode::pointCloudSubCallback, this);
#endif
    point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2 >("point_cloud2_resized",1);
    map_pub_ = n_.advertise<sensor_msgs::PointCloud2 >("point_cloud_map",1);
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("markers", 1);
    settings_sub = n_.subscribe<std_msgs::String>("command", 1000, &TestNode::commandCallback, this);


    old_q_ = Eigen::Matrix3f::Identity();
    old_t_(0)=old_t_(1)=old_t_(2)=0;


#if 0
    {
      Eigen::Vector3f p1a,p1b,  p2a, p2b;

      p1a(0) = 1;
      p1a(1) = 0;
      p1a(2) = 0;

      p1b(0) = 2;
      p1b(1) = 0;
      p1b(2) = 0;

      p2a(0) = 1;
      p2a(1) = 1;
      p2a(2) = 0;

      p2b(0) = 2;
      p2b(1) = 1;
      p2b(2) = 0;

      /*p1a(0) = 1;
    p1a(1) = 0;
    p1a(2) = 0;

    p1b(0) = 0;
    p1b(1) = 1;
    p1b(2) = 0;

    p2a(0) = 0;
    p2a(1) = 1;
    p2a(2) = 0;

    p2b(0) = -1;
    p2b(1) = 0;
    p2b(2) = 0;*/

      Eigen::Vector3f u = p1b-p1a, v=p2b-p2a;

      Eigen::Quaternionf q;
      q.setFromTwoVectors(u,v);

      Eigen::Vector3f t = p1b-q.toRotationMatrix()*p1a;

      std::cout<<u<<std::endl;
      std::cout<<v<<std::endl;
      std::cout<<q.toRotationMatrix()<<std::endl;
      std::cout<<t<<std::endl;
    }
    exit(0);
#endif
  }

  void commandCallback(const std_msgs::String::ConstPtr& msg)
  {
    ROS_INFO("I heard: [%s]", msg->data.c_str());

  }


  // Destructor
  ~TestNode()
  {
    /// void
  }


#define getInd(x, y) ((x)+(y)*pc.width)


  void
  pointCloudSubCallback(const sensor_msgs::PointCloud2ConstPtr& pc_in)
  {
    boost::timer _timer;

    header_ = pc_in->header;

    pcl::PointCloud<Point> pc;
    pcl::fromROSMsg(*pc_in,pc);


    static pcl::PointCloud<Point> pc_old;

    static int snn=0;
    snn++;
    if(g_step&&snn%30) return;

    if(pc_old.size()==pc.size())
    {

      static Eigen::Matrix4f Stransf = Eigen::Matrix4f::Identity();
      static unsigned char *depth_map=new unsigned char[pc.height*pc.width];

      //diff
      float diff1=0.03;
      float diff2=0.02;
      float diff3=0.03;

      diff3=diff2=diff1=0.01;

      std::vector<int> indices_pos, indices_neg;

      memset(depth_map, 0, pc.height*pc.width);
      pcl::PointCloud<Point> _p;
      for(int y=0; y<pc.height; y++) {
        for(int x=0; x<pc.width; x++) {

          int ind = getInd(x,y);
          Point &pn=pc.points[ind];
          Point &po=pc_old.points[ind];

          //if(!pcl_isfinite(pn.z)) pn.z=1000;
          //if(!pcl_isfinite(po.z)) po.z=1000;

          float d = pn.z-po.z;
          float di= std::min(pn.z, po.z);
          di = di*di*diff3;
          if(d>di) {
            depth_map[ind]=1;
            indices_pos.push_back(ind);}
          else if(d<-di) {
            depth_map[ind]=1;
            indices_neg.push_back(ind);
          }

          Point pp=po;
          pp.b=pp.g=std::min(255.f,256*std::abs(d));
          pp.r=std::min(255.f,256*std::abs(d/di));
          if(d>di||d<-di)
            _p.push_back(pp);

        }
      }
      {
        static int it=0;
        ++it;
        char buffer[128];
        sprintf(buffer,"_p%d.pcd",it);
        pcl::io::savePCDFile(buffer,_p);
      }


      ROS_INFO("found %d %d", indices_pos.size(), indices_neg.size());
      ROS_INFO("took1 %f s", _timer.elapsed());

      if(indices_pos.size()+indices_neg.size()<1000)
        return;

      std::vector<int> indices_pos2, indices_neg2;
      std::cout<<"INFO A:  ";
      for(int i=0; i<indices_pos.size(); i++) {
        int mi;
        int info=getI(indices_pos[i], depth_map, pc);
        //if(info<10) std::cout<<info<<" ";
        if( info<17 && info>1) {
          if(getMaxDiff(pc_old, indices_pos[i])>diff2)
            if(makes_sense(pc_old[indices_pos[i]].z)) indices_pos2.push_back(indices_pos[i]);
          if(getMaxDiff2(pc, indices_pos[i], pc_old, mi)>diff2)
            if(makes_sense(pc[mi].z)) indices_neg2.push_back(mi);
        }
      }
      std::cout<<"\nINFO A:  ";
      for(int i=0; i<indices_neg.size(); i++) {
        int mi;
        int info=getI(indices_neg[i], depth_map, pc);
        //if(info<10) std::cout<<info<<" ";
        if( info<17 && info>1 ) {
          if(getMaxDiff(pc, indices_neg[i])>diff2)
            if(makes_sense(pc[indices_neg[i]].z)) indices_neg2.push_back(indices_neg[i]);
          if(getMaxDiff2(pc_old, indices_neg[i], pc, mi)>diff2)
            if(makes_sense(pc_old[mi].z)) indices_pos2.push_back(mi);
        }
      }
      std::cout<<"\n";
      ROS_INFO("found %d %d", indices_pos2.size(), indices_neg2.size());
      ROS_INFO("took1.5 %f s", _timer.elapsed());

      //TODO: TESTING
      std::vector<int> _cor_inds;
      float _cor_inds_qual;
      {
        pcl::PointCloud<Point> tmp_pc_old, tmp_pc_new;
        for(int i=0; i<indices_pos2.size(); i++) {
          tmp_pc_old.points.push_back(pc_old.points[indices_pos2[i]]);
          Point pp=tmp_pc_old[i];
#ifdef RGB_
          pp.g=0;pp.r=255;
          pp.b=0;
#endif
          _p.push_back(pp);
        }
        for(int i=0; i<indices_neg2.size(); i++) {
          tmp_pc_new.points.push_back(pc.points[indices_neg2[i]]);
          Point pp=tmp_pc_new[i];
#ifdef RGB_
          pp.g=pp.r=0;
          pp.b=255;
#endif
          _p.push_back(pp);
        }
        {
          static int it=0;
          ++it;
          char buffer[128];
          sprintf(buffer,"Op%d.pcd",it);
          pcl::io::savePCDFile(buffer,tmp_pc_old);
          sprintf(buffer,"Np%d.pcd",it);
          pcl::io::savePCDFile(buffer,tmp_pc_new);
        }
        {
          sensor_msgs::PointCloud2 pc_out;
          //pcl::transformPointCloud(_p,_p,Stransf);
          pcl::toROSMsg(_p,pc_out);
          pc_out.header = header_;
          point_cloud_pub_.publish(pc_out);
        }
#if 0
        tmp_pc_old.clear();
        Point p;
        for(int i=0; i<2; i++)
          for(int j=0; j<2; j++)
            for(int k=0; k<2; k++) {
              p.x=i;
              p.y=j;
              p.z=k;
              tmp_pc_old.push_back(p);
            }
        if(0){
          Eigen::Vector3f va,vb;
          va(2)=1;va(1)=0;va(0)=0;
          vb(2)=1;vb(1)=0.1;vb(0)=0;
          Eigen::Quaternionf qq;
          qq.setFromTwoVectors(va,vb);
          Eigen::Matrix4f ttt=Eigen::Matrix4f::Identity();
          for(int x=0; x<3; x++)
            for(int y=0; y<3; y++)
              ttt(x,y) = qq.toRotationMatrix()(x,y);
          pcl::transformPointCloud(tmp_pc_old,tmp_pc_new,ttt);
          std::cout<<"CORRECT TRANS\n"<<ttt<<"\n";
        }


        std::vector<int> ind_t_old, ind_t_new;
        for(int i=0; i<tmp_pc_old.size(); i++)ind_t_old.push_back(i);
        for(int i=0; i<tmp_pc_new.size(); i++)ind_t_new.push_back(i);
        Eigen::Matrix4f ttt=findCorr3_tf(tmp_pc_old,tmp_pc_new, ind_t_old,ind_t_new, ind_t_old.size(),ind_t_new.size());

        Stransf=ttt*Stransf;

        if(1) {
          pcl::PointCloud<pcl::PointXYZRGB> pc2;
          pcl::fromROSMsg(*pc_in,pc2);
          static pcl::PointCloud<pcl::PointXYZRGB> map_;

          pcl::transformPointCloud(pc2,pc2,Stransf);
          map_.header = pc2.header;
          map_+=pc2;

          pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
          voxel.setInputCloud(map_.makeShared());
          voxel.setLeafSize(0.03,0.03,0.03);
          voxel.filter(map_);

          sensor_msgs::PointCloud2 pc_out;
          pcl::toROSMsg(map_,pc_out);
          map_pub_.publish(pc_out);
        }
#endif

#if 0
        std::vector<SORT_S> tv;
        for(int i=0; i<tmp_pc_new.size(); i++) {
          SORT_S s;
          Eigen::Vector3f v=tmp_pc_new.points[i].getArray3fMap();
          s.dis=v.norm();
          s.ind=i;
          tv.push_back(s);
        }

        int numsR[8]={},numsT[9]={};

        float rmax=0.1;
        float tmax=0.2;
        float m=0;
        for(int i=0; i<tmp_pc_old.size(); i++) {
          Eigen::Vector3f v=tmp_pc_old.points[i].getArray3fMap();
          float t = v.norm();
          for(int j=0; j<tv.size(); j++) {
            if( std::abs(tv[j].dis-t) < tmax) {
              float dT, dA;
              float d_min;
              getDis(tmp_pc_old.points[i].getArray3fMap(), tmp_pc_new.points[tv[j].ind].getArray3fMap(), dT, dA);

              if(dA>=rmax) continue;

              dA=std::max(0.f,dA);
              numsT[ (int)((tv[j].dis-t+tmax)/(2*tmax)*9) ]++;
              numsR[ (int)(dA/rmax*8) ]++;
              m++;
            }
          }
        }

        for(int i=0; i<9; i++)
          std::cout<<(i-4.5)*tmax/(4.5)<<" \t";
        std::cout<<"\nts:\n";
        for(int i=0; i<9; i++)
          std::cout<<(int)(numsT[i]/m*100)<<" \t";
        std::cout<<"\n";
        for(int i=0; i<8; i++)
          std::cout<<(i)*rmax/(8)<<" \t";
        std::cout<<"\nrs:\n";
        for(int i=0; i<8; i++)
          std::cout<<(int)(numsR[i]/m*100)<<" \t";
        std::cout<<"\n";

        float ges=1;
        int best=7;
        for(;best>0; best--) {
          if(ges-numsR[best]/m<0.2)
            break;
          ges-=numsR[best]/m;
        }
        std::cout<<"best rot: "<<best<<"\n";

        best=numsT[0]/m;
        best=0;
        for(int i=1; i<9; i++)
        {
          if(best<numsT[i]/m) {
            best=numsT[i]/m;
            best=i;
          }
        }
        std::cout<<"best trans: "<<best<<"\n";

        float min_t=(best-4.5)*tmax/(4.5);
        float max_t=(best+1-4.5)*tmax/(4.5);

        m=0;
        for(int i=0; i<8; i++)numsR[i]=0;

        std::vector<int> inds_A, inds_B;

        for(int i=0; i<tmp_pc_old.size(); i++) {

          Eigen::Vector3f v=tmp_pc_old.points[i].getArray3fMap();
          float t = v.norm();
          for(int j=0; j<tv.size(); j++) {
            if( (int)((tv[j].dis-t+tmax)/(2*tmax)*9) == best) {
              float dT, dA;
              float d_min;
              getDis(tmp_pc_old.points[i].getArray3fMap(), tmp_pc_new.points[tv[j].ind].getArray3fMap(), dT, dA);

              if(dA>=rmax) continue;

              dA=std::max(0.f,dA);
              numsR[ (int)(dA/rmax*8) ]++;
              m++;

              inds_A.push_back(i);
              inds_B.push_back(tv[j].ind);

            }
          }
        }

        ROS_INFO("num %d",m);

        findCorr3_tf(tmp_pc_old,tmp_pc_new,inds_A, inds_B, inds_A.size(), inds_B.size());

        for(int i=0; i<8; i++)
          std::cout<<(i)*rmax/(8)<<" \t";
        std::cout<<"\nrs:\n";
        for(int i=0; i<8; i++)
          std::cout<<(int)(numsR[i]/m*100)<<" \t";
        std::cout<<"\n";

        ges=1;
        best=7;
        for(;best>0; best--) {
          if(ges-numsR[best]/m<0.2)
            break;
          ges-=numsR[best]/m;
        }
        std::cout<<"best rot: "<<best<<"\n";
#endif

#if 0

        //do ICP
        pcl::IterativeClosestPoint<Point,Point> icp;

        icp.setInputCloud( tmp_pc_new.makeShared());
        //icp.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        icp.setInputTarget(tmp_pc_old.makeShared());
        icp.setMaximumIterations(50);
        icp.setRANSACOutlierRejectionThreshold(0.02);
        icp.setMaxCorrespondenceDistance(0.1);
        icp.setTransformationEpsilon (0.00001);

        pcl::PointCloud<Point> result;
        icp.align(result);

        Eigen::Matrix4f transf = icp.getFinalTransformation();

        Stransf=transf*Stransf;

        if(rand()%3==0) {
          pcl::PointCloud<pcl::PointXYZRGB> pc2;
          pcl::fromROSMsg(*pc_in,pc2);
          static pcl::PointCloud<pcl::PointXYZRGB> map_;

          pcl::transformPointCloud(pc2,pc2,Stransf);
          map_.header = pc2.header;
          map_+=pc2;

          pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
          voxel.setInputCloud(map_.makeShared());
          voxel.setLeafSize(0.03,0.03,0.03);
          voxel.filter(map_);

          sensor_msgs::PointCloud2 pc_out;
          pcl::toROSMsg(map_,pc_out);
          map_pub_.publish(pc_out);
        }
      }

#else

      pcl::PointCloud<Point> source, target;

#if USE_SMART_GRID_
#elif 1
      //for(int i=0; i<10; i++)
      if(tmp_pc_new.size()<tmp_pc_old.size())
        ROS_INFO("new method %f", _cor_inds_qual=findCorr(tmp_pc_old, tmp_pc_new, _cor_inds));
      else
        ROS_INFO("new method %f", _cor_inds_qual=findCorr(tmp_pc_new, tmp_pc_old, _cor_inds));
      ROS_INFO("took1.6 %f s", _timer.elapsed());

      if(_cor_inds_qual<0.45f)
        return;

      if(tmp_pc_new.size()<tmp_pc_old.size()) {
        for(int i=0; i<_cor_inds.size(); i++)
          if( _cor_inds[i]!=-1 ) {
            source.points.push_back(tmp_pc_old.points[i]);
            target.points.push_back(tmp_pc_new.points[_cor_inds[i]]);
          }
      }
      else {
        for(int i=0; i<_cor_inds.size(); i++)
          if( _cor_inds[i]!=-1 ) {
            source.points.push_back(tmp_pc_old.points[_cor_inds[i]]);
            target.points.push_back(tmp_pc_new.points[i]);
          }
      }
#else
      {
        // From the set of correspondences found, attempt to remove outliers
        // Create the registration model
        typedef typename pcl::SampleConsensusModelRegistration<Point>::Ptr SampleConsensusModelRegistrationPtr;
        SampleConsensusModelRegistrationPtr model;
        model.reset (new pcl::SampleConsensusModelRegistration<Point> (tmp_pc_new.makeShared ()));
        // Pass the target_indices
        model->setInputTarget (tmp_pc_old.makeShared());
        // Create a RANSAC model
        pcl::RandomSampleConsensus<Point> sac (model, 0.02);
        sac.setMaxIterations (1000);

        // Compute the set of inliers
        if (!sac.computeModel ())
        {
          return;
        }
        else
        {
          std::vector<int> inliers;
          // Get the inliers
          sac.getInliers (inliers);

          for (size_t i = 0; i < inliers.size (); ++i) {
            source.points.push_back(tmp_pc_old.points[i]);
            target.points.push_back(tmp_pc_new.points[inliers[i]]);
          }

        }
      }
#endif

      for(int i=0; i<source.size(); i++) {
        publishLineMarker(source.points[i].getVector3fMap(), target.points[i].getVector3fMap(), -i);
      }
      if(g_step) return;

      ROS_INFO("final size %d", source.size());

      //if(!source.size()) return;

      ROS_INFO("error before %f", calc_error(source, target));
      Eigen::Matrix4f transf=Eigen::Matrix4f::Identity();
#if USE_SMART_GRID_
      //transf=findTF(tmp_pc_new,tmp_pc_old);

      if(!pcl_isfinite(transf(0,0)))
        return;

#elif USE_LM
      pcl::registration::TransformationEstimationLM<Point, Point> estimate;

      estimate.estimateRigidTransformation(target,source, transf);
#elif USE_TF
      if(!source.size()) return;

      Eigen::Vector3f mid_s1=source.points[0].getVector3fMap();
      Eigen::Vector3f mid_t1=target.points[0].getVector3fMap();
      for(int i=1; i<source.size(); i++) {
        mid_s1+=source.points[i].getVector3fMap();
        mid_t1+=target.points[i].getVector3fMap();
      }
      mid_s1/=source.size();
      mid_t1/=target.size();

      Eigen::Vector3f mid_s=source.points[0].getVector3fMap();
      Eigen::Vector3f mid_t=target.points[0].getVector3fMap();

      Eigen::Vector3f mid_s2=source.points[source.size()-1].getVector3fMap();
      Eigen::Vector3f mid_t2=target.points[source.size()-1].getVector3fMap();

      int n1=0,n2=0;
      for(int i=0; i<source.size(); i++) {
        if(mid_s1.squaredNorm()<source.points[i].getVector3fMap().squaredNorm()) {
          mid_s+=source.points[i].getVector3fMap();
          mid_t+=target.points[i].getVector3fMap();
          n1++;
        }
        else {
          mid_s2+=source.points[i].getVector3fMap();
          mid_t2+=target.points[i].getVector3fMap();
          n2++;
        }
      }
      mid_s/=n1;
      mid_t/=n1;
      mid_s2/=n2;
      mid_t2/=n2;

      /*std::cout<<mid_s<<"\n";
      std::cout<<mid_t<<"\n";
      std::cout<<mid_s2<<"\n";
      std::cout<<mid_t2<<"\n";

      float rotationaxes[3]={};
      for(int i=0; i<source.size()/2-1; i++) {
        //mid_s = source.points[source.size()-1-i].getVector3fMap();
        //mid_t = target.points[target.size()-1-i].getVector3fMap();

        //std::cout<<source.points[i].getVector3fMap()-mid_s<<"\n";
        //std::cout<<target.points[i].getVector3fMap()-mid_t<<"\n";

        Eigen::Quaternionf q;
        q.setFromTwoVectors(target.points[i].getVector3fMap()-mid_t, source.points[i].getVector3fMap()-mid_s);
        Eigen::Matrix3f t =q.toRotationMatrix();
        float rx = atan2(t(2,1), t(2,2));
        float ry = asin(-t(2,0));
        float rz = atan2(t(1,0), t(0,0));

        rotationaxes[0]+=rx;
        rotationaxes[1]+=ry;
        rotationaxes[2]+=rz;

        ROS_INFO("rot %f %f %f (%f %f)", rotationaxes[0],rotationaxes[1],rotationaxes[2], (target.points[i].getVector3fMap()-mid_t).norm(), (source.points[i].getVector3fMap()-mid_s).norm());
        Eigen::Vector3f v;
        v(0)=v(1)=0;
        v(2)=1;
        v=t*v;
        ROS_INFO("v %f %f %f",v(0),v(1),v(2));
      }
      rotationaxes[0]/=source.size();
      rotationaxes[1]/=source.size();
      rotationaxes[2]/=source.size();*/

      Eigen::Matrix3f R;
      /*R = Eigen::AngleAxisf(rotationaxes[2], Eigen::Vector3f::UnitZ())
       * Eigen::AngleAxisf(rotationaxes[1], Eigen::Vector3f::UnitY())
       * Eigen::AngleAxisf(rotationaxes[0], Eigen::Vector3f::UnitX());*/

      {
        Eigen::Quaternionf q;
        q.setFromTwoVectors(mid_t-mid_t2,mid_s-mid_s2);
        Eigen::Vector3f v;
        v(0)=v(1)=0;
        v(2)=1;
        v=q.toRotationMatrix()*v;
        //ROS_INFO("v %f %f %f",v(0),v(1),v(2));

        ROS_INFO("angular distance %f", q.angularDistance(Eigen::Quaternionf::Identity()));

        R=q.toRotationMatrix();
      }

      Eigen::Vector3f t;
      t(0)=t(1)=t(2)=0.f;

      for(int i=0; i<source.size(); i++) {
        t+=source.points[i].getVector3fMap()-R*target.points[i].getVector3fMap();
      }
      t/=source.size();

      for(int x=0; x<3; x++)
        for(int y=0; y<3; y++)
          transf(x,y) = R(x,y);
      transf.col(3).head<3>() = t;

      transf=getTF(mid_t,mid_t2,mid_s,mid_s2);

      pcl::transformPointCloud(source,source,transf);
#else
      int num_it=0;
      for(int i=0; i<150; i++) {
        pcl::PointCloud<Point> s, t;
        for(int j=0; j<10; j++) {
          int r=rand()%source.size();
          s.points.push_back( source.points[r] );
          t.points.push_back( target.points[r] );
        }
        float e1,e2;
        e1=calc_error(source, target);

        Eigen::Matrix4f tr;
        pcl::estimateRigidTransformationSVD(t,s, tr);

        t=target;
        pcl::transformPointCloud(t,t,tr);
        e2=calc_error(source, t);
        if(e2<e1) {
          transf=transf*tr;
          target=t;
        }

      }
#endif
      ROS_INFO("error after %f", calc_error(source, target));

      std::cout<<transf<<"\n";
      Stransf=transf*Stransf;

      if(rand()%4==0) {
        pcl::PointCloud<pcl::PointXYZRGB> pc2;
        pcl::fromROSMsg(*pc_in,pc2);
        static pcl::PointCloud<pcl::PointXYZRGB> map_;

        pcl::transformPointCloud(pc2,pc2,Stransf);
        map_.header = pc2.header;
        map_+=pc2;

        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        voxel.setInputCloud(map_.makeShared());
        voxel.setLeafSize(0.03,0.03,0.03);
        voxel.filter(map_);

        sensor_msgs::PointCloud2 pc_out;
        pcl::toROSMsg(map_,pc_out);
        map_pub_.publish(pc_out);
      }
    }
#endif
#if 0
    /*for(int i=0; i<indices_neg2.size(); i++) {
        Point pp=pc.points[indices_neg2[i]];
        pp.g=0;
        pp.r=255;
        pp.b=0;
        _p.push_back(pp);
      }
      for(int i=0; i<indices_pos2.size(); i++) {
        Point pp=pc_old.points[indices_pos2[i]];
        pp.g=255;
        pp.r=0;
        pp.b=0;
          _p.push_back(pp);
      }
      {
        sensor_msgs::PointCloud2 pc_out;
        //pcl::transformPointCloud(_p,_p,Stransf);
        pcl::toROSMsg(_p,pc_out);
        pc_out.header = header_;
        point_cloud_pub_.publish(pc_out);
      }
      return;*/

    std::vector<int> try_pos, try_neg;
    std::vector<int> try_posC, try_negC;
    std::vector<int> tmp_try_posC, tmp_try_negC;


    //try_pos=indices_pos2;
    //try_neg=indices_neg2;
    /*int tries=0;
        while(try_pos.size()<222&&tries<10000) {
          ++tries;
          int r=indices_pos[rand()%indices_pos.size()];
          r=getBest(pc_old,r,diff1);
          if(r!=-1)
            try_pos.push_back(r);
        }

        tries=0;
        while(try_neg.size()<222&&tries<10000) {
          ++tries;
          int r=indices_neg[rand()%indices_neg.size()];
          r=getBest(pc,r,diff1);
          if(r!=-1)
            try_neg.push_back(r);
        }*/

    ROS_INFO("edges %d %d", try_pos.size(), try_neg.size());

    /*for(int i=0; i<try_pos.size(); i++)
          try_posC.push_back( getNearest(pc,pc_old, try_pos[i], diff2) );
        for(int i=0; i<try_neg.size(); i++)
          try_negC.push_back( getNearest(pc_old,pc, try_neg[i], diff2) );*/

    if(indices_pos2.size()<indices_neg2.size()) {
      try_pos=indices_pos2;

      pcl::PointCloud<Point> pc_new;
      for(int i=0; i<indices_neg2.size(); i++) {
        pc_new.points.push_back(pc.points[indices_neg2[i]]);
      }

      pc_new.height=1;
      pc_new.width=pc_new.size();
      pcl::KdTree<Point>::Ptr tree (new pcl::KdTreeFLANN<Point>);
      tree->setInputCloud (pc_new.makeShared());

      for(int i=0; i<indices_pos2.size(); i++) {
        std::vector< int > k_indices;
        std::vector< float > k_distances;
        tree->nearestKSearch(pc_old.points[indices_pos2[i]],1,k_indices,k_distances);
        if(k_distances[0]<0.2)
          try_posC.push_back(indices_neg2[k_indices[0]]);
        else
          try_posC.push_back(-1);
      }
    }
    else {
      try_neg=indices_neg2;

      pcl::PointCloud<Point> pc_new;
      for(int i=0; i<indices_pos2.size(); i++) {
        pc_new.points.push_back(pc_old.points[indices_pos2[i]]);
      }

      pc_new.height=1;
      pc_new.width=pc_new.size();
      pcl::KdTree<Point>::Ptr tree (new pcl::KdTreeFLANN<Point>);
      tree->setInputCloud (pc_new.makeShared());

      for(int i=0; i<indices_neg2.size(); i++) {
        std::vector< int > k_indices;
        std::vector< float > k_distances;
        tree->nearestKSearch(pc.points[indices_neg2[i]],1,k_indices,k_distances);
        if(k_distances[0]<0.2)
          try_negC.push_back(indices_pos2[k_indices[0]]);
        else
          try_negC.push_back(-1);
      }
    }


    Eigen::Vector3f m_new, m_old;
    m_old(0)=m_old(1)=m_old(2)=0;
    m_new=m_old;

    //debug output
    int found_p=0;
    for(int i=0; i<try_posC.size(); i++)
      if( try_posC[i]!=-1 ) {
        m_old+=pc_old.points[try_pos[i]].getVector3fMap();
        m_new+=pc.points[try_posC[i]].getVector3fMap();
        found_p++;
        pc.points[try_posC[i]].r=255;
        pc.points[try_posC[i]].g=0;
        pc.points[try_posC[i]].b=0;
        pc_old.points[try_pos[i]].r=0;
        pc_old.points[try_pos[i]].g=0;
        pc_old.points[try_pos[i]].b=255;
        _p.points.push_back(pc_old.points[try_pos[i]]);
        _p.points.push_back(pc.points[try_posC[i]]);
      }
    int found_n=0;
    for(int i=0; i<try_negC.size(); i++)
      if( try_negC[i]!=-1 ) {
        m_old+=pc_old.points[try_negC[i]].getVector3fMap();
        m_new+=pc.points[try_neg[i]].getVector3fMap();
        found_n++;
        pc.points[try_neg[i]].r=255;
        pc.points[try_neg[i]].g=0;
        pc.points[try_neg[i]].b=0;
        pc_old.points[try_negC[i]].r=0;
        pc_old.points[try_negC[i]].g=0;
        pc_old.points[try_negC[i]].b=255;
        _p.points.push_back(pc.points[try_neg[i]]);
        _p.points.push_back(pc_old.points[try_negC[i]]);
      }
    ROS_INFO("edges match %d %d", found_p, found_n);
    ROS_INFO("took2 %f s", _timer.elapsed());

    m_new/=found_n+found_p;
    m_old/=found_p+found_n;

    std::cout<<m_new<<"\n";
    std::cout<<m_old<<"\n";

    {
      sensor_msgs::PointCloud2 pc_out;
      //pcl::transformPointCloud(_p,_p,Stransf);
      pcl::toROSMsg(_p,pc_out);
      pc_out.header = header_;
      point_cloud_pub_.publish(pc_out);
    }
    if(g_step&&0) {
      sensor_msgs::PointCloud2 pc_out;
      pcl::toROSMsg(pc_old,pc_out);
      map_pub_.publish(pc_out);
      return;
    }

    tmp_try_posC=try_posC;
    tmp_try_negC=try_negC;
    for(int iteration=0; iteration<100; iteration++) {
      if(iteration==99)
        return;

      try_posC=tmp_try_posC;
      try_negC=tmp_try_negC;

      Eigen::Vector3f pos_new, pos_old;

      if(try_posC.size()>0) {
        int i;
        while(try_posC.size()) {
          i=rand()%try_posC.size();
          if(try_posC[i]==-1) continue;
          m_old=pc_old.points[try_pos[i]].getVector3fMap();
          m_new=pc.points[try_posC[i]].getVector3fMap();
          break;
        }
        while(try_posC.size()) {
          i=rand()%try_posC.size();
          if(try_posC[i]==-1) continue;
          pos_old=pc_old.points[try_pos[i]].getVector3fMap();
          pos_new=pc.points[try_posC[i]].getVector3fMap();
          break;
        }
      }
      else if(try_negC.size()>0){
        int i;
        while(try_negC.size()) {
          i=rand()%try_negC.size();
          if(try_negC[i]==-1) continue;
          m_old=pc_old.points[try_negC[i]].getVector3fMap();
          m_new=pc.points[try_neg[i]].getVector3fMap();
          break;
        }
        while(try_negC.size()) {
          i=rand()%try_negC.size();
          if(try_negC[i]==-1) continue;
          pos_old=pc_old.points[try_negC[i]].getVector3fMap();
          pos_new=pc.points[try_neg[i]].getVector3fMap();
          break;
        }
      }

      std::vector<float> dis_trans, dis_rot;

      for(int i=0; i<try_posC.size(); i++)
        if( try_posC[i]!=-1 ) {
          Eigen::Vector3f C =pc_old.points[try_pos[i]].getVector3fMap();
          Eigen::Vector3f S =pc.points[try_posC[i]].getVector3fMap();

          /*std::cout<<C(0)<<" "<<C(1)<<" "<<C(2)<<"\n";
            std::cout<<S(0)<<" "<<S(1)<<" "<<S(2)<<"\n";
            std::cout<<std::abs((S-m_new).squaredNorm()-(C-m_old).squaredNorm())<<"\n";*/
          if( std::abs((S-m_new).squaredNorm()-(C-m_old).squaredNorm())>0.015 ) {
            try_posC[i]=-1;
            continue;
          }

          float delta_trans = std::abs(C.norm()-S.norm()),
              delta_rot = acosf(C.dot(S)/(C.norm()*S.norm()));

          dis_trans.push_back(delta_trans);
          dis_rot.push_back(delta_rot);
        }

      for(int i=0; i<try_negC.size(); i++)
        if( try_negC[i]!=-1 ) {
          Eigen::Vector3f C =pc_old.points[try_negC[i]].getVector3fMap();
          Eigen::Vector3f S =pc.points[try_neg[i]].getVector3fMap();

          /*std::cout<<C(0)<<" "<<C(1)<<" "<<C(2)<<"\n";
            std::cout<<S(0)<<" "<<S(1)<<" "<<S(2)<<"\n";
            std::cout<<std::abs((S-m_new).squaredNorm()-(C-m_old).squaredNorm())<<"\n";*/
          if( std::abs((S-m_new).squaredNorm()-(C-m_old).squaredNorm())>0.015 ) {
            try_negC[i]=-1;
            continue;
          }

          float delta_trans = std::abs(C.norm()-S.norm()),
              delta_rot = acosf(C.dot(S)/(C.norm()*S.norm()));

          dis_trans.push_back(delta_trans);
          dis_rot.push_back(delta_rot);
        }
      ROS_INFO("final size %d %f", dis_trans.size(), dis_trans.size()/(float)(try_posC.size()+try_negC.size()));

      if(dis_trans.size()<0.3*(try_posC.size()+try_negC.size()))
        continue;

      std::sort(dis_trans.begin(),dis_trans.end());
      std::sort(dis_rot.begin(),dis_rot.end());

      /*for(int i=0; i<dis_trans.size(); i++) {
          ROS_INFO("%f %f",dis_trans[i],dis_rot[i]);
        }*/

      float bo_t=dis_trans[dis_trans.size()*5/6], bo_r=dis_rot[dis_rot.size()*5/6];
      /*bo_t=bo_r=0;
        for(int i=0; i<dis_trans.size();i++) {
          bo_t+=dis_trans[i];
          bo_r+=dis_rot[i];
        }
        bo_t/=dis_trans.size();
        bo_r/=dis_trans.size();*/

      if(dis_trans.size()>0)
        ROS_INFO("mean %f %f", bo_t, bo_r);

      if(bo_t>0.2)
        return;

      pcl::PointCloud<Point> source, target;

      for(int i=0; i<try_posC.size(); i++)
        if( try_posC[i]!=-1 ) {
          Eigen::Vector3f C =pc_old.points[try_pos[i]].getVector3fMap();
          Eigen::Vector3f S =pc.points[try_posC[i]].getVector3fMap();

          float delta_trans = std::abs(C.norm()-S.norm()),
              delta_rot = acosf(C.dot(S)/(C.norm()*S.norm()));

          {//if( std::abs(delta_trans-bo_t)<0.03 && std::abs(delta_rot-bo_r)<0.03 ) {
            source.points.push_back(pc_old.points[try_pos[i]]);
            target.points.push_back(pc.points[try_posC[i]]);
          }
        }

      for(int i=0; i<try_negC.size(); i++)
        if( try_negC[i]!=-1 ) {
          Eigen::Vector3f C =pc_old.points[try_negC[i]].getVector3fMap();
          Eigen::Vector3f S =pc.points[try_neg[i]].getVector3fMap();
          float delta_trans = std::abs(C.norm()-S.norm()),
              delta_rot = acosf(C.dot(S)/(C.norm()*S.norm()));

          {//if( std::abs(delta_trans-bo_t)<0.03 && std::abs(delta_rot-bo_r)<0.03 ) {
            source.points.push_back(pc_old.points[try_negC[i]]);
            target.points.push_back(pc.points[try_neg[i]]);
          }
        }

      for(int i=0; i<source.size(); i++) {
        publishLineMarker(source.points[i].getVector3fMap(), target.points[i].getVector3fMap(), -i);
      }

      ROS_INFO("final size %d", source.size());

      if(!source.size()) return;

      ROS_INFO("error before %f", calc_error(source, target));
      Eigen::Matrix4f transf=Eigen::Matrix4f::Identity();
      for(int i=0; i<100; i++) {
        pcl::PointCloud<Point> s, t;
        for(int j=0; j<10; j++) {
          int r=rand()%source.size();
          s.points.push_back( source.points[r] );
          t.points.push_back( target.points[r] );
        }
        float e1,e2;
        e1=calc_error(source, target);

        Eigen::Matrix4f tr;
        pcl::estimateRigidTransformationSVD(t,s, tr);

        t=target;
        pcl::transformPointCloud(t,t,tr);
        e2=calc_error(source, t);
        if(e2<e1) {
          transf=transf*tr;
          target=t;
        }

      }
      ROS_INFO("error after %f", calc_error(source, target));

      std::cout<<transf<<"\n";
      Stransf=transf*Stransf;

      if(rand()%3==0) {
        pcl::PointCloud<pcl::PointXYZRGB> pc2;
        pcl::fromROSMsg(*pc_in,pc2);
        static pcl::PointCloud<pcl::PointXYZRGB> map_;

        pcl::transformPointCloud(pc2,pc2,Stransf);
        map_.header = pc2.header;
        map_+=pc2;

        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        voxel.setInputCloud(map_.makeShared());
        voxel.setLeafSize(0.03,0.03,0.03);
        voxel.filter(map_);

        sensor_msgs::PointCloud2 pc_out;
        pcl::toROSMsg(map_,pc_out);
        map_pub_.publish(pc_out);
      }

      break;
    }

#endif
  }

  pc_old = pc;

  ROS_INFO("took full %f s", _timer.elapsed());
}

int getI(const int ind, unsigned char *depth_map, const pcl::PointCloud<Point> &pc) {
  int x=ind%pc.width;
  int y=ind/pc.width;
  if(x<2||y<2||x>=pc.width-2||y>=pc.height-2)
    return 25;

  int r=0;
  for(int dx=-2; dx<=2; dx++) {
    for(int dy=-2; dy<=2; dy++) {
      if(dx==0 && dy==0)
        continue;
      r+=depth_map[getInd(x+dx,y+dy)];
    }
  }

  return r;
}

float calc_error(const pcl::PointCloud<Point> &s,const pcl::PointCloud<Point> &t) {
  float error=0.f;
  for(int i=0; i<s.points.size(); i++)
    error+=(s.points[i].getVector3fMap()-t.points[i].getVector3fMap()).norm();
  return error;
}

#define _SEARCH_ 2
float getMaxDiff(const pcl::PointCloud<Point> &pc,const int ind) {
  int x=ind%pc.width;
  int y=ind/pc.width;
  if(x<_SEARCH_||y<_SEARCH_||x>=pc.width-_SEARCH_||y>=pc.height-_SEARCH_)
    return 0.f;

  float z=pc.points[ind].z;

  float m=0.f;
  for(int dx=-_SEARCH_; dx<=_SEARCH_; dx++) {
    for(int dy=-_SEARCH_; dy<=_SEARCH_; dy++) {
      if(dx==0 && dy==0)
        continue;
      m=std::max(m,pc.points[getInd(x+dx,y+dy)].z-z);
    }
  }

  return m;
}

float getMaxDiff2(const pcl::PointCloud<Point> &pc,const int ind,const pcl::PointCloud<Point> &pc2, int &mi) {
  int x=ind%pc.width;
  int y=ind/pc.width;
  if(x<_SEARCH_||y<_SEARCH_||x>=pc.width-_SEARCH_||y>=pc.height-_SEARCH_)
    return 0.f;

  float z=pc2.points[ind].z;

  float m=0.f, _mi=z*z*0.01;
  for(int dx=-_SEARCH_; dx<=_SEARCH_; dx++) {
    for(int dy=-_SEARCH_; dy<=_SEARCH_; dy++) {
      if(dx==0 && dy==0)
        continue;
      float f=pc.points[getInd(x+dx,y+dy)].z-z;
      m=std::max(m,f);
      f=std::abs(f);
      if(f<_mi) {
        _mi=f;
        mi = getInd(x+dx,y+dy);
      }
    }
  }

  return m;
}

int getNearest(const pcl::PointCloud<Point> &pc, const pcl::PointCloud<Point> &pc2, const int ind, const float diff) {
  int x=ind%pc.width;
  int y=ind/pc.width;
  const int rad=40;
  if(x<rad||y<rad||x>=pc.width-rad||y>=pc.height-rad)
    return -1;

  Eigen::Vector3f v = pc2.points[ind].getVector3fMap();

  int mi=-1;
  float dis=100.f;

  for(int dx=-rad; dx<=rad; dx++) {
    for(int dy=-rad; dy<=rad; dy++) {
      int in = getInd(x+dx,y+dy);
      if(in==ind) continue;

      Eigen::Vector3f v2 = pc.points[in].getVector3fMap();
      float delta_trans = std::abs(v2.norm()-v.norm());
      if(delta_trans<0.05 && (v2-v).squaredNorm()<dis && getMaxDiff(pc, in)>diff) {
        dis=(v2-v).squaredNorm();
        mi= in;
      }
    }
  }

  //ROS_INFO("%f %f", v(2), pc.points[mi].z);

  /*if(mi!=-1) {
    Eigen::Vector3f C =pc2.points[ind].getVector3fMap();
    Eigen::Vector3f S =pc.points[mi].getVector3fMap();

    float delta_trans = std::abs(C.norm()-S.norm());
    if(delta_trans>0.05)
      return -1;
    }*/


  return mi;
}

int getBest(const pcl::PointCloud<Point> &pc, const int ind, const float diff) {
  int x=ind%pc.width;
  int y=ind/pc.width;
  const int rad=20;
  if(x<rad||y<rad||x>=pc.width-rad||y>=pc.height-rad)
    return -1;

  for(int dx=-rad; dx<=rad; dx++) {
    for(int dy=-rad; dy<=rad; dy++) {
      int in = getInd(x+dx,y+dy);
      if(getMaxDiff(pc, in)>diff) {
        return in;
      }
    }
  }
  return -1;
}


//pc... already filtered!!!!! (PERFORMANCE)
float findCorr(pcl::PointCloud<Point> &pc_old, const pcl::PointCloud<Point> &pc_new, std::vector<int> &cor_inds) {
  //, std::vector<int> kp_old, std::vector<int> kp_new
  //2 for model, 1 for checking
  if(pc_old.size()<3 || pc_new.size()<3)
    return 0.f;

  PrecisionStopWatch psw;

  const int maxK_ = 20;
  const float matchingThreshold_ = 0.05*0.05; //5cm, verify

  boost::shared_ptr<pcl::KdTree<Point> > tree (new pcl::KdTreeFLANN<Point>);
  tree->setInputCloud (pc_new.makeShared());
  std::vector< int > k_indices;
  std::vector< float > k_distances;


  //resort
  std::vector<SORT_S> resort;
  for(int i=0; i<pc_old.size(); i++) {

    if(!tree->nearestKSearch(pc_old[i],1,k_indices,k_distances))
      continue;

    SORT_S s;
    s.ind = i;
    s.dis = k_distances[0];
    resort.push_back(s);
  }

  SORT_S _s_;
  std::sort(resort.begin(), resort.end(), _s_);
  for(int i=0; i<resort.size(); i++) {
    if(resort[i].ind>i)
      std::swap(pc_old.points[i],pc_old.points[resort[i].ind]);
  }


  int usingA=resort.size()/8; //TODO: check this
  Eigen::Vector4f best_pairs[4];
  float best_overall=0.f;

#if DEBUG_SWITCH_
  static double tt1=0;
  tt1+=psw.precisionStop();
  ROS_INFO("start %f", tt1);
#endif

  float per_sum=0.f, max_per=0.f;
  std::vector<int> inds;
  int inds_size;
  for(int k=0; k<15; k++) {
#if DEBUG_SWITCH_
    ROS_INFO("-------------------------");
#endif

    if(k!=0 && pc_old.size()*0.7<inds_size) {
      ;//just nothing :)
    }
    else {
      inds_size=pc_old.size();
      for(int i=0; i<inds_size; i++) inds.push_back(i);
    }

    Eigen::Vector4f pairs[4];

    per_sum=0.f;
    int tries=1;
    int matching_iteration_nr = 0;
    bool pairs_valid=false;
    do {
      ++matching_iteration_nr;
      if(matching_iteration_nr>20) {
#if DEBUG_SWITCH_
        ROS_INFO("break because it");
#endif
        break;
      }

      if(inds_size<3) {
#if DEBUG_SWITCH_
        ROS_INFO("break because inds");
#endif
        break;
      }

      int A,Ac, O,Oc;
      float desiredD;
      Eigen::Vector4f vO,vOc,vA,vAc;

      int sample_selection_iteration_nr = 0;
      while(sample_selection_iteration_nr<12) {
        ++sample_selection_iteration_nr;

#if 0
        //TODO: perhaps sorted with I, so that mor informative point is chosen preferably
        //TODO: prevent double selection
        int Aind = rand()%inds_size;
        A = inds[Aind];
        std::swap(inds[Aind], inds[inds_size-1]);
        vA = pc_old.points[A].getVector4fMap();

        //get nearest to A -> A*
        tree->nearestKSearch(pc_old.points[A],maxK_,k_indices,k_distances);
        Ac = k_indices[0];
        vAc = pc_new.points[Ac].getVector4fMap();
        float dT, dA;
        float d_min;
        getDis(vA, vAc, dT, dA);
        d_min = dT*dT+dA*dA;
        //TODO: this
#if PERHAPS_CHECK
        for(int i=1; i<k_indices.size(); i++) {
          getDis(vA, pc_new.points[k_indices[i]].getVector4fMap(), dT, dA);
          float f = dT*dT+dA*dA;

          if(d_min>f) {
            d_min=f;
            Ac = k_indices[i];
          }
        }
#endif

        //TODO: check plausability with dT<dTmax

        //TODO: perhaps sorted with I, so that mor informative point is chosen preferably
        //TODO: prevent double selection
        int inds_size2=inds_size-1;
        do {
          if(inds_size2<2) {
            ROS_INFO("error: not found");
            break;
          }
          int Oind = rand()%inds_size2;
          O=inds[Oind];
          if(O==A) continue;

          vO = pc_old.points[O].getVector4fMap();
          desiredD = (vA-vO).squaredNorm();

          --inds_size2;
          std::swap(inds[Oind], inds[inds_size2]);

          if(desiredD<0.1) continue;

          std::swap(inds[inds_size2], inds[inds_size-2]);
          break;
        } while(1);

        //get nearest to O -> O*
        tree->nearestKSearch(pc_old.points[O],maxK_,k_indices,k_distances);

        //TODO: applay better filtering with dT
        Oc=k_indices[0];
        float tmp_min = std::abs(desiredD-(pc_new.points[Oc].getVector4fMap()-vAc).squaredNorm());
        for(int i=1; i<k_indices.size(); i++) {
          float f= std::abs(desiredD-(pc_new.points[k_indices[i]].getVector4fMap()-vAc).squaredNorm());
          if(f<tmp_min) {
            Oc=k_indices[i];
            tmp_min=f;
          }
        }
        //ROS_INFO("tmp_min %f, %f", tmp_min, sqrtf(desiredD));
        if(tmp_min>matchingThreshold_) {
          //ROS_INFO("error: not good enough");
          continue;
        }
        vOc = pc_new.points[Oc].getVector4fMap();

#else

        //TODO: perhaps sorted with I, so that mor informative point is chosen preferably
        //TODO: prevent double selection
        int Aind = (usingA++)%inds_size;//rand()%inds_size;
        A = inds[Aind];
        std::swap(inds[Aind], inds[inds_size-1]);
        vA = pc_old.points[A].getVector4fMap();

        //get nearest to A -> A*
        tree->nearestKSearch(pc_old.points[A],maxK_,k_indices,k_distances);

#if DEBUG_SWITCH_
        static double tt21=0;
        tt21+=psw.precisionStop();
        ROS_INFO("A %f", tt21);
#endif

        //TODO: perhaps sorted with I, so that mor informative point is chosen preferably
        //TODO: prevent double selection
        int inds_size2=inds_size-1;
        do {
          if(inds_size2<2) {
            ROS_INFO("error: not found");
            break;
          }
          int Oind = rand()%inds_size2;
          O=inds[Oind];
          if(O==A) continue;

          vO = pc_old.points[O].getVector4fMap();
          desiredD = (vA-vO).squaredNorm();

          --inds_size2;
          std::swap(inds[Oind], inds[inds_size2]);

          if(desiredD<0.1) continue;

          std::swap(inds[inds_size2], inds[inds_size-2]);
          break;
        } while(1);

        std::vector< int > k_indicesO;
        std::vector< float > k_distancesO;

        //get nearest to O -> O*
        tree->nearestKSearch(pc_old.points[O],maxK_,k_indicesO,k_distancesO);

#if DEBUG_SWITCH_
        static double tt2=0;
        tt2+=psw.precisionStop();
        ROS_INFO("O+A %f", tt2);
#endif

        //O and A are known
        float tmp_min = sqrtf(matchingThreshold_);
        int m_a=-1, m_o=-1;
        for(int a=0; a<k_indices.size(); a++) {
          for(int o=0; o<k_indicesO.size(); o++) {
            Eigen::Vector4f d;
            float dT1, dT2, dA1, dA2;
            getDis(vA.head<3>(), pc_new.points[k_indices[a]].getVector3fMap(), dT1, dA1);
            getDis(vO.head<3>(), pc_new.points[k_indicesO[o]].getVector3fMap(), dT2, dA2);
            /*d(0) = desiredD-(pc_new.points[k_indicesO[o]].getVector3fMap()-pc_new.points[k_indices[a]].getVector3fMap()).squaredNorm();
              d(1) = 0;//dT1-dT2;
              d(2) = 0;//dA1-dA2;
              float f= d.squaredNorm();*/

            //TODO: check sqrtf
            float f = std::abs( sqrtf(desiredD)-(pc_new.points[k_indicesO[o]].getVector3fMap()-pc_new.points[k_indices[a]].getVector3fMap()).norm() );
            //float f = std::abs( (desiredD)-(pc_new.points[k_indicesO[o]].getVector3fMap()-pc_new.points[k_indices[a]].getVector3fMap()).squaredNorm() );
            //ROS_INFO("mm %f",f);
            if(f<tmp_min) {
              m_a=k_indices[a];
              m_o=k_indicesO[o];
              tmp_min=f;
            }
          }
        }

#if DEBUG_SWITCH_
        //ROS_INFO("min %f",(tmp_min));
#endif

        if(m_a==-1)
          continue;

        Oc=m_o;
        Ac=m_a;

        vOc = pc_new.points[Oc].getVector4fMap();
        vAc = pc_new.points[Ac].getVector4fMap();
#endif

        break;
      }

      inds_size-=2;

      if(sample_selection_iteration_nr==12) {
#if DEBUG_SWITCH_
        ROS_INFO("nothing found");
#endif
        continue;
      }

      //DEBUG OUTPUT
      /*if(sample_selection_iteration_nr>=20) {
          ROS_INFO("ERROR: not found");
          break;
        }

        std::cout<<"A:  "<<pc_old.points[A]<<"\n";
        std::cout<<"A*: "<<pc_new.points[Ac]<<"\n";
        std::cout<<"O:  "<<pc_old.points[O]<<"\n";
        std::cout<<"O*: "<<pc_new.points[Oc]<<"\n";

        float dT, dA;
        getDis(vA, vAc, dT, dA);
        std::cout<<"own disA: "<<dT<<" "<<dA<<" "<<(vA-vAc).squaredNorm()<<"\n";

        getDis(vO, vOc, dT, dA);
        std::cout<<"own disO: "<<dT<<" "<<" "<<dA<<(vO-vOc).squaredNorm()<<"\n";*/

      //TODO: check plausability with dT<dTmax

      // OK, now we have good samples

      const Eigen::Vector4f V=vO-vA, Vc=vOc-vAc;
      int supporters=0, max_supporters=inds_size;

      if(pairs_valid) {
        //old samples should be explained by the new ones
        Eigen::Matrix4f tf=getTF(vA.head<3>(),vO.head<3>(),vAc.head<3>(),vOc.head<3>());
        //Eigen::Vector3f vP = pairs[0]-vA;
        //Eigen::Quaternionf q;
        //q.setFromTwoVectors(V, vP);
        Eigen::Vector4f vPc = tf*pairs[0];//(q.toRotationMatrix()*Vc)*sqrtf(vP.squaredNorm()/desiredD)+vAc;  //this is assumed!!!

        if( (pairs[2]-vPc).squaredNorm()>matchingThreshold_*9) {
#if DEBUG_SWITCH_
          ROS_INFO("old couldn't be foundt 1");
#endif
          continue;
        }
#if DEBUG_SWITCH_
        else
          ROS_INFO("dis1 %f", (pairs[2]-vPc).norm());
#endif


        //vP = pairs[1]-vA;
        //q.setFromTwoVectors(V, vP);
        vPc = tf*pairs[1];//(q.toRotationMatrix()*Vc)*sqrtf(vP.squaredNorm()/desiredD)+vAc;  //this is assumed!!!

        if( (pairs[3]-vPc).squaredNorm()>matchingThreshold_*9) {
#if DEBUG_SWITCH_
          ROS_INFO("old couldn't be foundt 2");
#endif
          continue;
        }
#if DEBUG_SWITCH_
        else
          ROS_INFO("dis2 %f", (pairs[3]-vPc).norm());
#endif
      }

#if DEBUG_SWITCH_
      static double tt3=0;
      tt3+=psw.precisionStop();
      ROS_INFO("O*+A* %f", tt3);
#endif

      Eigen::Matrix4f tf=getTF(vA.head<3>(),vO.head<3>(),vAc.head<3>(),vOc.head<3>());
      /*Eigen::Quaternionf q;
      q.setFromTwoVectors(V, Vc);
      Eigen::Matrix3f R=q.toRotationMatrix();
      Eigen::Vector3f t=vAc-R*vA;*/
      for(int j=0; j<inds_size; j++) {
        if( (j>40 && supporters<4) || (j>100 && supporters<12)) //give up
          break;

        int i=inds[j];

        Eigen::Vector4f vP = pc_old[i].getVector4fMap();
        Eigen::Vector4f vPc = tf*vP;//t+R*vP;//(q.toRotationMatrix()*Vc)*sqrtf(vP.squaredNorm()/desiredD)+vAc;  //this is assumed!!!

        Point p;
        p.x=vPc(0);
        p.y=vPc(1);
        p.z=vPc(2);
        if(!tree->nearestKSearch(p,1,k_indices,k_distances))
          continue;

        if(k_distances[0]<matchingThreshold_) {
          supporters++;

          --inds_size;
          std::swap(inds[j], inds[inds_size]);
          j--;

        }
      }

#if DEBUG_SWITCH_
      static double tt4=0;
      tt4+=psw.precisionStop();
      ROS_INFO("cor %f", tt4);
#endif

      float per = supporters/(float)max_supporters;
      per_sum += supporters/(float)pc_old.size();

#if DEBUG_SWITCH_
      ROS_INFO("cor %d %d %d %d %f %f", supporters, max_supporters, inds_size, pc_old.size(), per, per_sum);
#endif

      //for testing purpose
      supporters=0;
      if(per>0.4) {
        pairs_valid=true;
        pairs[0] = vA;
        pairs[1] = vO;
        pairs[2] = vAc;
        pairs[3] = vOc;

#if 1
        Eigen::Matrix4f tf=getTF(vA.head<3>(),vO.head<3>(),vAc.head<3>(),vOc.head<3>());
        /*Eigen::Quaternionf q;
        q.setFromTwoVectors(V, Vc);
        Eigen::Matrix3f R=q.toRotationMatrix();
        Eigen::Vector3f t=vAc-R*vA;*/
        for(int i=0; i<pc_old.size(); i+=2) {
          Eigen::Vector4f vP = pc_old[i].getVector4fMap();
          Eigen::Vector4f vPc = tf*vP;//t+R*vP;//(q.toRotationMatrix()*Vc)*sqrtf(vP.squaredNorm()/desiredD)+vAc;  //this is assumed!!!

          Point p;
          p.x=vPc(0);
          p.y=vPc(1);
          p.z=vPc(2);

          if(!tree->nearestKSearch(p,1,k_indices,k_distances))
            continue;

          if(k_distances[0]<matchingThreshold_)
            supporters++;
        }
        float overall=supporters/(float)(pc_old.size()/2);
        ROS_INFO("overall %f", overall);
#else
        float overall = per_sum;
#endif

        if(overall+0.1<per_sum)
          ROS_INFO("ERROR: overall schould be around iterative percentage (hopefully)");

        per = overall-per_sum;
        per_sum = overall;

        if(overall>best_overall) {
          best_pairs[0] = pairs[0];
          best_pairs[1] = pairs[1];
          best_pairs[2] = pairs[2];
          best_pairs[3] = pairs[3];
          best_overall = overall;
        }
      }

#if DEBUG_SWITCH_
      static double tt5=0;
      tt5+=psw.precisionStop();
      ROS_INFO("o %f", tt5);
#endif

      if( (per<0.3f&&tries>1+matching_iteration_nr)||inds_size<10) {
#if DEBUG_SWITCH_
        ROS_INFO("break because last");
#endif
        break;
      }
      ++tries;
      if(per>0.3f)
        tries=0;

    } while(per_sum<0.9f);

    ROS_INFO("%f", per_sum);

    max_per = std::max(max_per, per_sum);

    if(best_overall>0.95)
      break;
  }

  {

    //get corrs
    std::vector< float > againstD;
    std::vector< int > k_indices, against;
    std::vector< float > k_distances;
    int supporters=0;
    int real_supporters=0;
    const Eigen::Vector4f V=best_pairs[1]-best_pairs[0], Vc=best_pairs[3]-best_pairs[2];
    const float desiredD = V.squaredNorm();
    for(int i=0; i<pc_new.size(); i++) {
      against.push_back(-1);
      againstD.push_back(-1);
    }
    Eigen::Matrix4f tf=getTF(best_pairs[0].head<3>(),best_pairs[1].head<3>(),
                             best_pairs[2].head<3>(),best_pairs[3].head<3>());
    for(int i=0; i<pc_old.size(); i++) {
      /*Eigen::Vector4f vP = pc_old[i].getVector4fMap()-best_pairs[0];
      Eigen::Quaternionf q;
      q.setFromTwoVectors(V, vP);
      Eigen::Vector4f vPc = (q.toRotationMatrix()*Vc)*sqrtf(vP.squaredNorm()/desiredD)+best_pairs[2];  //this is assumed!!!
       */
      Eigen::Vector4f vPc = tf*pc_old[i].getVector4fMap();

      Point p;
      p.x=vPc(0);
      p.y=vPc(1);
      p.z=vPc(2);

      if(!tree->nearestKSearch(p,1,k_indices,k_distances)) {
        cor_inds.push_back(-1);
        continue;
      }


      /*std::cout<<vP<<"\n";
          std::cout<<(q.toRotationMatrix()*Vc)*sqrtf(vP.squaredNorm()/desiredD)<<"\n\n";
          std::cout<<vP.squaredNorm()<<"\n";
          std::cout<<desiredD<<"\n";
          std::cout<<pairs[2]<<"\n";
          std::cout<<p<<"\n";

          float dT,dA;
          getDis(pc_old[i].getVector3fMap(), pc_new[k_indices[0]].getVector3fMap(), dT, dA);
          std::cout<<"own dis#: "<<dT<<" "<<dA<<" "<<(pc_old[i].getVector3fMap()-pc_new[k_indices[0]].getVector3fMap()).squaredNorm()<<"\n";
       */

      if(k_distances[0]<matchingThreshold_) {

        supporters++;

        /*if(against[k_indices[0]]!=-1) {
          if( k_distances[0]<againstD[k_indices[0]] ) { //we are better!!
            cor_inds[against[k_indices[0]]] = -1;
          }
          else {
            cor_inds.push_back(-1);
            continue;
          }
        }
        else*/
        real_supporters++;

        against[k_indices[0]] = i;
        againstD[k_indices[0]] = k_distances[0];

        //ROS_INFO("%d -> %d (%f)", cor_inds.size(), k_indices[0], k_distances[0]);
        cor_inds.push_back(k_indices[0]);
      }
      else
        cor_inds.push_back(-1);
    }
    per_sum=supporters/(float)pc_old.size();
    ROS_INFO("real %f", real_supporters/(float)pc_old.size());
    ROS_INFO("final matches %f", per_sum);

    return per_sum;
  }

  return max_per;
}


void publishMarkerPoint(const Point &p, int id, float r, float g, float b, float Size=0.02)
{
  if(!show_markers_)
    return;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.lifetime = ros::Duration(0.5);
  marker.header = header_;
  //marker.header.stamp = stamp;

  //create the marker in the table reference frame
  //the caller is responsible for setting the pose of the marker to match

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = Size;
  marker.scale.y = Size;
  marker.scale.z = Size;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  marker.id = id;
  marker.pose.position.x = p.x;
  marker.pose.position.y = p.y;
  marker.pose.position.z = p.z;

  marker_pub_.publish(marker);
}

static void publishTextMarker(const Eigen::Vector3f pos, const std::string &text, const int id=rand()%111111)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.lifetime = ros::Duration(id<0?0:0.4);
  marker.header = header_;
  //marker.header.stamp = stamp;



  //create the marker in the table reference frame
  //the caller is responsible for setting the pose of the marker to match

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.pose.position.x = pos(0);
  marker.pose.position.y = pos(1);
  marker.pose.position.z = pos(2);

  marker.scale.z = 10;

  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.3;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 1.0;

  marker.id = id;

  marker.text = text;

  marker_pub_.publish(marker);
}

static void publishLineMarker(const std::vector<Eigen::Vector4f> &pl, const int id=rand()%111111)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.lifetime = ros::Duration(id<0?0:4);
  marker.header = header_;
  //marker.header.stamp = stamp;



  //create the marker in the table reference frame
  //the caller is responsible for setting the pose of the marker to match

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1.0;

  marker.id = id;

  marker.points.resize(pl.size()+1);

  for(int i=0; i<pl.size()+1; i++) {
    marker.points[i].x = pl[i%pl.size()](0);
    marker.points[i].y = pl[i%pl.size()](1);
    marker.points[i].z = pl[i%pl.size()](2);
  }

  marker_pub_.publish(marker);
}

static void publishLineMarker(Eigen::Vector3f a, Eigen::Vector3f b, const int id=rand()%111111)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.lifetime = ros::Duration(id<0?0:4);
  marker.header = header_;
  //marker.header.stamp = stamp;



  //create the marker in the table reference frame
  //the caller is responsible for setting the pose of the marker to match

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = 0;
  marker.color.g = 1;
  marker.color.b = 0;
  marker.color.a = 1.0;

  marker.id = id;

  marker.points.resize(2);

  marker.points[0].x = a(0);
  marker.points[0].y = a(1);
  marker.points[0].z = a(2);

  marker.points[1].x = b(0);
  marker.points[1].y = b(1);
  marker.points[1].z = b(2);

  marker_pub_.publish(marker);
}


protected:
ros::Subscriber point_cloud_sub_;             //subscriber for input pc
ros::NodeHandle n_;

};

#include <pcl/surface/mls.h>

#include <registration/general_registration.h>
#include <registration/registration_info.h>
#include <boost/thread.hpp>

#define OCTOMAP_ 0
class TestNode2
{
  typedef pcl::PointXYZRGB Point;

  Registration_Infobased<Point> reg;
  pcl::PointCloud<pcl::PointXYZRGB> pcreg;

  Eigen::Matrix4f tf_;
  sensor_msgs::PointCloud2ConstPtr temporal_pcmsg_;
  boost::mutex mutex_, mutex2_;
  boost::thread *pthread_;
protected:
  ros::Subscriber point_cloud_sub_;             //subscriber for input pc
  ros::NodeHandle n_;

  bool first_, build_map_always_;

  //DEBUG
  int registered_fr_;
public:
  // Constructor
  TestNode2():
    tf_(Eigen::Matrix4f::Identity()), first_(true), build_map_always_(false), registered_fr_(0)
  {
    point_cloud_sub_ = n_.subscribe("/camera/rgb/points", 1, &TestNode2::pointCloudSubCallback, this);
    point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2 >("point_cloud2_resized",1);
#if OCTOMAP_
    map_pub_ = n_.advertise<sensor_msgs::PointCloud2 >("cloud_in",1);
#else
    map_pub_ = n_.advertise<sensor_msgs::PointCloud2 >("point_cloud_map",1);
#endif
    marker_pub_ = n_.advertise<visualization_msgs::Marker>("reg_markers", 1);

    build_map_always_=false;

    reg.setThresholdDiff(0.06);
    reg.setThresholdStep(0.06);
    reg.setMinInfo(2);
    reg.setMaxInfo(16);
    //reg.setMinChanges(500);
    //reg.setUseICP(true);

    mutex2_.lock();
    pthread_ = new boost::thread(&TestNode2::createMap, this);
  }



  // Destructor
  ~TestNode2()
  {
    pthread_->detach();
    delete pthread_;
  }

  void
  dofile(const char *fn)
  {

    std::cout<<"loading "<<fn<<"\n";

    pcl::PointCloud<Point> pc;
    pcl::io::loadPCDFile(fn,pc);

    PrecisionStopWatch psw2;

    reg.setInputOginalCloud(pc.makeShared());

    ++registered_fr_;
    PrecisionStopWatch psw;
    bool success = reg.compute();
    ROS_ERROR("took1 %f", psw.precisionStop());
    std::cout<<"Success: "<<success<<"\n";
    std::cout<<"No. frames: "<<registered_fr_<<"\n";
    std::cout<<reg.getTransformation()<<"\n";

    pcl::PointCloud<Point> pcO, pcN;
    reg.getClouds(pcO,pcN);

    sensor_msgs::PointCloud2 pc_out;
    pcl::toROSMsg(*reg.getMarkers2(),pc_out);
    pc_out.header.frame_id="openni_camera";
    point_cloud_pub_.publish(pc_out);

    if(!success && pcreg.size()>0 && !first_) {
      ROS_ERROR("took2 %f", psw2.precisionStop());
      return;
    }

    first_=false;

    ROS_ERROR("took2 %f", psw2.precisionStop());

    if(success) {
      pcl::PointCloud<pcl::PointXYZRGB> pc_rgb;
      pcl::io::loadPCDFile(fn,pc_rgb);

      pcl::transformPointCloud(pc_rgb,pc_rgb,reg.getTransformation());
      pc_rgb.header.frame_id = pcreg.header.frame_id;
      pcreg += pc_rgb;

      pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
      voxel.setInputCloud(pcreg.makeShared());
      voxel.setLeafSize(0.01,0.01,0.01);
      voxel.filter(pcreg);

      pcl::toROSMsg(pcreg,pc_out);
      pc_out.header.frame_id="openni_camera";
      map_pub_.publish(pc_out);
    }
  }

  void
  pointCloudSubCallback(const sensor_msgs::PointCloud2ConstPtr& pc_in)
  {
    static int counter=0;
    if(++counter<5) return;
    PrecisionStopWatch psw2;

    pcl::PointCloud<Point> pc;
    pcl::fromROSMsg(*pc_in,pc);

    for(size_t i=0; i<pc.size(); i++) {
      if(pc[i].z>3.)
        pc[i].z=pc[i].y=pc[i].x=10.f;//std::numeric_limits<float>::quiet_NaN();
    }

    reg.setInputOginalCloud(pc.makeShared());

    ++registered_fr_;
    PrecisionStopWatch psw;
    bool success = reg.compute();
    ROS_INFO("took1 %f", psw.precisionStop());
    std::cout<<"Success: "<<success<<"\n";
    std::cout<<"No. frames: "<<registered_fr_<<"\n";
    //std::cout<<reg.getTransformation()<<"\n";

    pcl::PointCloud<Point> pcO, pcN;
    reg.getClouds(pcO,pcN);

    sensor_msgs::PointCloud2 pc_out;
    pcl::toROSMsg(*reg.getMarkers2(),pc_out);
    pc_out.header.frame_id="openni_camera";
    point_cloud_pub_.publish(pc_out);

    if(!success && pcreg.size()>0 && !first_) {
      ROS_INFO("took2 %f", psw2.precisionStop());

      if(reg.getBadCounter()>100) {
        pcreg.clear();
        reg.reset();
        if(mutex_.try_lock()) {
          tf_=tf_.Identity();
          temporal_pcmsg_=pc_in;
          mutex2_.unlock();
          mutex_.unlock();
        }
      }
      return;
    }

    first_=false;

    //findTF(pcO,pcN).inverse();
    if(mutex_.try_lock()) {
      tf_=reg.getTransformation();
      temporal_pcmsg_=pc_in;
      mutex2_.unlock();
      mutex_.unlock();
    }

    ROS_INFO("took2 %f", psw2.precisionStop());

  }

  void createMap()
  {

    Eigen::Matrix4f last=Eigen::Matrix4f::Identity();

    while(ros::ok())
    {
      mutex2_.lock();

      mutex_.lock();
      Eigen::Matrix4f tf=tf_;
      sensor_msgs::PointCloud2ConstPtr pc_in=temporal_pcmsg_;
      mutex_.unlock();

      if(!pc_in)
        continue;

      Eigen::Matrix4f T=last.inverse()*tf;
      if(build_map_always_ || pcreg.size()<1 || T.col(3).head<3>().squaredNorm()>0.01 || Eigen::Quaternionf(T.topLeftCorner<3, 3> ()).angularDistance(Eigen::Quaternionf::Identity())>0.1)
      {
        ROS_INFO("building map");
#if OCTOMAP_
        map_pub_.publish(pc_in);
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        Eigen::Vector4f t = tf.col(3);
        Eigen::Quaternionf R(T.topLeftCorner<3, 3> ());
        transform.setOrigin( tf::Vector3(t(0),t(1),t(2)) );
        transform.setRotation( tf::Quaternion(R.x(),R.y(),R.z(),R.w()) );
        br.sendTransform(tf::StampedTransform(transform, pc_in->header.stamp, "map", pc_in->header.frame_id));
#else

        pcl::PointCloud<pcl::PointXYZRGB> pc_rgb;
        pcl::fromROSMsg(*pc_in,pc_rgb);

        for(int i=0; i<pc_rgb.size(); i++) {
          if(pc_rgb[i].z>4.)
          {
            pc_rgb[i].z=pc_rgb[i].y=pc_rgb[i].x=std::numeric_limits<float>::quiet_NaN();
          }
        }
        pc_rgb.width=pc_rgb.size();
        pc_rgb.height=1;

        pcl::transformPointCloud(pc_rgb,pc_rgb,tf);
        pc_rgb.header.frame_id = pcreg.header.frame_id;
        pcreg += pc_rgb;

        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        voxel.setInputCloud(pcreg.makeShared());
        voxel.setLeafSize(0.01,0.01,0.01);
        voxel.filter(pcreg);

        sensor_msgs::PointCloud2 pc_out;
        pcl::toROSMsg(pcreg,pc_out);
        pc_out.header.frame_id="openni_camera";
        map_pub_.publish(pc_out);

        pcl::io::savePCDFileBinary("map.pcd",pcreg);

        last=tf;
#endif
      }

      usleep(1000*250);
    }

  }
};

int main(int argc, char **argv) {


  if(1){
    ros::init(argc, argv, "dynamic_tutorials");

    TestNode2 tn;

    ROS_INFO("Spinning node");
    ros::spin();

    return 0;
  }
  else {
    ros::init(argc, argv, "dynamic_tutorials");

    TestNode2 tn;
    for(int i=1; i<argc; i++)
      tn.dofile(argv[i]);
  }
  g_step = argc>1;

#if TESTING_

#if 1
  std::vector<std::string> files;
  std::string fn="/home/josh/tmp/files.txt";
  if(argc>1)
    fn=argv[1];
  std::ifstream ifs(fn.c_str());

  char buffer[256];
  while(!ifs.eof()) {
    ifs.getline(buffer,256);
    files.push_back(buffer);
  }
  std::cout<<"loading...\n";

  ros::init(argc, argv, "dynamic_tutorials");
  ros::NodeHandle n_;
  point_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2 >("point_cloud2_resized",1);
  map_pub_ = n_.advertise<sensor_msgs::PointCloud2 >("point_cloud_map",1);

  Registration_Infobased<pcl::PointXYZ> reg;

  reg.setThresholdDiff(0.07);
  reg.setThresholdStep(0.07);
  reg.setMinInfo(1);
  reg.setMaxInfo(17);
  //reg.setUseICP(true);

  Eigen::Matrix4f all_tf = Eigen::Matrix4f::Identity();
  pcl::PointCloud<pcl::PointXYZRGB> pcreg;
  pcl::PointCloud<pcl::PointXYZ> pcold;
  for(int i=0; i<files.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ> pc1_, pc1;

    pcl::io::loadPCDFile(files[i],pc1_);

    pc1=pc1_;
    for(int x=0; x<pc1.width; x++) {
      for(int y=0; y<pc1.height; y++) {
        int ic=((x)+(y)*pc1.width);
        int is=((y)+(x)*pc1.height);
        pc1[ic]=pc1_[is];
      }
    }

    for(int i=0; i<pc1.size(); i++) {
      if(pc1[i].z==0||pc1[i].z>10)
        pc1[i].z=pc1[i].y=pc1[i].x=std::numeric_limits<float>::quiet_NaN();
    }

    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();

#if 0
    sensor_msgs::PointCloud2 pc_out;

    if(i>0)
    {
      Registration_Infobased<pcl::PointXYZ> reg;

      reg.setThresholdDiff(0.1);
      reg.setThresholdStep(0.1);
      reg.setMinInfo(1);
      reg.setMaxInfo(17);

      reg.setInputOginalCloud(pcold.makeShared());
      reg.compute_features();

      reg.setInputOginalCloud(pc1.makeShared());

      PrecisionStopWatch psw;
      reg.compute_features();
      ROS_ERROR("took1 %f", psw.precisionStop());

      pcl::PointCloud<pcl::PointXYZ> pcO, pcN;
      reg.getClouds(pcO,pcN);

      sensor_msgs::PointCloud2 pc_out;
      pcl::toROSMsg(*reg.getMarkers2(),pc_out);
      pc_out.header.frame_id="openni_camera";
      map_pub_.publish(pc_out);

      {
        PrecisionStopWatch psw;
        //calcTF<pcl::PointXYZ> ctf(pcO,pcN);
        tf=findTF_fast(pcO,pcN).inverse();
        //tf=findTF(pcO,pcN).inverse();
        ROS_ERROR("took2 %f", psw.precisionStop());
      }

      std::cout<<tf<<"\n";
    }
#else
    reg.setInputOginalCloud(pc1.makeShared());

    PrecisionStopWatch psw;
    reg.compute();
    ROS_ERROR("took1 %f", psw.precisionStop());

    all_tf = reg.getTransformation();

    pcl::PointCloud<pcl::PointXYZ> pcO, pcN;
    reg.getClouds(pcO,pcN);

    sensor_msgs::PointCloud2 pc_out;
    pcl::toROSMsg(*reg.getMarkers2(),pc_out);
    pc_out.header.frame_id="openni_camera";
    map_pub_.publish(pc_out);

    /* {
      PrecisionStopWatch psw;
      //calcTF<pcl::PointXYZ> ctf(pcO,pcN);
      tf=findTF_fast(pcO,pcN).inverse();
      //tf=findTF(pcO,pcN).inverse();
      ROS_ERROR("took2 %f", psw.precisionStop());
      }*/

    std::cout<<all_tf<<"\n";
#endif

    pcold=pc1;

    all_tf=tf*all_tf;

    pcl::PointCloud<pcl::PointXYZRGB> tmp;
    pcl::io::loadPCDFile(files[i],tmp);
    pcl::transformPointCloud(tmp,tmp,all_tf);
    pcreg += tmp;

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(pcreg.makeShared());
    voxel.setLeafSize(0.01,0.01,0.01);
    voxel.filter(pcreg);

    pcl::io::savePCDFileBinary("out/a.pcd",pcreg);

    pcl::toROSMsg(pcreg,pc_out);
    pc_out.header.frame_id="openni_camera";
    point_cloud_pub_.publish(pc_out);

    std::cout<<"DONE with "<<i<<"\n";
    if(getchar()=='q')
      return 0;
  }
#elif 1
  pcl::PointCloud<pcl::PointXYZ> pc1, pc2;
  pcl::PointCloud<pcl::PointXYZ> pc1_, pc2_;
  pcl::PointCloud<pcl::PointXYZ> pcO, pcN;


  pcl::io::loadPCDFile(_FN1_,pc1_);
  pcl::io::loadPCDFile(_FN2_,pc2_);

  pc1=pc1_;
  pc2=pc2_;

  for(int x=0; x<pc1.width; x++) {
    for(int y=0; y<pc1.height; y++) {
      int ic=((x)+(y)*pc1.width);
      int is=((y)+(x)*pc1.height);
      pc1[ic]=pc1_[is];
      pc2[ic]=pc2_[is];
    }
  }

  for(int i=0; i<pc1.size(); i++) {
    if(pc1[i].z==0||pc1[i].z>10)
      pc1[i].z=pc1[i].y=pc1[i].x=std::numeric_limits<float>::quiet_NaN();
  }

  for(int i=0; i<pc2.size(); i++) {
    if(pc2[i].z==0||pc2[i].z>10)
      pc2[i].z=pc2[i].y=pc2[i].x=std::numeric_limits<float>::quiet_NaN();
  }

  while(1) {
    Registration_Infobased<pcl::PointXYZ> reg;

    float f;
    int n;

    reg.setThresholdDiff(0.1);
    reg.setThresholdStep(0.02);
    reg.setMinInfo(1);
    reg.setMaxInfo(17);

    /*std::cout<<"thr diff ";
    std::cin>>f;
    reg.setThresholdDiff(f);

    std::cout<<"thr step ";
    std::cin>>f;
    reg.setThresholdStep(f);

    std::cout<<"min i ";
    std::cin>>n;
    reg.setMinInfo(n);

    std::cout<<"max i ";
    std::cin>>n;
    reg.setMaxInfo(n);*/

    reg.setInputOginalCloud(pc1.makeShared());
    reg.compute_features();

    reg.setInputOginalCloud(pc2.makeShared());
    reg.compute_features();

    /*pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(reg.getMarkers2(),"p1");

    while (!viewer.wasStopped ())
    {
      usleep(1000);
    }*/

    reg.getClouds(pcO,pcN);

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(pcO.makeShared());
    voxel.setLeafSize(0.1,0.1,0.1);
    voxel.setLeafSize(0.01,0.01,0.01);
    voxel.setLeafSize(0.03,0.03,0.03);
    //voxel.filter(pcO);

    voxel.setInputCloud(pcN.makeShared());
    voxel.setLeafSize(0.1,0.1,0.1);
    voxel.setLeafSize(0.01,0.01,0.01);
    voxel.setLeafSize(0.03,0.03,0.03);
    //voxel.filter(pcN);

    {
      PrecisionStopWatch psw;
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setInputCloud(pcO.makeShared());
      icp.setInputTarget(pcN.makeShared());
      pcl::PointCloud<pcl::PointXYZ> Final;
      icp.align(Final);
      ROS_ERROR("took %f", psw.precisionStop());
      std::cout << "has converged:" << icp.hasConverged() << " score: " <<
          icp.getFitnessScore() << std::endl;
      std::cout << icp.getFinalTransformation() << std::endl;

      std::vector<COR_S> cors;
      pcl::PointCloud<pcl::PointXYZ> tpc;
      pcl::transformPointCloud(pc1,tpc,icp.getFinalTransformation());
      visualize(tpc,pc2, cors);

      pcl::PointCloud<pcl::PointXYZRGB> pc1_,pc2_;
      pcl::io::loadPCDFile("/home/josh/tmp/1305031229.564442.png.img.pcd",pc1_);
      pcl::io::loadPCDFile("/home/josh/tmp/1305031229.596617.png.img.pcd",pc2_);

      pcl::transformPointCloud(pc1_,pc1_,icp.getFinalTransformation());

      pcl::io::savePCDFileBinary("out/a.pcd",pc1_);
      pcl::io::savePCDFileBinary("out/b.pcd",pc2_);
    }


    calcTF<pcl::PointXYZ> ctf(pcO,pcN);

    std::cout<<ctf.findCloud()<<"\n";

  }

#elif 1
  pcl::PointCloud<pcl::PointXYZ> pc1, pc2;
  //pcl::io::loadPCDFile("Op"+std::string(argv[1])+".pcd",pc1);
  //pcl::io::loadPCDFile("Np"+std::string(argv[1])+".pcd",pc2);
  pcl::io::loadPCDFile("/home/josh/tmp/Op6.pcd",pc1);
  pcl::io::loadPCDFile("/home/josh/tmp/Np6.pcd",pc2);
  //pcl::io::loadPCDFile("pcn.pcd",pc1);
  //pcl::io::loadPCDFile("pco.pcd",pc2);

  for(int i=0; i<pc1.size(); i++)
    if(!pcl_isfinite(pc1[i].x)) {pc1.points.erase(pc1.points.begin()+i);--i;}
  for(int i=0; i<pc2.size(); i++)
    if(!pcl_isfinite(pc2[i].x)) {pc2.points.erase(pc2.points.begin()+i);--i;}


  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setMeanK (5);
  sor.setStddevMulThresh (0.05);
  sor.setNegative(true);

  /*sor.setInputCloud (pc1.makeShared());
  sor.filter (pc1);

  sor.setInputCloud (pc2.makeShared());
  sor.filter (pc2);*/

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(pc1.makeShared());
  voxel.setLeafSize(0.03,0.03,0.03);
  voxel.setLeafSize(0.1,0.1,0.1);
  voxel.setLeafSize(0.01,0.01,0.01);
  voxel.filter(pc1);

  voxel.setInputCloud(pc2.makeShared());
  voxel.setLeafSize(0.03,0.03,0.03);
  voxel.setLeafSize(0.1,0.1,0.1);
  voxel.setLeafSize(0.01,0.01,0.01);
  voxel.filter(pc2);

  calcTF<pcl::PointXYZ> ctf(pc1,pc2);

  std::cout<<ctf.findCloud()<<"\n";


  {
    PrecisionStopWatch psw;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(pc1.makeShared());
    icp.setInputTarget(pc2.makeShared());
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    ROS_ERROR("took %f", psw.precisionStop());
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    std::vector<COR_S> cors;
    pcl::PointCloud<pcl::PointXYZ> tpc;
    pcl::transformPointCloud(pc1,tpc,icp.getFinalTransformation());
    visualize(tpc,pc2, cors);
  }


#else
  for(int j=0; j<3; j++) {
    srand(clock());
    pcl::PointCloud<pcl::PointXYZ> pc1, pc2;
    for(int i=0; i<500; i++) {
      pcl::PointXYZ p;
      for(int j=0; j<2; j++) {
        p.x = (rand()%500-250)/250.;
        p.y = (rand()%500-250)/250.;
        p.z = 1+(rand()%200)/45.;

        pc1.points.push_back(p);}

      p.x = i/100. + (rand()%100)/5000.;
      p.y = 1 + (rand()%100)/5000.;
      p.z = 2+i/100. + (rand()%100)/5000.;
      pc1.points.push_back(p);
      p.y = i/100. + (rand()%100)/5000.;
      p.x = 1 + (rand()%100)/5000.;
      p.z = 2 + (rand()%100)/5000.;
      pc1.points.push_back(p);
    }

    /*pcl::io::loadPCDFile("/home/josh/svn/pcl-trunk/test/bunny.pcd",pc1);
    pcl::transformPointCloud(pc1,pc1,Eigen::Matrix4f::Identity()*3);
    for(int i=0; i<pc1.size(); i++) pc1[i].z+=3;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(pc1.makeShared());
    voxel.setLeafSize(0.02,0.02,0.02);
    voxel.filter(pc1);*/

    Eigen::Vector3f va,vb;
    //va(2)=0.5;va(1)=1;va(0)=0;
    va(2)=0;va(1)=1;va(0)=0;
    vb(2)=1;vb(1)=0;vb(0)=0.1;

    std::swap(va(j),va(2));
    std::swap(vb(j),vb(2));

    Eigen::Quaternionf qq;
    Eigen::Matrix3f R;
    qq.setFromTwoVectors(va,vb);
    R=qq.toRotationMatrix();

    va.normalize();
    std::cout<<"correct axis\n"<<va<<"\n";
    Eigen::AngleAxisf aa(-0.05, va);
    R=aa.toRotationMatrix();
    Eigen::Matrix4f ttt=Eigen::Matrix4f::Identity();
    //R=qq.toRotationMatrix();
    for(int x=0; x<3; x++)
      for(int y=0; y<3; y++)
        ttt(x,y) = R(x,y);
    ttt(1,3)=0.05;
    pcl::transformPointCloud(pc1,pc2,ttt);

    std::cout<< ttt*pc1[0].getVector4fMap()<<"\n";
    std::cout<< pc2[0].getVector4fMap()<<"\n";

    for(int i=0; i<100; i++)
      pc2.points.erase(pc2.points.begin()+(rand()%pc2.size()));
    for(int i=0; i<pc2.size(); i++) {
      pc2[i].x+=(rand()%101-50)*0.0001;
      pc2[i].y+=(rand()%101-50)*0.0001;
      pc2[i].z+=(rand()%101-50)*0.0001;
    }
    for(int i=0; i<100; i++) {
      pcl::PointXYZ p;
      p.x = (rand()%500-250)/50.;
      p.y = (rand()%500-250)/50.;
      p.z = 1+(rand()%20)/5.;

      pc2.points.push_back(p);
    }

    //std::cout<<findTF(pc1,pc2)<<"\n";

    calcTF<pcl::PointXYZ> ctf(pc1,pc2);

    ctf.findCloud();

    {
      PrecisionStopWatch psw;
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setInputCloud(pc1.makeShared());
      icp.setInputTarget(pc2.makeShared());
      pcl::PointCloud<pcl::PointXYZ> Final;
      icp.align(Final);
      ROS_ERROR("took %f", psw.precisionStop());
      std::cout << "has converged:" << icp.hasConverged() << " score: " <<
          icp.getFitnessScore() << std::endl;
      std::cout << icp.getFinalTransformation() << std::endl;
    }

    //std::cout<<"TRANS\n"<<getTF3(pc1[0].getVector3fMap(),pc1[10].getVector3fMap(),pc1[11].getVector3fMap(), pc2[0].getVector3fMap(),pc2[10].getVector3fMap(),pc2[11].getVector3fMap())<<"\n";
    std::cout<<"CORRECT TRANS\n"<<ttt<<"\n";
    //std::cout<<"CORRECT TRANS\n"<<ttt*ttt.transpose()<<"\n";
    getchar();

    //std::cout<<findTF(pc1,pc2)<<"\n";
    std::cout<<"CORRECT TRANS\n"<<ttt<<"\n";
  }
#endif
  return 0;
#endif

  ros::init(argc, argv, "dynamic_tutorials");

  TestNode tn;

  ROS_INFO("Spinning node");
  ros::spin();

  return 0;
}
