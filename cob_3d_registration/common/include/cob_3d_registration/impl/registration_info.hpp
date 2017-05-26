/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: registration
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Joshua Hampp
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Nov 21, 2011
 * ToDo:
 *
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

  // organized access to pointcloud
#define getInd(x, y) ((x)+(y)*pc.width)

  // evaluation mode uses every result, otherwise only if error is below 3%
#define EVALUATION_MODE_ 0

  // debug mode outputs more information on console and creates pointclouds of HIRN points
#define DEBUG_SWITCH_ 0

  // using odometry to determine keyframes -> set to one was workaround for "standing"-check
#define USED_ODO_ 1


namespace cob_3d_registration {

  template <typename Point>
  bool Registration_Infobased<Point>::compute_features()
  {
    if(!this->input_org_)
      return false;

    this->scene_changed_=false;

    indices_pos2.clear();
    indices_neg2.clear();

    // first frame -> frame is next frame to register against
    if(!this->last_input_) {
      this->last_input2_ = this->last_input_ = this->input_org_;
      register_ = *this->input_org_;
      return false;
    }
    if(kinect_f_==0) getKinectParameters();

    reproject();
    if(!this->last_input2_)
      return false;

    const pcl::PointCloud<Point> &pc_old  = *this->last_input2_;
    const pcl::PointCloud<Point> &pc      = *this->input_org_;

#if DEBUG_SWITCH_
    assert(pc_old.size()==pc.size());
#endif

    if(!depth_map)
      depth_map=new unsigned char[pc.height*pc.width];

    //diff
    std::vector<int> indices_pos, indices_neg;

    pcl::PointCloud<Point> _p;
    for(int y=0; y<(int)pc.height; y++) {
      for(int x=0; x<(int)pc.width; x++) {

        int ind = getInd(x,y);
        const Point &pn=pc.points[ind];
        const Point &po=pc_old.points[ind];

        if(std::min(pn.z,po.z)>4)
          continue;

        float d = pn.z-po.z;

        // threshold is calculated in consideration of quantization error
        const float di = std::max(pn.z,po.z) * threshold_diff_;
        if(d>di) {
          depth_map[ind]=1;
          indices_pos.push_back(ind);}
        else if(d<-di) {
          depth_map[ind]=1;
          indices_neg.push_back(ind);
        }
        else
          depth_map[ind]=0;

      }
    }

#if DEBUG_SWITCH_
    ROS_INFO("found  %d %d", indices_pos.size(), indices_neg.size());
#endif

    if((int)(indices_pos.size()+indices_neg.size())<min_changes_ /*|| std::min(indices_pos.size(), indices_neg.size())<100*/)
    {
      if(this->moved_)
      {
        standing_++;
        if(always_relevant_changes_ && standing_>15) {
          odo_is_good_=true;
          standing_=0;
          return true;
        }
      }
      return false;
    }

    //find HIRN points (instable points) which are lying on edges
    for(unsigned int i=0; i<indices_pos.size(); i++) {
      int mi;
      int info=getI(indices_pos[i], pc);
      if( info<max_info_ && info>min_info_) {
        if(getMaxDiff(pc_old, indices_pos[i])>threshold_step_)
          indices_pos2.push_back(indices_pos[i]);
        else if(getMaxDiff2(pc, indices_pos[i], pc_old, mi)>threshold_step_)
          indices_neg2.push_back(mi);
      }
    }
    for(unsigned int i=0; i<indices_neg.size(); i++) {
      int mi;
      int info=getI(indices_neg[i], pc_old);
      if( info<max_info_ && info>min_info_ ) {
        if(getMaxDiff(pc, indices_neg[i])>threshold_step_) {
          indices_neg2.push_back(indices_neg[i]);
        }
        else if(getMaxDiff2(pc_old, indices_neg[i], pc, mi)>threshold_step_)
          indices_pos2.push_back(mi);
      }
    }

#if DEBUG_SWITCH_
    markers_.clear();
    for(int i=0; i<indices_pos2.size(); i++)
    {
      pcl::PointXYZRGB p;
      p.x=pc_old.points[indices_pos2[i]].x;
      p.y=pc_old.points[indices_pos2[i]].y;
      p.z=pc_old.points[indices_pos2[i]].z;
      p.g=p.b=0;
      p.r=255;
      markers_.points.push_back(p);
    }
    for(int i=0; i<indices_neg2.size(); i++)
    {
      pcl::PointXYZRGB p;
      p.x=pc.points[indices_neg2[i]].x;
      p.y=pc.points[indices_neg2[i]].y;
      p.z=pc.points[indices_neg2[i]].z;
      p.g=p.r=0;
      p.b=255;
      markers_.points.push_back(p);
    }
    markers_.width=markers_.size();
    markers_.height=1;

    ROS_INFO("found2 %d %d", indices_pos2.size(), indices_neg2.size());
#endif

    this->scene_changed_=true;

#if USED_ODO_
    return this->moved_;
#elif USE_INFINITE_
    return std::min(indices_pos2.size(), indices_neg2.size())>400;
#else
    return std::min(indices_pos2.size(), indices_neg2.size())>200;
#endif
  }

  template <typename Point>
  bool Registration_Infobased<Point>::compute_corrospondences()
  {
    return true;
  }

  template <typename Point>
  bool Registration_Infobased<Point>::compute_transformation()
  {
    Eigen::Matrix4f T;

    this->failed_++;
    this->standing_=0;

    // if many points are HIRN points start from beginning with ICP as its faster in this case
    if(odo_is_good_){
      T=T.Identity();
      ROS_INFO("using odometry because it's good");
    }
    else if(use_icp_ || indices_pos2.size()+indices_neg2.size()>10000 ) {
      //do ICP
      ModifiedICP<Point> icp;

      const pcl::PointCloud<Point> &pc_old  = *this->last_input2_;
      const pcl::PointCloud<Point> &pc      = *this->input_org_;

      pcl::PointCloud<Point> tmp_pc_old, tmp_pc_new;
      for(int i=0; i<(int)indices_pos2.size(); i++)
        tmp_pc_old.points.push_back(pc_old.points[indices_pos2[i]]);
      for(int i=0; i<(int)indices_neg2.size(); i++)
        tmp_pc_new.points.push_back(pc.points[indices_neg2[i]]);

      icp.setInputSource( tmp_pc_new.makeShared() );
      //icp.setIndices(boost::make_shared<pcl::PointIndices>(indices));
      icp.setInputTarget(tmp_pc_old.makeShared());
      icp.setMaximumIterations(40);
      icp.setRANSACOutlierRejectionThreshold(0.05);
      icp.setMaxCorrespondenceDistance(0.5);
      icp.setTransformationEpsilon (0.0001);

      pcl::PointCloud<Point> result;
      icp.align(result);

      T = icp.getFinalTransformation();

      //check if transformation ist within max.
      if(T.col(3).head<3>().squaredNorm()>tmax_*tmax_)
        T=T.Identity();
      if(Eigen::Quaternionf(T.topLeftCorner<3, 3> ()).angularDistance(Eigen::Quaternionf::Identity())>rmax_)
        T=T.Identity();

      if(check_samples_ && !checkSamples(T)) {
        if(!use_odometry_ || this->failed_<10)
          return false;
        else {
          T = T.Identity();
#if DEBUG_SWITCH_
          ROS_INFO("using odometry");
#endif
        }
      }

    }
    else {
      TransformationEstimationMultipleCorrespondences<Point> tf_est;

      tf_est.setMaxAngularDistance(rmax_);
      tf_est.setMaxTranslationDistance(tmax_);

      const pcl::PointCloud<Point> &pc_old  = *this->last_input2_;
      const pcl::PointCloud<Point> &pc      = *this->input_org_;

      pcl::PointCloud<Point> tmp_pc_old, tmp_pc_new;
      for(int i=0; i<(int)indices_pos2.size(); i++)
        tmp_pc_old.points.push_back(pc_old.points[indices_pos2[i]]);
      for(int i=0; i<(int)indices_neg2.size(); i++)
        tmp_pc_new.points.push_back(pc.points[indices_neg2[i]]);

      T = tf_est.compute(tmp_pc_old,tmp_pc_new).inverse();

      //check if transformation ist within max.
      if(T.col(3).head<3>().squaredNorm()>tmax_*tmax_)
        T=T.Identity();
      if(Eigen::Quaternionf(T.topLeftCorner<3, 3> ()).angularDistance(Eigen::Quaternionf::Identity())>rmax_)
        T=T.Identity();

      int bad=100;
      if(check_samples_) {
        bool b=checkSamples(T, &bad);

        if(bad>2) { // if error is ove 2% switch to ICP -> slower
          ModifiedICP<Point> icp;
          icp.setInputSource( tmp_pc_new.makeShared() );
          //icp.setIndices(boost::make_shared<pcl::PointIndices>(indices));
          icp.setInputTarget(tmp_pc_old.makeShared());
          icp.setMaximumIterations(40);
          icp.setRANSACOutlierRejectionThreshold(0.05);
          icp.setMaxCorrespondenceDistance(0.2);
          icp.setTransformationEpsilon (0.0001);

          pcl::PointCloud<Point> result;
          if(b)
            icp.align(result,T);
          else
            icp.align(result);
          T = icp.getFinalTransformation();
        }

        checkSamples(T, &bad);
        if(bad>2) {
#if EVALUATION_MODE_
          T=T.Identity();
#else
          bad_counter_++;
          if(!use_odometry_ || this->failed_<10)
            return false;
        }
        else {
          T = T.Identity();
          ROS_INFO("using odometry");
        }
#endif
      }


    }

    if(T.col(3).head<3>().squaredNorm()>9*tmax_*tmax_)
      T=T.Identity();
    if(Eigen::Quaternionf(T.topLeftCorner<3, 3> ()).angularDistance(Eigen::Quaternionf::Identity())>3*rmax_)
      T=T.Identity();

    this->transformation_ = this->transformation_*T;

    this->last_input_ = this->input_org_;
    this->odometry_last_ = this->odometry_;
    this->failed_ = 0;

    return true;
  }

#define INFO_SEARCH_RADIUS_ 1

  template <typename Point>
  float Registration_Infobased<Point>::getMaxDiff(const pcl::PointCloud<Point> &pc,const int ind) {
    int x=ind%pc.width;
    int y=ind/pc.width;
    if(x<INFO_SEARCH_RADIUS_||y<INFO_SEARCH_RADIUS_||x>=(int)pc.width-INFO_SEARCH_RADIUS_||y>=(int)pc.height-INFO_SEARCH_RADIUS_)
      return 0.f;

    float z=pc.points[ind].z;

    float m=0.f;
    for(int dx=-INFO_SEARCH_RADIUS_; dx<=INFO_SEARCH_RADIUS_; dx++) {
      for(int dy=-INFO_SEARCH_RADIUS_; dy<=INFO_SEARCH_RADIUS_; dy++) {
        if(dx==0 && dy==0)
          continue;
        float t=pc.points[getInd(x+dx,y+dy)].z;
#if USE_INFINITE_
        if(!pcl_isfinite(t)) t=100;
#endif
        m=std::max(m,t-z);
      }
    }

    return m;
  }

  template <typename Point>
  float Registration_Infobased<Point>:: getMaxDiff2(const pcl::PointCloud<Point> &pc,const int ind,const pcl::PointCloud<Point> &pc2, int &mi) {
    int x=ind%pc.width;
    int y=ind/pc.width;
    if(x<INFO_SEARCH_RADIUS_||y<INFO_SEARCH_RADIUS_||x>=(int)pc.width-INFO_SEARCH_RADIUS_||y>=(int)pc.height-INFO_SEARCH_RADIUS_)
      return 0.f;

    float z=pc2.points[ind].z;

    float m=0.f, _mi=1000.f;
    for(int dx=-INFO_SEARCH_RADIUS_; dx<=INFO_SEARCH_RADIUS_; dx++) {
      for(int dy=-INFO_SEARCH_RADIUS_; dy<=INFO_SEARCH_RADIUS_; dy++) {
        if(dx==0 && dy==0)
          continue;
        float f=pc.points[getInd(x+dx,y+dy)].z-z;
#if USE_INFINITE_
        if(!pcl_isfinite(f)) f=100;
#endif
        m=std::max(m,f);
        f=std::abs(f);
        if(f<_mi) {
          _mi=f;
          mi = getInd(x+dx,y+dy);
        }
      }
    }

    if(_mi>threshold_diff_*z)
      return 0;

    return m;
  }


  template <typename Point>
  int Registration_Infobased<Point>::getI(const int ind, const pcl::PointCloud<Point> &pc) {
    int x=ind%pc.width;
    int y=ind/pc.width;
    if(x<2||y<2||x>=(int)pc.width-2||y>=(int)pc.height-2)
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

  template <typename Point>
  void Registration_Infobased<Point>::getKinectParameters()
  {
    const pcl::PointCloud<Point> &pc      = *this->input_org_;

    //get kinect paramters
    Point p1,p2;
    p1.x=p2.x=0.f;
    p1.y=p2.y=0.f;
    p1.z=p2.z=0.f;
    int i1=-1, i2=-1;

    for(int x=0; x<(int)pc.width; x+=8) {
      for(int y=0; y<(int)pc.height; y+=8) {
        int ind = getInd(x,y);
        if(pcl_isfinite(pc[ind].z)) {
          p1=pc[ind];
          i1=ind;
          x=pc.width;
          break;
        }
      }
    }

    for(int x=pc.width-1; x>=0; x-=8) {
      for(int y=pc.height-1; y>=0; y-=8) {
        int ind = getInd(x,y);
        if(pcl_isfinite(pc[ind].z)&&pc[ind].z!=p1.z&&pc[ind].z<10.f) {
          p2=pc[ind];
          i2=ind;
          x=-1;
          break;
        }
      }
    }

    if(i1==-1||i2==-1) {
      ROS_ERROR("no valid points found to detect kinect parameters, please RESTART!!!");
      return;
    }

    float ax1,ax2, bx1,bx2;
    float ay1,ay2, by1,by2;

    int x=i1%pc.width;
    int y=i1/pc.width;
    ax1=p1.z/p1.x*x;
    bx1=p1.z/p1.x;
    ay1=p1.z/p1.y*y;
    by1=p1.z/p1.y;

    x=i2%pc.width;
    y=i2/pc.width;
    ax2=p2.z/p2.x*x;
    bx2=p2.z/p2.x;
    ay2=p2.z/p2.y*y;
    by2=p2.z/p2.y;

    this->kinect_dx_ = (ax1-ax2)/(bx1-bx2);
    this->kinect_dy_ = (ay1-ay2)/(by1-by2);
    this->kinect_f_ = ax1 - bx1*this->kinect_dx_;
  }

  template <typename Point>
  bool Registration_Infobased<Point>::checkSamples(const Eigen::Matrix4f &T, int *bad_out)
  {
    const pcl::PointCloud<Point> &pc_old  = *this->last_input2_;
    const pcl::PointCloud<Point> &pc      = *this->input_org_;

    //raycast result to compare
    int found=0, bad=0;
    for(int xx=0; xx<(int)pc.width; xx+=8) {
      for(int yy=0; yy<(int)pc.height; yy+=8) {
        int ind = getInd(xx,yy);
        if(pcl_isfinite(pc[ind].z)) {
          Eigen::Vector4f v=T*pc[ind].getVector4fMap();

          if(v(2)>10.f) continue;

          int x=kinect_f_*v(0)/v(2)+kinect_dx_;
          int y=kinect_f_*v(1)/v(2)+kinect_dy_;

          if(x<0||x>=(int)pc_old.width || y<0||y>=(int)pc_old.height)
            continue;

          ++found;
          if( std::abs(pc_old[getInd(x,y)].z-v(2))>threshold_diff_*v(2)*0.5 )
            bad++;
        }
      }
    }

    if(found<10)
      return false;

#if DEBUG_SWITCH_
    std::cout<<bad<<"/"<<found<<"\n";
    std::cout<<bad*100/found<<"\n";
#endif
    if(bad_out) *bad_out = bad*100/found;

    return bad*17<found;
  }

  template <typename Point>
  void Registration_Infobased<Point>::getClouds(pcl::PointCloud<Point> &tmp_pc_old, pcl::PointCloud<Point> &tmp_pc_new)
  {
    source.clear();
    target.clear();

    if(!indices_pos2.size()||!indices_neg2.size())
      return;

    std::vector<int> _cor_inds;
    const pcl::PointCloud<Point> &pc_old  = *this->last_input2_;
    const pcl::PointCloud<Point> &pc      = *this->input_org_;

    for(size_t i=0; i<indices_pos2.size(); i++)
      tmp_pc_old.points.push_back(pc_old.points[indices_pos2[i]]);
    for(size_t i=0; i<indices_neg2.size(); i++)
      tmp_pc_new.points.push_back(pc.points[indices_neg2[i]]);
  }

  template <typename Point>
  void Registration_Infobased<Point>::reproject()
  {
    this->last_input2_ = this->last_input_;
    if(!use_odometry_ || this->kinect_f_==0)
      return;

    const Eigen::Matrix4f T = (this->odometry_last_.inverse()*this->odometry_).inverse();
    if( fabs ((T).sum ()) < 0.01 )
      return;

    pcl::PointCloud<Point> pc  = *this->last_input_;
    boost::shared_ptr<pcl::PointCloud<Point> > pc_old_new(pc.makeShared());

    for(int xx=0; xx<(int)pc.width; xx++)
      for(int yy=0; yy<(int)pc.height; yy++)
        (*pc_old_new)[getInd(xx,yy)].x = (*pc_old_new)[getInd(xx,yy)].y = (*pc_old_new)[getInd(xx,yy)].z = std::numeric_limits<float>::quiet_NaN();

    //raycast
    for(int xx=0; xx<(int)pc.width; xx++) {
      for(int yy=0; yy<(int)pc.height; yy++) {
        int ind = getInd(xx,yy);
        if(pcl_isfinite(pc[ind].z)) {
          Eigen::Vector4f v=T*pc[ind].getVector4fMap();

          int x=round(kinect_f_*v(0)/v(2)+kinect_dx_);
          int y=round(kinect_f_*v(1)/v(2)+kinect_dy_);

          if(x<0||x>=(int)pc.width || y<0||y>=(int)pc.height)
            continue;

          (*pc_old_new)[getInd(x,y)].x = v(0);
          (*pc_old_new)[getInd(x,y)].y = v(1);
          (*pc_old_new)[getInd(x,y)].z = v(2);
        }
      }
    }

    this->last_input2_ = pc_old_new;
  }

#undef getInd

}

#define PCL_INSTANTIATE_Registration_Infobased(T) template class PCL_EXPORTS cob_3d_registration::Registration_Infobased<T>;
