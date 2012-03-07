/*
 * registration_edges.h
 *
 *  Created on: Nov 21, 2011
 *      Author: goa-jh
 */

#ifndef REGISTRATION_EDGES_H_
#define REGISTRATION_EDGES_H_


#include "registration_icp.h"
#include "features/edges.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>


template <typename Point>
class Registration_ICP_Edges : public Registration_ICP_Features<Point>
{
  Feature_Edges<Point> *mom_;
  pcl::PointCloud<Point> tmp_input_;
  bool first_;

public:
  Registration_ICP_Edges():
    mom_(new Feature_Edges<Point>), first_(true)
  {
    this->setCorrDist(0.1);
    this->setFeatures(mom_);
  }

  virtual ~Registration_ICP_Edges() {
    delete mom_;
  }

  virtual boost::shared_ptr<const pcl::PointCloud<Point> > getMarkers() {
    boost::shared_ptr<pcl::PointCloud<Point> > r(new pcl::PointCloud<Point>);
    //*r=*this->input_;
    //*r+=this->register_;
    *r=this->register_;
    return r;
  }

  void setRadius(float v) {mom_->setRadius(v);}
  void setThreshold(float v) {mom_->setThreshold(v);}
  void setDisThreshold(float v) {mom_->setDisThreshold(v);}

protected:
  virtual bool compute_features() {
    //save input first
    tmp_input_ = *this->input_org_;

    mom_->setSearchRadius(this->icp_max_corr_dist_);
    mom_->build(this->register_, *this->input_org_);

    return true;
  }

  virtual bool compute_transformation() {
    ROS_INFO("compute_transformation");
    bool b=false;

    if(first_) {
      this->register_ = mom_->getFilteredOutputCloud();
      b=true;
      first_=false;
      return b;
    }

    pcl::PointCloud<Point> tmp = this->register_,tmp2;

    //replace input cloud...
    this->register_= mom_->getFilteredInputCloud();
    this->input_  = mom_->getFilteredOutputCloud().makeShared();

    if(this->register_.size()>2&&this->input_->size()>2) {

      /*pcl::PassThrough<Point> pass;
      pass.setInputCloud (this->register_.makeShared());
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 6.0);
      pass.filter (this->register_);
      pass.setInputCloud (this->input_);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.2, 6.0);
      pass.filter (tmp2);
      this->input_  = tmp2.makeShared();*/

      /*DEBUGGING*/
      /*for(int i=1389; i<std::min((int)this->register_.size(),1390); i++)
        std::cout<<this->register_[i]<<" ";
      for(int i=1389; i<std::min((int)this->input_->size(),1390); i++)
        std::cout<<(*this->input_)[i]<<" ";
      {pcl::visualization::CloudViewer viewer("Cloud Viewer");

      viewer.showCloud(this->register_.makeShared(),"a");

      while (!viewer.wasStopped ())
      {
      }
      }
      {pcl::visualization::CloudViewer viewer("Cloud Viewer");

      viewer.showCloud(this->input_,"b");

      while (!viewer.wasStopped ())
      {
      }
      }*/
      /*END*/

      b=Registration_ICP_Features<Point>::compute_transformation();

      this->register_ = tmp;
      if(b) {
        //pcl::transformPointCloud(tmp_input_, tmp_input_, this->transformation_);
        this->register_ = mom_->getFilteredOutputCloud();
      }

    }

    ROS_INFO("done");
    return b;
  }
};

#endif /* REGISTRATION_EDGES_H_ */
