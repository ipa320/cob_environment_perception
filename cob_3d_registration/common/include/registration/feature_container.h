

#ifndef FEATURE_CONTAINER_H_
#define FEATURE_CONTAINER_H_


class FeatureContainerInterface
{
public:
  virtual bool isValid () = 0;
  virtual void findFeatureCorrespondences (int index, std::vector<int> &correspondence_indices,
                                           std::vector<float> &distances) = 0;
};

template <typename FeatureType>
class FeatureContainer : public FeatureContainerInterface
{
public:
  typedef typename pcl::PointCloud<FeatureType>::ConstPtr FeatureCloudConstPtr;

#ifdef PCL_VERSION_COMPARE
  typedef typename pcl::search::KdTree<FeatureType> KdTree;
  typedef typename pcl::search::KdTree<FeatureType>::Ptr KdTreePtr;
#else
  typedef typename pcl::KdTree<FeatureType> KdTree;
  typedef typename pcl::KdTree<FeatureType>::Ptr KdTreePtr;
#endif
  typedef boost::function<int (const pcl::PointCloud<FeatureType> &, int, std::vector<int> &,
                               std::vector<float> &)> SearchMethod;

  FeatureContainer () : k_(0), radius_(0) {}

  void setSourceFeature (const FeatureCloudConstPtr &source_features)
  {
    source_features_ = source_features;
  }

  FeatureCloudConstPtr getSourceFeature ()
  {
    return (source_features_);
  }

  void setTargetFeature (const FeatureCloudConstPtr &target_features)
  {
    target_features_ = target_features;
    if (tree_)
    {
      tree_->setInputCloud (target_features_);
    }
  }

  FeatureCloudConstPtr getTargetFeature ()
  {
    return (target_features_);
  }

  void setRadiusSearch (KdTreePtr tree, float r)
  {
    tree_ = tree;
    radius_ = r;
    k_ = 0;
    if (target_features_)
    {
      tree_->setInputCloud (target_features_);
    }
  }

  void setKSearch (KdTreePtr tree, int k)
  {
    tree_ = tree;
    k_ = k;
    radius_ = 0.0;
    if (target_features_)
    {
      tree_->setInputCloud (target_features_);
    }
  }

  virtual bool isValid ()
  {
    if (!source_features_ || !target_features_ || !tree_)
    {
      return (false);
    }
    else
    {
      return (source_features_->points.size () > 0 &&
          target_features_->points.size () > 0 &&
          (k_ > 0 || radius_ > 0.0));
    }
  }

  virtual void findFeatureCorrespondences (int index, std::vector<int> &correspondence_indices,
                                           std::vector<float> &distances)
  {
    if (k_ > 0)
    {
      correspondence_indices.resize (k_);
      distances.resize (k_);
      tree_->nearestKSearch (*source_features_, index, k_, correspondence_indices, distances);
    }
    else
    {
      tree_->radiusSearch (*source_features_, index, radius_, correspondence_indices, distances);
    }
  }

private:
  FeatureCloudConstPtr source_features_, target_features_;
  KdTreePtr tree_;
  SearchMethod search_method_;
  int k_;
  double radius_;
};

template <typename Point>
class FeatureContainerInterface_Euclidean : public FeatureContainerInterface
{
protected:
  bool build_;
  float radius2_;
  pcl::PointCloud<Point> org_in_, org_out_;
#ifdef PCL_VERSION_COMPARE
  boost::shared_ptr<pcl::search::KdTree<Point> > tree_;
#else
  boost::shared_ptr<pcl::KdTree<Point> > tree_;
#endif
public:

  FeatureContainerInterface_Euclidean():
    build_(false), radius2_(0.05)
  {
  }

  void setSearchRadius(float v) {radius2_ = v;}

  virtual void build(const pcl::PointCloud<Point> &src, const pcl::PointCloud<Point> &tgt) {
    org_in_=src;
    org_out_=tgt;

#ifdef PCL_VERSION_COMPARE
    tree_.reset (new pcl::search::KdTree<Point>);
#else
    tree_.reset (new pcl::KdTreeFLANN<Point>);
#endif
    if(org_in_.size()>0) tree_->setInputCloud(org_in_.makeShared());

    build_ = hidden_build();
  }

  virtual bool isValid () {return build_;}
  virtual void findFeatureCorrespondences (int index, std::vector<int> &correspondence_indices,
                                           std::vector<float> &distances) {
    correspondence_indices.clear();
    distances.clear();

    int num_k2_=2;
    float border_=111;

    std::vector<int> inds;
    std::vector<float> dis;
    tree_->radiusSearch(org_out_.points[index], radius2_, inds, dis);
    if(inds.size()<1)
      tree_->nearestKSearch(org_out_.points[index], num_k2_, inds, dis);

    if(inds.size()<1)
      return;
    float min_dis=border_*border_;
    Eigen::VectorXf b = getFeatureOut(index);
    int mi=inds[0];
    for(int i=0; i<inds.size(); i++) {
      Eigen::VectorXf a = getFeatureIn(inds[i]);
      if( (a-b).squaredNorm()*(a-b).squaredNorm()*(org_in_.points[inds[i]].getVector3fMap()-org_out_.points[index].getVector3fMap()).squaredNorm()<min_dis ) {
        min_dis = (a-b).squaredNorm()*(a-b).squaredNorm()*(org_in_.points[inds[i]].getVector3fMap()-org_out_.points[index].getVector3fMap()).squaredNorm();
        mi = inds[i];
      }
    }
    if(mi!=-1)
      correspondence_indices.push_back(mi);

    for(int i=0; i<correspondence_indices.size(); i++) {
      distances.push_back( (org_in_.points[correspondence_indices[i]].getVector3fMap()-org_out_.points[index].getVector3fMap()).squaredNorm() );
    }
  }

  virtual Eigen::VectorXf getFeatureOut(const int)=0;
  virtual Eigen::VectorXf getFeatureIn(const int)=0;
  virtual bool hidden_build()=0;
};

#endif /* FEATURE_CONTAINER_H_ */
