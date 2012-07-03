
#include "../sub_structures/poly2d.hpp"
#include "../sub_structures/labeling.h"

template <typename Point, typename PointLabel>
Segmentation_QuadRegression<Point,PointLabel>::Segmentation_QuadRegression():
MIN_LOD(8), FINAL_LOD(0), GO_DOWN_TO_LVL(3),
ch_(NULL), outline_check_(0), outline_check_size_(0)
{
}


template <typename Point, typename PointLabel>
void Segmentation_QuadRegression<Point,PointLabel>::prepare(const pcl::PointCloud<Point> &pc) {
  getKinectParams(*input_);

  int w=input_->width/2;
  int h=input_->height/2;

  for(int i=0; i<4; i++){
    levels_.push_back(SubStructure::ParamC(w,h));

    w/=2;
    h/=2;
  }

  ch_=new int[levels_[0].w*levels_[0].h];
  memset(ch_,0,levels_[0].w*levels_[0].h*4);
}

template <typename Point, typename PointLabel>
void Segmentation_QuadRegression<Point,PointLabel>::getKinectParams(const pcl::PointCloud<Point> &pc) {
  Point p1,p2;
  int i1=-1, i2=-1;

  for(size_t x=0; x<pc.width; x+=8) {
    for(size_t y=0; y<pc.height; y+=8) {
      int ind = getIndPC(x,y);
      if(pcl_isfinite(pc[ind].z)&&pc[ind].z<10.f) {
        p1=pc[ind];
        i1=ind;
        x=pc.width;
        break;
      }
    }
  }

  for(int x=pc.width-1; x>=0; x-=8) {
    for(int y=pc.height-1; y>=0; y-=8) {
      int ind = getIndPC(x,y);
      if(pcl_isfinite(pc[ind].z)&&pc[ind].z!=p1.z&&pc[ind].z<10.f) {
        p2=pc[ind];
        i2=ind;
        x=-1;
        break;
      }
    }
  }

  if(i1==-1||i2==-1) {
    ROS_WARN("no valid points");
    return;
  }

  int x=i1%pc.width;
  int y=i1/pc.width;
  float ax1,ax2, bx1,bx2;
  float ay1,ay2, by1,by2;

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

  kinect_params_.dx = (ax1-ax2)/(bx1-bx2);
  kinect_params_.dy = (ay1-ay2)/(by1-by2);
  kinect_params_.f = ax1 - bx1*kinect_params_.dx;
}


template <typename Point, typename PointLabel>
void Segmentation_QuadRegression<Point,PointLabel>::buildTree(const pcl::PointCloud<Point> &pc) {
  int j;

  //go through all levels
  for(size_t i=0; i<levels_.size(); i++) {

    SubStructure::ParamC *lvl = &levels_[i];
    if(i==0) {  ///lowest-level: take points
      Eigen::Vector3f p,t;
      for(size_t x=0; x<lvl->w; x++) {
        for(size_t y=0; y<lvl->h; y++) {
          int num=0;
          p(0)=p(1)=p(2)=0.f;

          // sub-sample 4 points to one
          j=getIndPC(2*x,2*y);
          if(pcl_isfinite(pc[j].z))
          {
            p(0)+=pc[j].x;
            p(1)+=pc[j].y;
            p(2)+=pc[j].z;
            ++num;
          }

          j=getIndPC(2*x,2*y+1);
          if(pcl_isfinite(pc[j].z))
          {
            p(0)+=pc[j].x;
            p(1)+=pc[j].y;
            p(2)+=pc[j].z;
            ++num;
          }

          j=getIndPC(2*x+1,2*y);
          if(pcl_isfinite(pc[j].z))
          {
            p(0)+=pc[j].x;
            p(1)+=pc[j].y;
            p(2)+=pc[j].z;
            ++num;
          }

          j=getIndPC(2*x+1,2*y+1);
          if(pcl_isfinite(pc[j].z))
          {
            p(0)+=pc[j].x;
            p(1)+=pc[j].y;
            p(2)+=pc[j].z;
            ++num;
          }

          j=getInd(x,y);
          lvl->data[j]=p/num;

          lvl->data[j].occopied=-1;
        }
      }
    }
    else { //other levels take lower level
      SubStructure::ParamC *lvl_prev = &levels_[i-1];

      for(size_t x=0; x<lvl->w; x++) {
        for(size_t y=0; y<lvl->h; y++) {
          j=getInd(x,y);
          lvl->data[j] = lvl_prev->data[getInd1(x*2,y*2)];
          lvl->data[j]+= lvl_prev->data[getInd1(x*2,y*2+1)];
          lvl->data[j]+= lvl_prev->data[getInd1(x*2+1,y*2)];
          lvl->data[j]+= lvl_prev->data[getInd1(x*2+1,y*2+1)];
        }
      }
    }

  }
}

template <typename Point, typename PointLabel>
void Segmentation_QuadRegression<Point,PointLabel>::calc() {

  for(int i=(int)levels_.size()-1; i>(int)(levels_.size()-GO_DOWN_TO_LVL); i--) {

    //from one corner to the other
    size_t sx=0, sy=0;
    while(sx<levels_[i].w-1 && sy<levels_[i].h-1) {
      for(int x=sx+1; x<levels_[i].w-1; x++) {
        //if(isOccupied2(i,x,sy)==-1&&!checkOccupiedDeep(i,x,sy))
        if(isOccupied(i,x,sy)==-1)
        {
          SubStructure::Model m;
          grow(m, i, x,sy);
        }
      }
      sx+=2;

      for(size_t y=sy+1; y<levels_[i].h-1; y++) {
        //if(isOccupied2(i,sx,y)==-1&&!checkOccupiedDeep(i,sx,y))
        if(isOccupied(i,sx,y)==-1)
        {
          SubStructure::Model m;
          grow(m, i, sx,y);
        }
      }
      sy+=2;
    }

  }

  //preparePolygons();
}

template <typename Point, typename PointLabel>
void Segmentation_QuadRegression<Point,PointLabel>::grow(SubStructure::Model &model, const int i, const int x, const int y) {
  static SubStructure::VISITED_LIST<SubStructure::SVALUE> list;
  list.init();
  list.add( SubStructure::SVALUE(getInd(x,y),levels_[i].w*levels_[i].h) );

  grow(list, model, i, polygons_.size(), true);
}

template <typename Point, typename PointLabel>
void Segmentation_QuadRegression<Point,PointLabel>::grow(SubStructure::VISITED_LIST<SubStructure::SVALUE> &list, SubStructure::Model &model, const int i, const int mark, bool first_lvl) {
  int x, y, hops, occ, found=0;
  bool bNew,bNew2;
  int last_size=-1;
#ifdef CHECK_CONNECTIVITY
  S_POLYGON_CONNECTIVITY connectivity;
#endif

  do {
    bNew=false;
    bNew2=false;
    while(list.pos+1<list.size) {
      list.move();

      x= list.vals[list.pos].v%levels_[i].w;
      y= list.vals[list.pos].v/levels_[i].w;
      hops = list.vals[list.pos].hops;

      occ = isOccupied(i,x,y);
      if(occ==mark) {
        if(list.pos>last_size&&hops) list.remove();
        continue;
      }
      else if(occ!=-1) {
#ifdef CHECK_CONNECTIVITY
        if(/*i==FINAL_LOD && */checkModelAt(model, i,x,y,
                                            0.02)) { //TODO: check threshold
          while(occ>=connectivity.connections_.size())
            connectivity.connections_.push_back(0);
          connectivity.connections_[occ]+=(1<<i);
        }
#endif
        continue;
      }

      float d=std::min(
          std::abs(levels_[i].data[getInd(x,y)].z_(0)/levels_[i].data[getInd(x,y)].model_(0,0)),
          std::abs(model.param.z_(0)/model.param.model_(0,0))
      );

      const float thr=(d*d+1.2f)*0.0035f;

      if( hops>0 && x>0&&y>0&&x+1<levels_[i].w&&y+1<levels_[i].h &&
          d!=0.f && ((found<1&&first_lvl) ||
              (
#ifdef USE_MIN_MAX_RECHECK_
                  (levels_[i].data[getInd(x,y)].v_max_-levels_[i].data[getInd(x,y)].v_min_)< 1.5f*(model.get_max_gradient(levels_[i].data[getInd(x,y)])*d*(1<<i)/kinect_params_.f+4*thr) /*std::min(0.5f,std::max(0.02f,0.05f*d))*/ //TODO: in anbhaengigkeit der steigung
                  //&& std::abs(model.model(levels_[i].data[getInd(x,y)].v_min_(0),levels_[i].data[getInd(x,y)].v_min_(1))-levels_[i].data[getInd(x,y)].v_min_(2))<thr
                  //&& std::abs(model.model(levels_[i].data[getInd(x,y)].v_max_(0),levels_[i].data[getInd(x,y)].v_max_(1))-levels_[i].data[getInd(x,y)].v_max_(2))<thr
                  &&
#endif
                  (checkModelAt(model, i,x,y,
                                thr)
                                /*||
                    checkModelAtZ(model, i,x,y,
                                  0.01f)*/)
                                  //d*(0.04-0.005*levels_[i].data[getInd(x,y)].model_(0,0)/model.param.model_(0,0))))
                                  //0.015)) {
                                  //d*(0.01-0.005*levels_[i].data[getInd(x,y)].model_(0,0)/model.param.model_(0,0)))
              )
#ifdef USE_NORMAL_CHECK
              && ( (i!=1&&i!=2) || std::abs(SubStructure::Model(levels_[i].data[getInd(x,y)]).getLinearNormal().dot(
                  model.getNormal(levels_[i].data[getInd(x,y)].model_(1,0)/levels_[i].data[getInd(x,y)].model_(0,0),
                                  levels_[i].data[getInd(x,y)].model_(3,0)/levels_[i].data[getInd(x,y)].model_(0,0)) ))>0.8 ) //TODO: optimize normalize out
#endif
          )
      ) {
        ++found;
        levels_[i].data[getInd(x,y)].occopied=mark;
        model+=levels_[i].data[getInd(x,y)];
        if(hops>10 || found%20==0)
          model.get();
        bNew=bNew2;

        list.add(SubStructure::SVALUE(getInd(x+1,y),hops-1));
        list.add(SubStructure::SVALUE(getInd(x-1,y),hops-1));
        list.add(SubStructure::SVALUE(getInd(x,y+1),hops-1));
        list.add(SubStructure::SVALUE(getInd(x,y-1),hops-1));

      }
      else
        bNew2=true;
    }
    last_size=list.pos;
    list.pos=-1;
    //break;
  } while(bNew);

  if(first_lvl && found<MIN_LOD) {
    for(int j=0; j<list.size; j++) {
      if(levels_[i].data[list.vals[j].v].occopied==mark)
        levels_[i].data[list.vals[j].v].occopied=-1;
    }
    return;
  }

  if(i==FINAL_LOD) {
    if(!list.size)
    {
      ROS_ASSERT(0);
      return;
    }

    S_POLYGON poly;
    model.isLinearAndTo();
    poly=model;
#ifdef CHECK_CONNECTIVITY
    poly.connectivity_=connectivity;
#endif

    static std::vector<SubStructure::SXY> outs;
    SubStructure::SXY pt;

    outs.clear();

    for(int j=0; j<list.size; j++) {
      x= list.vals[j].v%levels_[i].w;
      y= list.vals[j].v/levels_[i].w;

      if(x>0 && y>0 && x<levels_[i].w && y<levels_[i].h && isOccupied(i,x,y)==mark) continue;

      pt.x=x;pt.y=y;
#ifdef USE_MIN_MAX_RECHECK_
      pt.back = x>32 && y>32 && x<320-32 && y<240-32 && //TODO: improve this stupid thing (but it was easy :) )
          (levels_[i+2].data[getInd2(x/4,y/4)].v_min_-
              model.model(levels_[i].data[getInd(x,y)].model_(1)/levels_[i].data[getInd(x,y)].model_(0,0),
                          levels_[i].data[getInd(x,y)].model_(3)/levels_[i].data[getInd(x,y)].model_(0,0)))>=-0.05;
#endif
      outs.push_back(pt);
    }

    outline(ch_, levels_[i].w,levels_[i].h,outs,i, poly, model, mark);

    polygons_.push_back(poly);

    return;
  }

  SubStructure::VISITED_LIST<SubStructure::SVALUE> list_lower;
  for(int j=0; j<list.size; j++) {
    x= list.vals[j].v%levels_[i].w;
    y= list.vals[j].v/levels_[i].w;

    hops=8;

    if(filterOccupied(i,x,y,mark))
      continue;

    bool above = y>0 &&               filterOccupied(i,x,y-1,mark);
    bool below = y+1<levels_[i].h &&  filterOccupied(i,x,y+1,mark);
    bool left  = x>0 &&               filterOccupied(i,x-1,y,mark);
    bool right = x+1<levels_[i].w &&  filterOccupied(i,x+1,y,mark);

    if(above&&below&&left&&right)
      continue;

    if(above) {
      list_lower.add(SubStructure::SVALUE(getInd1(2*x,2*y),hops/2));
      list_lower.add(SubStructure::SVALUE(getInd1(2*x+1,2*y),hops/2));
    }
    if(below) {
      list_lower.add(SubStructure::SVALUE(getInd1(2*x,2*y+1),hops/2));
      list_lower.add(SubStructure::SVALUE(getInd1(2*x+1,2*y+1),hops/2));
    }
    if(left && !above) list_lower.add(SubStructure::SVALUE(getInd1(2*x,2*y),hops/2));
    if(left && !below) list_lower.add(SubStructure::SVALUE(getInd1(2*x,2*y+1),hops/2));
    if(right && !above) list_lower.add(SubStructure::SVALUE(getInd1(2*x+1,2*y),hops/2));
    if(right && !below) list_lower.add(SubStructure::SVALUE(getInd1(2*x+1,2*y+1),hops/2));
  }

  grow(list_lower, model, i-1, mark, false);
}

template <typename Point, typename PointLabel>
int Segmentation_QuadRegression<Point,PointLabel>::getPos(int *ch, const int xx, const int yy, const int w, const int h) {
  int p=0;
  const int i=0;
  for(int x=-1; x<=1; x++) {
    for(int y=-1; y<=1; y++) {
      if( xx+x>=0 && yy+y>=0 && xx+x<w && yy+y<h &&
          (x||y) && ch[getInd(xx+x,yy+y)]>0)
      {
        p |= (1<<Contour2D::SplineMap[ (y+1)*3 + x+1]);
      }
    }

  }
  return p;
}

template <typename Point, typename PointLabel>
void Segmentation_QuadRegression<Point,PointLabel>::outline(int *ch, const int w, const int h, std::vector<SubStructure::SXY> &out, const int i, S_POLYGON &poly, const SubStructure::Model &model, const int mark)
{
  SubStructure::SXYcmp ttt;
  std::sort(out.begin(),out.end(), ttt);

  for(size_t j=0; j<out.size(); j++) {
    ch[ getInd(out[j].x,out[j].y) ]=(int)j+1;
  }

  if(outline_check_size_<out.size()) {
    delete [] outline_check_;
    outline_check_ = new bool[out.size()];
    outline_check_size_=out.size();
  }
  memset(outline_check_,false,out.size());

  int n=-1;
  while(n+1<(int)out.size()) {
    ++n;
    if(outline_check_[n])
      continue;

    poly.segments_.push_back(std::vector<Eigen::Vector3f>());
#ifdef USE_BOOST_POLYGONS_
    poly.segments2d_.push_back(std::vector<BoostPoint>());
#endif

    int x=out[n].x;
    int y=out[n].y;
    int bf=8;
    int v=0;
    int back=0;
    int start_x=x, start_y=y;

    addPoint(i,x,y,mark, poly,model);
    int num=0,numb=0;

    while(1) {

      if(x<0 || y<0 || x>=w || y>=h || ch[ getInd(x,y) ]<1) {
        break;
      }

      back+=out[ch[ getInd(x,y) ]-1].back?1:0;
      ++numb;
      outline_check_[ch[ getInd(x,y) ]-1]=true;
      ch[ getInd(x,y) ]=-2;
      int p=getPos(ch,x,y,w,h);

      if(p==0|| (!Contour2D::g_Splines[bf][p].x&&!Contour2D::g_Splines[bf][p].y) )
      {
        break;
      }

      v+=v+Contour2D::g_Splines[bf][p].v;
      x+=Contour2D::g_Splines[bf][p].x;
      y+=Contour2D::g_Splines[bf][p].y;
      bf=Contour2D::g_Splines[bf][p].bf;
      ++num;

      if(std::abs(v)>5) {
        v=0;
        Eigen::Vector2f tv;
        tv(0)=x;tv(1)=y;
        addPoint(i,x,y,mark, poly,model,back/(float)numb);
        numb=back=0;
      }
    }
    poly.segments_.back()[0](2)=back/(float)numb;

    if(num<5 || (std::abs(x-start_x)+std::abs(y-start_y))>4 ) {
      poly.segments_.erase(poly.segments_.end()-1);
#ifdef USE_BOOST_POLYGONS_
      poly.segments2d_.erase(poly.segments2d_.end()-1);
#endif
    }


  }

  for(size_t j=0; j<out.size(); j++) {
    ch[ getInd(out[j].x,out[j].y) ]=0;
  }

}

template <typename Point, typename PointLabel>
bool Segmentation_QuadRegression<Point,PointLabel>::compute() {
  polygons_.clear();
  buildTree(*input_);
  calc();
  return true;
}

template <typename Point, typename PointLabel>
boost::shared_ptr<const pcl::PointCloud<PointLabel> > Segmentation_QuadRegression<Point,PointLabel>::compute_labeled_pc()
{
  typename pcl::PointCloud<PointLabel>::Ptr out(new pcl::PointCloud<PointLabel>);

  ROS_ASSERT(levels_.size()>0);

  out->resize(levels_[0].w*levels_[0].h);
  out->width = levels_[0].w;
  out->height= levels_[0].h;

  for(size_t x=0; x<levels_[0].w; x++)
  {
    for(size_t y=0; y<levels_[0].h; y++)
    {
      //position
      const int i=0;
      (*out)(x,y).x = levels_[0].data[getInd(x,y)].model_(0,1)/levels_[0].data[getInd(x,y)].model_(0,0);
      (*out)(x,y).y = levels_[0].data[getInd(x,y)].model_(0,3)/levels_[0].data[getInd(x,y)].model_(0,0);
      (*out)(x,y).z = levels_[0].data[getInd(x,y)].z_(0)/levels_[0].data[getInd(x,y)].model_(0,0);

      //color/label
      int mark = isOccupied(0,x,y);
      SetLabeledPoint<PointLabel>( (*out)(x,y), mark);
    }
  }

  return out;
}

template <typename Point, typename PointLabel>
boost::shared_ptr<const pcl::PointCloud<PointLabel> > Segmentation_QuadRegression<Point,PointLabel>::compute_reconstructed_pc()
{
  typename pcl::PointCloud<PointLabel>::Ptr out(new pcl::PointCloud<PointLabel>);

  ROS_ASSERT(levels_.size()>0);

  out->resize(levels_[0].w*levels_[0].h);
  out->width = levels_[0].w;
  out->height= levels_[0].h;

  for(size_t x=0; x<levels_[0].w; x++)
  {
    for(size_t y=0; y<levels_[0].h; y++)
    {
      //position
      const int i=0;
      (*out)(x,y).x = levels_[0].data[getInd(x,y)].model_(0,1)/levels_[0].data[getInd(x,y)].model_(0,0);
      (*out)(x,y).y = levels_[0].data[getInd(x,y)].model_(0,3)/levels_[0].data[getInd(x,y)].model_(0,0);

      //color/label
      int mark = isOccupied(0,x,y);
      if(mark>=0)
      (*out)(x,y).z = polygons_[mark].model_.model(
            levels_[0].data[getInd(x,y)].model_(0,1)/levels_[0].data[getInd(x,y)].model_(0,0),
            levels_[0].data[getInd(x,y)].model_(0,3)/levels_[0].data[getInd(x,y)].model_(0,0)
        );
else
      (*out)(x,y).z = std::numeric_limits<float>::quiet_NaN( );
      SetLabeledPoint<PointLabel>( (*out)(x,y), mark);
    }
  }

  return out;
}

class RunningStat
{
public:
  RunningStat() : m_n(0),m_newS(0) {}

  void Clear()
  {
    m_n = 0;
  }

  void Push(double x)
  {
    m_n++;

    // See Knuth TAOCP vol 2, 3rd edition, page 232
    if (m_n == 1)
    {
      m_oldM = m_newM = x;
      m_oldS = 0.0;
    }
    else
    {
      m_newM = m_oldM + (x - m_oldM)/m_n;
      m_newS = m_oldS + (x - m_oldM)*(x - m_newM);

      // set up for next iteration
      m_oldM = m_newM;
      m_oldS = m_newS;
    }
  }

  int NumDataValues() const
  {
    return m_n;
  }

  double Mean() const
  {
    return (m_n > 0) ? m_newM : 0.0;
  }

  double Variance() const
  {
    return ( (m_n > 1) ? m_newS/(m_n - 1) : 0.0 );
  }

  double StandardDeviation() const
  {
    return sqrt( Variance() );
  }

private:
  int m_n;
  double m_oldM, m_newM, m_oldS, m_newS;
};


template <typename Point, typename PointLabel>
void Segmentation_QuadRegression<Point,PointLabel>::compute_accuracy(float &mean, float &var, float &mean_abs, float &var_abs, size_t &used, size_t &mem, size_t &points)
{
  typename pcl::PointCloud<PointLabel>::Ptr out(new pcl::PointCloud<PointLabel>);

  ROS_ASSERT(levels_.size()>0);

  out->resize(levels_[0].w*levels_[0].h);
  out->width = levels_[0].w;
  out->height= levels_[0].h;

  RunningStat rstat, rstat_abs;
  points = 0;

  for(size_t x=0; x<levels_[0].w; x++)
  {
    for(size_t y=0; y<levels_[0].h; y++)
    {
      //position
      const int i=0;

      //color/label
      int mark = isOccupied(0,x,y);

      if(mark>0 && mark<(int)polygons_.size())
      {
        const float z_model = polygons_[mark].model_.model(
            levels_[0].data[getInd(x,y)].model_(0,1)/levels_[0].data[getInd(x,y)].model_(0,0),
            levels_[0].data[getInd(x,y)].model_(0,3)/levels_[0].data[getInd(x,y)].model_(0,0)
        );
        const float z = levels_[0].data[getInd(x,y)].z_(0)/levels_[0].data[getInd(x,y)].model_(0,0);

        if(pcl_isfinite(z - z_model))
        {
          rstat.Push(z - z_model);
          rstat_abs.Push(std::abs(z - z_model));
        }
      }

      if(levels_[0].data[getInd(x,y)].z_(0)/levels_[0].data[getInd(x,y)].model_(0,0)>0 && pcl_isfinite(levels_[0].data[getInd(x,y)].z_(0)/levels_[0].data[getInd(x,y)].model_(0,0)))
        points++;

    }
  }

  //points = levels_[0].w*levels_[0].h;
  used = rstat.NumDataValues();
  mem = 0;
  for(size_t i=0; i<polygons_.size(); i++)
    mem+=4*6 + polygons_[i].segments_.size()*2*4;

  mean = rstat.Mean();
  var = rstat.Variance();
  mean_abs = rstat_abs.Mean();
  var_abs = rstat_abs.Variance();
}
