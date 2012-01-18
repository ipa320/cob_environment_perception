// std:
#include <math.h>
// Boost:
#include <boost/program_options.hpp>
// PCL:
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "cob_3d_mapping_tools/io.h"
#include "cob_3d_mapping_common/point_types.h"

using namespace std;
using namespace pcl;

vector<string> file_o(3,"");
string file_i = "";
int r_ = 2;
int r2_ = 1;
float k_ = 1.0;
float max_ = 20;
int low_th_, high_th_;
int n_bins_ = 4;

void readOptions(int argc, char* argv[])
{
  using namespace boost::program_options;
  options_description options("Options");
  options.add_options()
    ("help", "produce help message")
    ("in", value<string>(&file_i), "input pcd file")
    ("out", value< vector<string> >(&file_o), "output files, first pcd, [second ppm]")
    ("radius,r",value<int>(&r_), "radius")
    ("dom_radius,R",value<int>(&r2_), "radius")
    ("smooth,k",value<float>(&k_), "smoothing")
    ("low,l",value<int>(&low_th_), "lower to zero")
    ("high,h",value<int>(&high_th_), "high to zero")
    ("bins,b",value<int>(&n_bins_), "number of bins")
    ("max,M",value<float>(&max_),"max change")
    ;

  positional_options_description p_opt;
  p_opt.add("in", 1).add("out", 3);
  variables_map vm;
  store(command_line_parser(argc, argv).options(options).positional(p_opt).run(), vm);
  notify(vm);

  if (vm.count("help"))
  {
    cout << options << endl;
    exit(0);
  }
  if (file_i == "") 
  {
    cout << "no input and output file defined " << endl << options << endl;
    exit(0);
  }
  if (file_o[0] == "") 
  {
    cout << "no output file defined " << endl << options << endl;
    exit(0);
  }
}

float getDepthVariance(const PointCloud<PointLabel>::Ptr &cloud, int idx_x, int idx_y)
{
  float var = 0.0;
  int tmp_curr, valid = 0;
  int centroid = cloud->at(idx_x,idx_y).label;
  for (int x = idx_x - r_; x <= idx_x + r_; x++)
  {
    for (int y = idx_y - r_; y <= idx_y + r_; y++)
    {
      tmp_curr = abs(cloud->at(x,y).label - centroid);
      if (tmp_curr > high_th_) continue;
      var += tmp_curr * tmp_curr;
      valid++;
    }
  }
  return (var / (float)valid);
}

int getMaxDepthChange(const PointCloud<PointLabel>::Ptr &cloud, int idx_x, int idx_y)
{
  int tmp_curr, d_max = 0, d_min = 0;
  int centroid = cloud->at(idx_x,idx_y).label;
  for (int x = idx_x - r_; x <= idx_x + r_; x++)
  {
    for (int y = idx_y - r_; y <= idx_y + r_; y++)
    {
      tmp_curr = cloud->at(x,y).label - centroid;
      if (abs(tmp_curr) > high_th_) continue;
      d_max = max(d_max,tmp_curr);
      d_min = min(d_min,tmp_curr);
    }
  }
  return (d_max - d_min);
}

int calcMean(const PointCloud<PointLabel>::Ptr &cloud, int idx)
{
  float mean = 0;
  int valid = 0;
  int idx_x = idx % cloud->width;
  int idx_y = idx / cloud->width;
  for (int x = idx_x - 1; x <= idx_x + 1; x++)
  {
    for (int y = idx_y - 1; y <= idx_y + 1; y++)
    {
      if (cloud->at(x,y).label == 0) continue;
      mean += cloud->at(x,y).label;
      valid++;
    }
  }
  return (round(mean/(float)valid));
}

int getSecondChange(const PointCloud<PointLabel>::Ptr &cloud, int idx_x, int idx_y)
{
  int tmp_curr, d_max = 0, d_min = 0;
  int centroid = cloud->at(idx_x,idx_y).label;
  for (int x = idx_x - 1; x <= idx_x + 1; x++)
  {
    for (int y = idx_y - 1; y <= idx_y + 1; y++)
    {
      tmp_curr = cloud->at(x,y).label - centroid;
      if (abs(tmp_curr) > high_th_) continue;
      d_max = max(d_max,tmp_curr);
      d_min = min(d_min,tmp_curr);
    }
  }
  if ( (d_max - d_min) < 2 )
    return 0;
  else
    return (d_max - d_min);
}

float getDepthChange(const PointCloud<PointLabel>::Ptr &cloud, int idx_x, int idx_y)
{
  float change_sum = 0.0;
  int curr, d;
  int ref = cloud->at(idx_x,idx_y).label;
  int valid = 0;
  for (int x = idx_x - r_; x <= idx_x + r_; x++)
  {
    for (int y = idx_y - r_; y <= idx_y + r_; y++)
    {
      curr = cloud->at(x,y).label;
      if (pcl_isnan((float)curr)) continue;
      d = fabs(ref - curr) - low_th_;
      if (d > high_th_) continue;
      if (d < 0)
	d = 0;
      change_sum += d + k_;
      valid++;
    }
  }
  return (change_sum / (float)(valid + k_ * valid));
}

float getDepthChangeCross(const PointCloud<PointLabel>::Ptr &cloud, int idx_x, int idx_y)
{
  float change_sum = 0.0;
  int curr, d;
  int ref = cloud->at(idx_x,idx_y).label;
  int valid = 0;
  for (int x = idx_x - r_; x <= idx_x + r_; x++)
  {
    if (x == idx_x) continue;
    curr = cloud->at(x,idx_y).label;
    if (pcl_isnan((float)curr)) continue;
    d = fabs(ref - curr) - low_th_;
    if (d > high_th_) continue;
    if (d < 0)
      d = 0;
    change_sum += d + k_;
    valid++;
  }
  for (int y = idx_y - r_; y <= idx_y + r_; y++)
  {
    curr = cloud->at(idx_x,y).label;
    if (y == idx_y) continue;
    if (pcl_isnan((float)curr)) continue;
    d = fabs(ref - curr) - low_th_;
    if (d > high_th_) continue;
    if (d < 0)
      d = 0;
    change_sum += d + k_;
    valid++;
  }
  return (change_sum / (float)(valid + k_ * valid));
}

float getDominantValue(const PointCloud<InterestPoint>::Ptr &cloud, int idx_x, int idx_y)
{
  vector<int> bins(n_bins_,0); // 0 to high_th_
  int bin;
  for (int x = idx_x - r2_; x <= idx_x + r2_; x++)
  {
    for (int y = idx_y - r2_; y <= idx_y + r2_; y++)
    {
      if (pcl_isnan(cloud->at(x,y).strength)) continue;
      if (cloud->at(x,y).strength > high_th_)
	bin = n_bins_ - 1;
      else if (cloud->at(x,y).strength < 0)
	bin = 0;
      else
	bin = floor(cloud->at(x,y).strength / (float)high_th_ * (float)n_bins_);
      //cout << cloud->at(x,y).strength << " | ";
      bins[bin]++;
    }
  }
  int b_max = 0;
  int idx_max;
  for (size_t i = 0; i < bins.size(); i++)
  {
    if (b_max < bins[i])
    {
      b_max = bins[i];
      idx_max = i;
    }
  }
  //cout << idx_max << " " << b_max << endl;
  return (((float)idx_max + 0.5)/(float)n_bins_ * (float)high_th_);
}

int main(int argc, char** argv)
{
  readOptions(argc, argv);
  PointCloud<PointXYZRGB>::Ptr p_org(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr p_lin(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr p_dom(new PointCloud<PointXYZRGB>);
  PointCloud<InterestPoint>::Ptr ip(new PointCloud<InterestPoint>);
  PointCloud<PointLabel>::Ptr l(new PointCloud<PointLabel>);
  PointCloud<PointLabel>::Ptr l2(new PointCloud<PointLabel>);
  PointCloud<PointLabel>::Ptr l3(new PointCloud<PointLabel>);

  PCDReader r;
  if (r.read(file_i, *p_org) == -1) return(0);

  copyPointCloud<PointXYZRGB, PointXYZRGB>(*p_org, *p_lin);
  p_lin->height = p_org->height;
  p_lin->width = p_org->width;
  copyPointCloud<PointXYZRGB, PointXYZRGB>(*p_org, *p_dom);
  p_dom->height = p_org->height;
  p_dom->width = p_org->width;
  ip->height = p_org->height;
  ip->width = p_org->width;
  ip->points.resize(ip->width * ip->height);
  l->height = p_org->height;
  l->width = p_org->width;
  l->points.resize(l->width * l->height);
  l2->height = p_org->height;
  l2->width = p_org->width;
  l2->points.resize(l2->width * l2->height);
  l3->height = p_org->height;
  l3->width = p_org->width;
  l3->points.resize(l3->width * l3->height);

  float max_d = FLT_MIN;
  float min_d = FLT_MAX;
  int d;

  cout << "init" << endl;
  for (size_t i = 0; i<p_lin->size(); i++)
  {
    //p_lin->points[i].z = sqrt(abs(p_org->points[i].z));
    l->points[i].label = round(1090.0f - ( 345.0f / p_org->points[i].z ));
    p_lin->points[i].z = 10.90f - ( 3.45f / p_org->points[i].z );
    if (l->points[i].label < 0)
    {
      l->points[i].label = 0;
      p_lin->points[i].z = 0;
    }
    p_lin->points[i].r = 0;
    p_lin->points[i].g = 255;
    p_lin->points[i].b = 0;
  }

  cout << "depth change" << endl;
  for (int x = r_+1; x < p_lin->width - r_; x++)
  {
    for (int y = r_+1; y < p_lin->height - r_; y++)
    {
      if (pcl_isnan(p_lin->at(x,y).z))
	continue;
      //d = getDepthVariance(l, x, y);
      //d = getDepthChangeCross(l, x, y);
      d = getMaxDepthChange(l, x, y);
      max_d = max((float)d,max_d);
      min_d = min((float)d,min_d);
      l2->points[x + y * l2->width].label = d;
      p_lin->points[x + y * l2->width].g = 0;
    }
  }
  //max_d = max_;
  cout << "colorize | max:" << max_d << " min: " << min_d << endl;
  for (size_t i = 0; i<p_lin->size(); i++)
  {
    if (p_lin->points[i].g != 0)
      continue;
    //cout << "("<<i<<"): " << ip->points[i].strength <<endl;
    l->points[i].label = calcMean(l2,i);
    float rgb = ((float)l->points[i].label - min_d) / (max_d - min_d) * 255;
    //float rgb = (ip->points[i].strength - min_d) / (max_d - min_d) * 255;
    if (rgb > 255.0)
      rgb = 255.0;
    else if (rgb < 0)
      rgb = 0;

    p_lin->points[i].r = (int)rgb;
    p_lin->points[i].g = (int)rgb;
    p_lin->points[i].b = (int)rgb;
  }

  if (file_o[1] != "")
  {
    cob_3d_mapping_tools::PPMWriter ppmW;
    ppmW.writeRGB(file_o[1], *p_lin);
  }


  // second change calc
  cout << "depth change 2" << endl;
  max_d = FLT_MIN;
  min_d = FLT_MAX;
  for (int x = r_+1; x < p_lin->width - r_; x++)
  {
    for (int y = r_+1; y < p_lin->height - r_; y++)
    {
      d = getSecondChange(l, x, y);
      max_d = max((float)d,max_d);
      min_d = min((float)d,min_d);
      l3->points[x + y * l3->width].label = d;
    }
  }

  for (size_t i = 0; i<p_lin->size(); i++)
  {
    //cout << "("<<i<<"): " << ip->points[i].strength <<endl;
    float rgb = ((float)l3->points[i].label - min_d) / (max_d - min_d) * 255;
    if (rgb > 255.0)
      rgb = 255.0;
    else if (rgb < 0)
      rgb = 0;

    p_lin->points[i].r = (int)rgb;
    p_lin->points[i].g = (int)rgb;
    p_lin->points[i].b = (int)rgb;
  }

  io::savePCDFileASCII(file_o[0], *p_lin);
  if (file_o[2] != "")
  {
    cob_3d_mapping_tools::PPMWriter ppmW;
    ppmW.writeRGB(file_o[2], *p_lin);
  }

  /* dominant
  for (int x = r2_+1; x < p_dom->width - r2_; x++)
  {
    for (int y = r2_+1; y < p_dom->height - r2_; y++)
    {
      p_dom->points[x + y * p_dom->width].z = getDominantValue(ip, x, y);
    }
  }

  for (size_t i = 0; i<p_dom->size(); i++)
  {
    if (pcl_isnan(p_dom->points[i].z))
    {
      p_dom->points[i].r = 0;
      p_dom->points[i].g = 100;
      p_dom->points[i].b = 0;
      continue;
    }

    float rgb = p_dom->points[i].z / (float)high_th_ * 255;
    if (rgb > 255.0)
      rgb = 255.0;
    else if (rgb < 0)
      rgb = 0;

    p_dom->points[i].r = (int)rgb;
    p_dom->points[i].g = (int)rgb;
    p_dom->points[i].b = (int)rgb;
  }

  if (file_o[2] != "")
  {
    cob_3d_mapping_tools::PPMWriter ppmW;
    ppmW.writeRGB(file_o[2], *p_dom);
  }
  */
}
