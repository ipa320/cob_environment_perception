#ifndef SAV_GOL_SMOOTHING_FILTER_HPP_
#define SAV_GOL_SMOOTHING_FILTER_HPP_


#include <cob_3d_mapping_tools/sav_gol_smoothing_filter.h>

template <typename PointInT, typename PointOutT> bool
SavGolSmoothingFilter<PointInT,PointOutT>::smoothPoint(int index, float &z)
{
  int idx_x = index % input_->width;
  int idx_y = index / input_->width;
  int i = 0, miss = 0;

  float z_i;
  double sum = 0.0;
  float limit = 0.4 * size_ * size_;

  for (int y = idx_y - r_; y <= idx_y + r_; y++)
  {
    for (int x = idx_x - r_; x <= idx_x + r_; x++)
    {
      z_i = input_->at(x,y).z;
      if (pcl_isnan((float)z_i) || std::fabs(z_i-input_->points[index].z) > high_th_) 
      {
	z_i = input_->points[index].z;
	miss++;
	if (miss > limit) return false;
      }
      
      sum += coef_[i] * z_i;
      i++;
    }
  }
  //std::cout << "old: " << z << " new: " << sum << std::endl;
  z = sum;
  return true;
}

template <typename PointInT, typename PointOutT> void
SavGolSmoothingFilter<PointInT,PointOutT>::reconstruct(PointCloudOut &output, std::vector<size_t> &ignored_indices)
{
  output = *input_;
  int idx = 0;
  
  for (int y = r_+1; y < input_->height - r_; y++)
  {
    for (int x = r_+1; x < input_->width - r_; x++)
    {
      idx = y*input_->width + x;
      if (pcl_isnan(input_->at(x,y).z))
	continue;
      if ( !smoothPoint(idx, output.points[(size_t)idx].z) )
      {
	ignored_indices.push_back((size_t)idx);
      }
    }
  }
  return;
}

template <typename PointInT, typename PointOutT> void
SavGolSmoothingFilter<PointInT,PointOutT>::reconstruct2ndPass(
  std::vector<size_t> &indices, PointCloudOut &output)
{
  std::vector<size_t>::iterator it;

  for (it = indices.begin(); it != indices.end(); it++)
  {
    smoothPoint(*it, output.points[*it].z);
  }
  return;
}

#endif // SAV_GOL_SMOOTHING_FILTER_HPP_
