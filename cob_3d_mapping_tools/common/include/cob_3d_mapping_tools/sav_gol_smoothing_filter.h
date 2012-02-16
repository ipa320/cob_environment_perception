#ifndef SAV_GOL_SMOOTHING_FILTER_H_
#define SAV_GOL_SMOOTHING_FILTER_H_

#include <pcl/point_types.h>
#include <Eigen/LU>

template <typename PointInT, typename PointOutT>
class SavGolSmoothingFilter
{
  public:

    typedef pcl::PointCloud<PointInT> PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef pcl::PointCloud<PointOutT> PointCloudOut;

  public:
    SavGolSmoothingFilter () : high_th_(5.0)
    { };

    inline void
      setInputCloud(const PointCloudInConstPtr &cloud)
    {
      input_ = cloud;
    }

    inline void
      setFilterCoefficients(int size, int order)
    {
      int n_coef = 0; //2*order+1; // number of coefficients
      for (int j=0;j<=order;j++) for (int i=0;i<=(order-j);i++) n_coef++;
      double sum, fac;

      Eigen::MatrixXd A2, C, A = Eigen::MatrixXd::Zero(size*size,n_coef);
      std::vector<double> coef;
      size_ = size;
      r_ = (size-1)/2;

      int d = 0;
      for (int y = -(size-1)/2; y <= (size-1)/2; y++)
      {
	for (int x = -(size-1)/2; x <= (size-1)/2; x++)
	{
	  int a = 0;
	  for (int j = 0; j <= order; j++)
	  {
	    for (int i = 0; i <= (order - j); i++)
	    {
	      A(d,a) = pow(x,i)*pow(y,j);
	      a++;
	    }
	  }
	  d++;
	}
      }

      A2 = A.transpose() * A;
      C = A2.inverse() * A.transpose();

      int mm;
      for(mm=0;mm<size*size;mm++) 
      {
	//std::cout << C(0,mm) << std::endl;
	coef.push_back(C(0,mm));
      }
      coef_ = coef;
      return;
    }

    inline void
      getFilterCoefficients(std::vector<double> &coef)
    {
      coef = coef_;
    }

    inline void
      setDistanceThreshold(float threshold)
    {
      high_th_ = threshold;
    }

    bool
      smoothPoint(int index, float &z);

    void
      reconstruct(PointCloudOut &output, std::vector<size_t> &ignored_indices);

    void
      reconstruct2ndPass(std::vector<size_t> &indices, 
			 PointCloudOut &output);
      

  private:

    std::vector<double> coef_;
    int size_;
    int r_;
    PointCloudInConstPtr input_;
    float high_th_;
  
};

#endif // SAV_GOL_SMOOTHING_FILTER_H_
