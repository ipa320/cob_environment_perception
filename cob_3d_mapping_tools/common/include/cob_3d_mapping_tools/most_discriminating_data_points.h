#ifndef __MOST_DISCRIMINATING_DATA_POINTS_H__
#define __MOST_DISCRIMINATING_DATA_POINTS_H__

#include <vector>

struct SortableDataPoint
{
public:
  SortableDataPoint() : idx(-1), dist(0.0) { }
  
  SortableDataPoint(const int &index, const float &mean_distance) : idx(index), dist(mean_distance) 
    { }
  
  inline bool
  operator<( const SortableDataPoint& other) const { return (dist < other.dist); }

  inline bool
  operator>( const SortableDataPoint& other) const { return (dist > other.dist); }

  inline bool
  operator<=( const SortableDataPoint& other) const { return (dist <= other.dist); }

  inline bool
  operator>=( const SortableDataPoint& other) const { return (dist >= other.dist); }


  int idx;
  float dist;
  
};

class MostDiscriminatingDataPoints
{
  public:
    MostDiscriminatingDataPoints () : predefined_initial_centers_(false), k_(1) { }


  inline void
     setInputData(const std::vector<std::vector<float> > * const pdata)
   {
     pdata_ = pdata;
     m_ = (pdata->at(0)).size();
     n_ = pdata->size();
   }

   inline void
     setK(const int k)
   {
     k_ = k;
   }

   inline void
     setInitialMeans(std::vector<int> * const pindices)
   {
     init_indices_ = pindices;
     predefined_initial_centers_ = true;
     k_ = pindices->size();
   }

   inline void
     resetInitialMeans()
   {
     predefined_initial_centers_ = false;
   }

   void
     computeInitialMeans(std::vector<int> * const output_init_indices);

   void
     computeDataPoints(std::vector<std::vector<float> > * const k_means);

  protected:
   void
     computeKmeans();

   int
     eStep();
   
   void
     mStep();

   bool predefined_initial_centers_;
   size_t k_;
   size_t m_; // size of a data point
   size_t n_; // number of data points
   const std::vector<std::vector<float> > *pdata_; // n x m
   std::vector<std::vector<float> > *pmeans_; // k x m

   std::vector<int> *init_indices_;
   std::vector<int> assigned_classes_;
   std::vector<int> count_;
   
};

#endif // __MOST_DISCRIMINATING_DATA_POINTS_H__
