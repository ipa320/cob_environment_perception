/*
 * accuracy_evaluator.h
 *
 *  Created on: 19.07.2011
 *      Author: goa-sf
 */

#ifndef COB_3D_MAPPING_FEATURES_FEATURE_EVALUATION_H_
#define COB_3D_MAPPING_FEATURES_FEATURE_EVALUATION_H_

#include <iostream>
#include <string>

#define RES_RSD  0
#define RES_PC   1
#define RES_FPFH 2

#define NUM_LBL 4

namespace cob_3d_mapping_features
{
  class EvalResults
  {
  private:
    char alg_type_;
    int ref_ [NUM_LBL]; // num of reference points per label
    int fp_ [NUM_LBL];  // false positive, error
    int fn_ [NUM_LBL];  // false negative, error
    int tp_ [NUM_LBL];  // true positive, correct
    int tn_ [NUM_LBL];  // true negative, correct
    int tp_not_edge_not_plane_; // points identified correctly as not edge and not plane
    int undef_;
    int all_; // all reference points

  public:
    EvalResults(char alg_type) : alg_type_(alg_type), undef_(0), 
      t_normal_(0.0), t_feature_(0.0), t_classifier_(0.0), points_(0)
    {
      // initialize arrays:
      for (size_t i = 0; i < NUM_LBL; ++i)
      {
	ref_[i] = 0;
	fp_[i] = 0;
	fn_[i] = 0;
	tp_[i] = -1;
	tn_[i] = -1;
      }
      tp_not_edge_not_plane_ = 0;
    }

    void incRef(const int &label) { ++ref_[label]; }
    void incFP(const int &label) { ++fp_[label]; }
    void incFN(const int &label) { ++fn_[label]; }
    void incTPnEnP() { ++tp_not_edge_not_plane_; }
    void incUndef() { ++undef_; }

    int getRef(const int &label) {return (ref_[label]);}
    int getFP(const int &label) {return (fp_[label]);}
    int getFN(const int &label) {return (fn_[label]);}
    int getUndef() {return (undef_);}
    int getTP(const int &label) {return (tp_[label]);}
    int getTN(const int &label) {return (tn_[label]);}
    int getAll() {return (all_);}
    int getTPnEnP() {return tp_not_edge_not_plane_;}

    void updateValues();

    float calcTruePosAcc(const int &label);
    float calcTrueNegAcc(const int &label);
    float calcFalseNegAcc(const int &label);
    float calcFalsePosAcc(const int &label);
    float calcProportion(const int &label);
    float calcAllAcc();
    float calcEdgePlaneAcc();

    void printAllToConsole(const std::string &prefix="");
    void printToConsole(const int &label, const std::string &prefix="");

    void writeTimerLog(const std::string &file);

    std::string getLabel();

    int points_;
    double t_normal_;
    double t_feature_;
    double t_classifier_;
  };
}

bool vis_enable_;
float pass_depth_;

bool rsd_enable_;
bool rsd_mls_enable_;      // enable surface smoothing
bool rsd_vox_enable_;      // enable voxel grid filtering
float rsd_vox_;            // voxel size
float rsd_normal_r_;       // normal search radius
float rsd_r_max_;          // r_max_ limit
float rsd_r_;              // search radius
float rsd_r_min_th_upper_; // min value to determine planes
float rsd_r_min_th_cylinder_lower_; // max value to determine edges
float rsd_r_min_th_sphere_lower_;
float rsd_max_min_ratio_;    // threshold between sphere and cylinder (r_max = x * r_min)
//float rsd_r_max_offset_;

bool pc_enable_;
bool pc_mls_enable_;       // enable surface smoothing
bool pc_vox_enable_;       // enable voxel grid filtering
float pc_vox_;             // voxel size
float pc_normal_r_;        // normal search radius
float pc_r_;               // search radius
float pc_c_max_th_cylinder_upper_; // min value to determine edges
float pc_c_max_th_sphere_upper_; // min value to determine edges
float pc_c_max_th_lower_; // max value to determine planes
float pc_max_min_ratio_;   // threshold between sphere and cylinder (r_max = x * r_min)
//float pc_c_min_offset_;

bool fpfh_enable_;
bool fpfh_mls_enable_;       // enable surface smoothing
bool fpfh_vox_enable_;       // enable voxel grid filtering
float fpfh_vox_;             // voxel size
float fpfh_normal_r_;        // normal search radius
float fpfh_r_;               // search radius
int fpfh_knn_k_;

#endif
