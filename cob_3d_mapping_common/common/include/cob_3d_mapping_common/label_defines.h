/*
 * label_defines.h
 *
 *  Created on: 19.07.2011
 *      Author: goa-sf
 */

#ifndef COB_3D_MAPPING_COMMON_LABEL_DEFINES_H_
#define COB_3D_MAPPING_COMMON_LABEL_DEFINES_H_


// --- define colors ---
#define LBL_PLANE    0x00CCFF
#define LBL_EDGE     0x7F0000
#define LBL_EDGE_CVX 0xFF9900
#define LBL_CYL      0x007F00
#define LBL_CYL_CVX  0x00FF99
#define LBL_SPH      0x00007F
#define LBL_SPH_CVX  0x9900FF
#define LBL_UNDEF    0x999999

// --- define SVM labels ---
#define SVM_PLANE    0
#define SVM_EDGE_CVX 1
#define SVM_SPH_CVX  2
#define SVM_CYL_CVX  3

#define SVM_EDGE_CAV 4
#define SVM_SPH_CAV  5
#define SVM_CYL_CAV  6

// --- define integer labels ---
#define I_PLANE 0
#define I_EDGE  1
#define I_SPH   2
#define I_CYL   3

#endif
