/*
 * atoms.cpp
 *
 *  Created on: 17.05.2012
 *      Author: josh
 */

#define DEBUG_
#define ATOM_TESTING_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <cob_3d_mapping_slam/curved_polygons/form.h>
#include <visualization_msgs/Marker.h>

#include <cob_3d_mapping_slam/curved_polygons/debug_interface.h>
#include <cob_3d_mapping_slam/dof/tflink.h>
#include <cob_3d_mapping_slam/dof/dof_uncertainty.h>
#include <cob_3d_mapping_slam/dof/dof_variance.h>
#include <cob_3d_mapping_slam/curved_polygons/objctxt.h>
#include <cob_3d_mapping_slam/slam/dummy/robot.h>

#include "gnuplot_i.hpp"
#include <gtest/gtest.h>
#include <unsupported/Eigen/NonLinearOptimization>

//#define ENABLE_TEST_1
//#define ENABLE_TEST_2
//#define ENABLE_TEST_3
//#define ENABLE_TEST_4
//#define ENABLE_TEST_5
//#define ENABLE_TEST_6
//#define ENABLE_TEST_7
//#define ENABLE_TEST_8
//#define ENABLE_TEST_9
//#define ENABLE_TEST_10
//#define ENABLE_TEST_11
//#define ENABLE_TEST_12
//#define ENABLE_TEST_14
//#define ENABLE_TEST_15
//#define ENABLE_TEST_16
#define ENABLE_TEST_17


struct MyFunctor
{
  const float a,b,c;
  const float x0,y0,z0;

  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
  {
    // Implement y = x^2
    fvec(0) = std::pow((-z0+b*x(1)*x(1)+c*x(0)*x(1)+a*x(0)*x(0)),2)+std::pow(x(1)-y0,2)+std::pow(x(0)-x0,2);

    std::cout<<"fvec\n"<<fvec<<std::endl;

    return 0;
  }

  int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
  {
    //Jacobian
    fjac(0,0) = 2*(c*x(1)+2*a*x(0))*(-z0+b*x(1)*x(1)+c*x(0)*x(1)+a*x(0)*x(0))+2*(x(0)-x0);
    fjac(0,1) = 2*(2*b*x(1)+c*x(0))*(-z0+b*x(1)*x(1)+c*x(0)*x(1)+a*x(0)*x(0))+2*(x(1)-y0);

    std::cout<<"fjac\n"<<fjac<<std::endl;

    return 0;
  }

  int inputs() const { return 2; }
  int values() const { return 2; } // number of constraints
};

#ifdef ENABLE_TEST_1
TEST(Opti, LM)
#else
void t1()
#endif
{
  Eigen::VectorXf x(2);
  x(0) = 1;
  x(1) = 1;
  std::cout << "x: " << x << std::endl;

  MyFunctor functor={5,6,7, 1,1,18};
  Eigen::LevenbergMarquardt<MyFunctor, float> lm(functor);
  lm.minimize(x);

  std::cout << "x that minimizes the function:\n" << x << std::endl;
}

void ADD_POINT(Slam_CurvedPolygon::Outline &O, const float x, const float y, const float e)
{
  Eigen::Vector3f v;
  v(0) = x;
  v(1) = y;
  v(2) = e;
  O += v;
}

#ifdef ENABLE_TEST_2
TEST(Merging, Extension)
#else
void test2()
#endif
{
  Slam_CurvedPolygon::Outline A, B;

  //  ADD_POINT(A, 0,0,    0.2);
  //  ADD_POINT(A, 0,3.5,  0.3);
  //  ADD_POINT(A, -1,3.5, 0.8);
  //  ADD_POINT(A, -1,2.5, 0.9);
  //  ADD_POINT(A, -2,2.5, 0.9);
  //  ADD_POINT(A, -2,0,   0.9);

  ADD_POINT(A, 0,0.5,    0.7);
  ADD_POINT(A, 0,3,     1);
  ADD_POINT(A, -2.5,3,  0.1);
  ADD_POINT(A, -2.5,0.5, 0.2);

  ADD_POINT(B, 1,-0.5,    0.7);
  ADD_POINT(B, 1,2,     1);
  ADD_POINT(B, -1.5,2,  0.1);
  ADD_POINT(B, -1.5,-0.5, 0.2);

  A.reverse();
  B.reverse();

  (B+A).debug_svg((A+B).debug_svg(B.debug_svg(A.debug_svg("A.svg"),100,300,"red"),100,300,"green"),100,300,"blue");
  //  (A+B).debug_svg(B.debug_svg(A.debug_svg("A.svg"),100,300,"red"),100,300,"green");
  //  exit(0);
}

#ifdef ENABLE_TEST_15
TEST(Merging, Extension8)
#else
void test15()
#endif
{
  Slam_CurvedPolygon::Outline A, B;

  ADD_POINT(A, 0,0,    0.7);
  ADD_POINT(A, 0,3,     1);
  ADD_POINT(A, 3,3,  0.1);
  ADD_POINT(A, 3,0, 0.2);

  ADD_POINT(B, 1,1,    0.7);
  ADD_POINT(B, 1,2,     1);
  ADD_POINT(B, 2,3,  0.1);
  ADD_POINT(B, 2,1, 0.2);

  (B+A).debug_svg((A+B).debug_svg(B.debug_svg(A.debug_svg("A7.svg"),100,300,"red"),100,300,"green"),100,300,"blue");
  //  (A+B).debug_svg(B.debug_svg(A.debug_svg("A.svg"),100,300,"red"),100,300,"green");
  //  exit(0);
}


#ifdef ENABLE_TEST_16
TEST(Merging, Extension9)
#else
void test16()
#endif
{
  Slam_CurvedPolygon::Outline A, B;
  ADD_POINT(A, -1.045884,-0.014834,         0.000000);
  ADD_POINT(A, -1.034974,1.645458,  0.000000);
  ADD_POINT(A, 1.407966,1.740485,   102.129730);
  ADD_POINT(A, 1.567071,1.729963,   110.842232);
  ADD_POINT(A, 1.518147,1.239551,   90.983131);
  ADD_POINT(A, 1.042412,1.229564,   0.000000);
  ADD_POINT(A, 0.233320,1.125095,   0.000000);
  ADD_POINT(A, -0.072294,1.023921,  59.796158);
  ADD_POINT(A, -0.106954,0.939260,  110.842232);
  ADD_POINT(A, -0.092559,0.617795,  110.842232);
  ADD_POINT(A, -0.130699,-0.005967,         110.842232);
  ADD_POINT(A, -1.015610,-0.037512,         9.617739);


  ADD_POINT(B, -1.205974,1.611247,  71.553490);
  ADD_POINT(B, -1.170201,1.649200,  0.000000);
  ADD_POINT(B, -0.848359,1.660372,  0.000000);
  ADD_POINT(B, -0.845104,1.702806,  0.000000);
  ADD_POINT(B, -0.581432,1.711480,  453.172150);
  ADD_POINT(B, 0.151303,1.695081,   453.172150);
  ADD_POINT(B, 1.510661,1.742285,   436.925049);
  ADD_POINT(B, 1.628683,1.662013,   453.172150);
  ADD_POINT(B, 1.830167,1.641676,   453.172150);
  ADD_POINT(B, 1.811066,1.537026,   453.172150);
  ADD_POINT(B, 1.746713,-0.252298,  385.196350);
  ADD_POINT(B, 1.684610,-0.544174,  385.196350);
  ADD_POINT(B, 1.701388,-0.963158,  436.578918);
  ADD_POINT(B, 1.571333,-0.986580,  445.391266);
  ADD_POINT(B, 1.241471,-0.963770,  452.844971);
  ADD_POINT(B, 1.140533,0.326848,   452.844971);
  ADD_POINT(B, 0.475619,-1.115085,  349.422333);
  ADD_POINT(B, 0.249968,-1.121169,  453.172150);
  ADD_POINT(B, -0.250100,-0.763183,         453.172150);
  ADD_POINT(B, -0.570233,-0.695548,         453.172150);
  ADD_POINT(B, -0.806169,-0.323459,         0.000000);
  ADD_POINT(B, -0.447819,0.019148,  0.000000);
  ADD_POINT(B, -1.204315,1.611264,  71.553490);
  ADD_POINT(B, -1.203599,1.673734,  0.000000);
  ADD_POINT(B, -0.581432,1.711480,  187.892410);
  ADD_POINT(B, -0.264263,1.704379,  453.172150);
  ADD_POINT(B, 1.513072,1.753348,   436.925995);
  ADD_POINT(B, 1.630895,1.717116,   453.172150);
  ADD_POINT(B, 1.627996,1.060091,   453.172150);
  ADD_POINT(B, 1.554010,1.183118,   453.172150);
  ADD_POINT(B, 1.313412,1.211873,   275.035492);
  ADD_POINT(B, 0.897379,1.211863,   0.000000);
  ADD_POINT(B, 0.040116,1.106657,   0.000000);
  ADD_POINT(B, -0.251436,1.011473,  343.437988);
  ADD_POINT(B, -0.264279,0.681655,  343.437988);
  ADD_POINT(B, -0.247862,0.210322,  453.172150);
  ADD_POINT(B, 0.132636,0.574106,   0.000000);
  ADD_POINT(B, 0.354757,0.580037,   0.000000);
  ADD_POINT(B, 0.386799,0.662925,   0.000000);
  ADD_POINT(B, 0.355860,0.983544,   0.000000);
  ADD_POINT(B, 0.001064,1.029807,   0.000000);
  ADD_POINT(B, -0.253165,0.967086,  0.000000);
  ADD_POINT(B, -0.264279,0.681653,  343.437988);
  ADD_POINT(B, -0.241488,0.027288,  453.172150);
  ADD_POINT(B, -0.482831,-0.014327,         453.172150);
  ADD_POINT(B, -0.619539,-0.145028,         21.026535);
  ADD_POINT(B, -1.129928,-0.175288,         453.172150);
  ADD_POINT(B, -1.111384,-0.269988,         85.979233);
  ADD_POINT(B, -1.389662,-0.255861,         28.425617);
  ADD_POINT(B, -1.425714,0.098974,  208.680603);
  ADD_POINT(B, -1.500362,0.447526,  164.860352);
  ADD_POINT(B, -1.796075,0.471063,  0.000000);
  ADD_POINT(B, -2.274216,0.418171,  234.627274);
  ADD_POINT(B, -2.296404,0.136106,  0.000000);
  ADD_POINT(B, -2.388968,0.193383,  0.000000);
  ADD_POINT(B, -2.313087,1.600093,  71.553490);
  ADD_POINT(B, -0.851855,1.614811,  72.198341);
  ADD_POINT(B, -0.845104,1.702806,  0.000000);
  ADD_POINT(B, -0.581432,1.711480,  453.172150);
  ADD_POINT(B, 1.640513,1.661748,   453.172150);
  ADD_POINT(B, 1.830167,1.641676,   453.172150);
  ADD_POINT(B, 1.811066,1.537026,   453.172150);
  ADD_POINT(B, 1.746713,-0.252298,  385.196350);
  ADD_POINT(B, 1.684610,-0.544174,  385.196350);
  ADD_POINT(B, 1.701388,-0.963158,  436.578918);
  ADD_POINT(B, 1.571333,-0.986580,  445.391266);
  ADD_POINT(B, 1.241471,-0.963770,  452.844971);
  ADD_POINT(B, 1.140533,0.326848,   452.844971);
  ADD_POINT(B, 0.475619,-1.115085,  349.422333);
  ADD_POINT(B, 0.249968,-1.121169,  453.172150);
  ADD_POINT(B, -0.250100,-0.763183,         453.172150);
  ADD_POINT(B, -0.570233,-0.695548,         453.172150);
  ADD_POINT(B, -0.806169,-0.323459,         0.000000);
  ADD_POINT(B, -0.482831,-0.014326,         0.000000);

  (B+A).debug_svg((A+B).debug_svg(B.debug_svg(A.debug_svg("A16.svg"),100,300,"red"),100,300,"green"),100,300,"blue");
  //  (A+B).debug_svg(B.debug_svg(A.debug_svg("A.svg"),100,300,"red"),100,300,"green");
  //  exit(0);
}


#ifdef ENABLE_TEST_14
TEST(Merging, Extension7)
#else
void test14()
#endif
{
  Slam_CurvedPolygon::Outline A, B;

  ADD_POINT(A, 1.269705,-0.975754,  136.011032);
  ADD_POINT(A, 1.276548,-0.271520,  51.004135);
  ADD_POINT(A, 1.173826,0.516804,   0.000000);
  ADD_POINT(A, 0.407346,0.444026,   0.000000);
  ADD_POINT(A, 0.422948,0.493314,   0.000000);
  ADD_POINT(A, 0.330092,0.987637,   0.000000);
  ADD_POINT(A, 0.018048,1.042598,   0.000000);
  ADD_POINT(A, -0.350869,1.057517,  0.000000);
  ADD_POINT(A, -0.879164,1.038550,  0.000000);
  ADD_POINT(A, -1.280176,0.954838,  66.554298);
  ADD_POINT(A, -1.339894,0.894041,  136.011032);
  ADD_POINT(A, -1.334974,0.536400,  136.011032);
  ADD_POINT(A, -1.384047,0.112611,  136.011032);
  ADD_POINT(A, -1.483590,0.481948,  136.011032);
  ADD_POINT(A, -1.557691,0.496490,  136.011032);
  ADD_POINT(A, -2.297758,0.392300,  13.601104);
  ADD_POINT(A, -2.320095,0.108696,  0.000000);
  ADD_POINT(A, -2.389679,0.168287,  0.000000);
  ADD_POINT(A, -2.334253,1.576999,  14.316951);
  ADD_POINT(A, -1.967494,1.574737,  136.011032);
  ADD_POINT(A, 1.549718,1.597524,   136.011032);
  ADD_POINT(A, 1.699055,1.555135,   136.011032);
  ADD_POINT(A, 1.695281,0.150421,   136.011032);
  ADD_POINT(A, 1.678939,-0.882572,  136.011032);
  ADD_POINT(A, 1.574028,-0.974080,  136.011032);


  ADD_POINT(B, 1.275477,-0.981885,  0.000000);
  ADD_POINT(B, 1.278862,-0.292657,  50.942062);
  ADD_POINT(B, 1.175625,0.503335,   0.000000);
  ADD_POINT(B, 0.409199,0.422850,   45.281837);
  ADD_POINT(B, 0.331370,0.982494,   0.000000);
  ADD_POINT(B, 0.019325,1.037043,   0.000000);
  ADD_POINT(B, -0.349669,1.052238,  0.000000);
  ADD_POINT(B, -0.878034,1.033058,  0.000000);
  ADD_POINT(B, -1.279239,0.949065,  66.449097);
  ADD_POINT(B, -1.338103,0.888892,  135.845505);
  ADD_POINT(B, -1.333009,0.590709,  135.845505);
  ADD_POINT(B, -1.370899,0.058651,  135.845505);
  ADD_POINT(B, -1.482420,0.475614,  113.537460);
  ADD_POINT(B, -1.555821,0.482391,  135.845505);
  ADD_POINT(B, -2.295438,0.385319,  13.584551);
  ADD_POINT(B, -2.317035,0.101376,  0.000000);
  ADD_POINT(B, -2.386773,0.161003,  0.000000);
  ADD_POINT(B, -2.333375,1.571880,  14.299527);
  ADD_POINT(B, -2.051811,1.566876,  135.845505);
  ADD_POINT(B, 1.420109,1.589480,   135.845505);
  ADD_POINT(B, 1.700595,1.551211,   135.845505);
  ADD_POINT(B, 1.697783,0.128521,   135.845505);
  ADD_POINT(B, 1.680066,-0.868659,  135.845505);
  ADD_POINT(B, 1.610140,-0.976139,  135.845505);

  (B+A).debug_svg((A+B).debug_svg(B.debug_svg(A.debug_svg("A.svg"),100,300,"red"),100,300,"green"),100,300,"blue");
  //    (A+B).debug_svg(B.debug_svg(A.debug_svg("A.svg"),100,300,"red"),100,300,"green");
  //    exit(0);
}

#ifdef ENABLE_TEST_3
TEST(Merging, Extension2)
#else
void test3()
#endif
{
  float ws[4]={1,0, 0,1};

  for(int i=0; i<2; i++)
  {
    Slam_CurvedPolygon::Outline A, B;

    ADD_POINT(A, 0,0,    ws[i*2+0]);
    ADD_POINT(A, 0,3.5,  ws[i*2+0]);
    ADD_POINT(A, -1,3.5, ws[i*2+0]);
    ADD_POINT(A, -1,2.5, ws[i*2+0]);
    ADD_POINT(A, -2,2.5, ws[i*2+0]);
    ADD_POINT(A, -2,0,   ws[i*2+0]);

    ADD_POINT(B, 1,-0.5,    ws[i*2+1]);
    ADD_POINT(B, 1,2,       ws[i*2+1]);
    ADD_POINT(B, -1.5,2,    ws[i*2+1]);
    ADD_POINT(B, -1.5,-0.5, ws[i*2+1]);

    A.reverse();
    B.reverse();

    char buffer[128];

    sprintf(buffer,"%d.svg",i);
    (B+A).debug_svg((A+B).debug_svg(B.debug_svg(A.debug_svg(buffer),100,300,"red"),100,300,"green"),100,300,"blue");
    //(B+A).debug_svg(B.debug_svg(A.debug_svg(buffer),100,300,"red"),100,300,"blue");
  }
}

#ifdef ENABLE_TEST_4
TEST(Merging, Extension3)
#else
void test4()
#endif
{
  Slam_CurvedPolygon::Outline A, B;

  ADD_POINT(A, 0.362614,-0.386075,        0.000000);
  ADD_POINT(A, 0.301979,-0.022260,  1.000000);
  ADD_POINT(A, -0.121422,-0.058531,         1.000000);
  ADD_POINT(A, -0.083883,-0.430221,         0.000000);

  ADD_POINT(B, 0.375979,-0.382710,  0.000000);
  ADD_POINT(B, 0.285977,-0.017765,  1.000000);
  ADD_POINT(B, -0.121091,-0.055354,         1.000000);
  ADD_POINT(B, -0.092725,-0.439833,         0.000000);

  A.reverse();
  B.reverse();

  //  (B+A).debug_svg((A+B).debug_svg(B.debug_svg(A.debug_svg("A3.svg",400,1000),400,1000,"red"),400,1000,"green"),400,1000,"blue");
  (A+B).debug_svg(B.debug_svg(A.debug_svg("A3.svg",400,1000),400,1000,"red"),400,1000,"green");
  B.debug_svg(A.debug_svg("A3X.svg",400,1000),400,1000,"red");
}

#ifdef ENABLE_TEST_5
TEST(Merging, Extension4)
#else
void test5()
#endif
{
  Slam_CurvedPolygon::Outline A, B;

  //  ADD_POINT(A, 0.352464,-0.328968,  0.544554);
  //  ADD_POINT(A, 0.310498,-0.012788,  0.625000);
  //  ADD_POINT(A, -0.105240,-0.051654,         1.000000);
  //  ADD_POINT(A, -0.085381,-0.333977,         0.354167);
  //
  //  ADD_POINT(B, 0.381562,-0.382153,  0.000000);
  //  ADD_POINT(B, 0.319971,-0.017452,  0.400000);
  //  ADD_POINT(B, -0.117289,-0.067878,         1.000000);
  //  ADD_POINT(B, -0.064972,-0.428602,         0.000000);

  //  ADD_POINT(A, 1.271197,-1.002109,  36.201504);
  //  ADD_POINT(A, 1.276123,-0.289343,  73.547798);
  //  ADD_POINT(A, 1.173015,0.508309,   32.055977);
  //  ADD_POINT(A, 0.422219,0.430278,   32.019093);
  //  ADD_POINT(A, 0.332066,0.989471,   0.000000);
  //  ADD_POINT(A, 0.019068,1.044561,   0.000000);
  //  ADD_POINT(A, -0.350912,1.059492,  0.000000);
  //  ADD_POINT(A, -0.880648,1.040339,  46.949902);
  //  ADD_POINT(A, -1.281505,0.958299,  139.610519);
  //  ADD_POINT(A, -1.342242,0.895776,  192.148376);
  //  ADD_POINT(A, -1.334560,0.568031,  192.148376);
  //  ADD_POINT(A, -1.378715,0.072318,  192.148376);
  //  ADD_POINT(A, -1.482872,0.482487,  173.982361);
  //  ADD_POINT(A, -1.557451,0.488403,  189.394531);
  //  ADD_POINT(A, -2.298002,0.391501,  19.141132);
  //  ADD_POINT(A, -2.316800,0.111018,  0.000000);
  //  ADD_POINT(A, -2.392913,0.168025,  10.024461);
  //  ADD_POINT(A, -2.329704,1.573308,  106.172142);
  //  ADD_POINT(A, -2.019224,1.569707,  192.148376);
  //  ADD_POINT(A, 1.417241,1.592710,   192.148376);
  //  ADD_POINT(A, 1.697454,1.553752,   192.148376);
  //  ADD_POINT(A, 1.695868,-0.567176,  192.148376);
  //  ADD_POINT(A, 1.687407,-0.873390,  192.148376);
  //  ADD_POINT(A, 1.630390,-1.005029,  184.242416);
  //
  //  A.weight_ = 200;
  //
  //
  //  ADD_POINT(B, 1.232858,-1.038015,  0.000000);
  //  ADD_POINT(B, 1.234751,-0.645128,  93.168755);
  //  ADD_POINT(B, 1.120643,0.477694,   0.000000);
  //  ADD_POINT(B, 0.588452,0.474953,   0.000000);
  //  ADD_POINT(B, 0.592946,0.791144,   0.000000);
  //  ADD_POINT(B, 0.501822,0.960626,   0.000000);
  //  ADD_POINT(B, 0.166220,1.010063,   0.000000);
  //  ADD_POINT(B, -0.222853,1.017744,  0.000000);
  //  ADD_POINT(B, -0.720829,0.990421,  0.000000);
  //  ADD_POINT(B, -1.116511,0.900883,  69.494606);
  //  ADD_POINT(B, -1.174784,0.839676,  132.740341);
  //  ADD_POINT(B, -1.162942,0.602914,  132.740341);
  //  ADD_POINT(B, -1.215195,-0.140121,         132.740341);
  //  ADD_POINT(B, -1.466481,-0.087689,         44.246780);
  //  ADD_POINT(B, -1.536346,0.423917,  0.000000);
  //  ADD_POINT(B, -2.169648,0.432516,  42.863258);
  //  ADD_POINT(B, -2.131289,1.512646,  17.698713);
  //  ADD_POINT(B, 1.574970,1.562794,   132.740341);
  //  ADD_POINT(B, 1.627951,1.517644,   132.740341);
  //  ADD_POINT(B, 1.637892,-0.282478,  132.740341);
  //  ADD_POINT(B, 1.627902,-0.618023,  132.740341);
  //  ADD_POINT(B, 1.583944,-1.039931,  132.740341);
  //
  //  B.weight_ = 200;


  ADD_POINT(A, -0.307397,-0.237957,         11.327011);
  ADD_POINT(A, -0.294620,0.200909,  204.053207);
  ADD_POINT(A, 0.069812,0.199390,   58.021393);
  ADD_POINT(A, 0.058937,-0.400914,  267.556702);
  A.weight_ = 200;

  ADD_POINT(B, -0.304834,-0.242607,         0.000000);
  ADD_POINT(B, -0.201638,0.103096,  5.053251);
  ADD_POINT(B, 0.001059,0.104582,   117.428665);
  ADD_POINT(B, 0.048089,-0.513129,  134.123077);
  B.weight_ = 200;

  //    B.debug_svg(A.debug_svg("A4.svg"),100,300,"red");
  (B+A).debug_svg((A+B).debug_svg(B.debug_svg(A.debug_svg("A4.svg",400,1000),400,1000,"red"),400,1000,"green"),400,1000,"blue");
  //    (B+A).debug_svg(B.debug_svg(A.debug_svg("A4.svg",400,1000),400,1000,"red"),400,1000,"green");
  //      exit(0);
}

#ifdef ENABLE_TEST_6
TEST(Merging, MergeSame)
#else
void test6()
#endif
{
  Slam_CurvedPolygon::Outline A, B;

  ADD_POINT(A, 0,0,    0.2);
  ADD_POINT(A, 1,0,  1.);
  ADD_POINT(A, 0,1,  0.3);

  ADD_POINT(B, 0,0,    0.2);
  ADD_POINT(B, 1,0,  0.3);
  ADD_POINT(B, 0,1,  0.3);

  (B+A).debug_svg((A+B).debug_svg(B.debug_svg(A.debug_svg("msA.svg",400,300),400,300,"red"),400,300,"green"),400,300,"blue");
}

#ifdef ENABLE_TEST_7
TEST(Merging, MergeSimiliar)
#else
void test7()
#endif
{
  Slam_CurvedPolygon::Outline A, B;

  ADD_POINT(A, 0,0,    0.2);
  ADD_POINT(A, 1,0,  0.3);
  ADD_POINT(A, 0,1,  0.3);

  ADD_POINT(B, 0,0,    0.2);
  ADD_POINT(B, 0.5,-1,  0.3);
  ADD_POINT(B, 1,0,  0.3);
  ADD_POINT(B, 0,1,  0.3);

  (B+A).debug_svg((A+B).debug_svg(B.debug_svg(A.debug_svg("msiA.svg",400,300),400,300,"red"),400,300,"green"),400,300,"blue");
}

#ifdef ENABLE_TEST_8
TEST(Merging, MergeInner)
#else
void test8()
#endif
{
  Slam_CurvedPolygon::Outline A, B;

  ADD_POINT(A, 0,0,  0);
  ADD_POINT(A, 1,0,  0);
  ADD_POINT(A, 1,1,  0);
  ADD_POINT(A, 0,1,  0);

  ADD_POINT(B, 0,0,  0);
  ADD_POINT(B, 1,0,  0);
  ADD_POINT(B, 0.1,0.5,  0);
  ADD_POINT(B, 1,1,  0);
  ADD_POINT(B, 0,1,  0);

  //  A.reverse();
  //  B.reverse();

  (B+A).debug_svg((A+B).debug_svg(B.debug_svg(A.debug_svg("8A.svg",400,300),400,300,"red"),400,300,"green"),400,300,"blue");

  (A|B).debug_svg("8C3.svg");
  (B|A).debug_svg("8C4.svg");
}

#ifdef ENABLE_TEST_9
TEST(Merging, Reverse)
#else
void test9()
#endif
{
  Slam_CurvedPolygon::Outline A, B;

  ADD_POINT(A, 0,1,  0);
  ADD_POINT(A, 0,0,  0);
  ADD_POINT(A, 1,0,  1);

  A.debug_svg("9A1.svg");

  A.reverse(true);
  A.debug_svg("9A2.svg");

  A.reverse(true);
  A.debug_svg("9A3.svg");
}

#ifdef ENABLE_TEST_10
TEST(Merging, FOV)
#else
void test10()
#endif
{
  typedef DOF6::DOF6_Source<DOF6::TFLinkvf,DOF6::DOF6_Uncertainty<Dummy::RobotParameters,float> > DOF6;
  BoundingBox::FoVBB<float> A, B;
  Eigen::Vector3f v1, v2;

  v1(0) = -0.5;
  v1(1) = 0;
  v1(2) = 2;
  v2(0) = 0;
  v2(1) = 0.5;
  v2(2) = 2.5;
  A.update(v1,v2);

  v1(0) = -0.2;
  v1(1) = 0;
  v1(2) = 2;
  v2(0) = 0.7;
  v2(1) = 0.5;
  v2(2) = 2.5;
  B.update(v1,v2);

  Eigen::Matrix3f R1, R2;
  Eigen::Vector3f t1, t2;
  R1 = R2 = Eigen::Matrix3f::Identity();
  t1 = t2 = Eigen::Vector3f::Zero();
  std::cout<< ((A.transform(R1,t1) & B.transform(R2,t2))?"intersects":"not intersects") <<"\n";
}

#ifdef ENABLE_TEST_11
TEST(Surface, Compare)
#else
void test11()
#endif
{
  const int NUM=10;

  for(size_t l=0; l<10; l++) {

    boost::array<float, 6> params;
    for(int i=0; i<6; i++)
      params[i] = (rand()%10000-5000)/10000.f;

    //    params[2]=params[4]=params[5]=0;
    //    params[0]=1;
    //    params[2]=1;params[4]=params[5]=0;
    //    params[5]=0;

    Eigen::Vector2f p2s[10];
    Eigen::Vector3f p3s[10];

    for(int i=0; i<NUM; i++)
    {
      p2s[i](0) = (rand()%10000-5000)/1000.f;
      p2s[i](1) = (rand()%10000-5000)/1000.f;

      p2s[i](0) = (i%3)*0.5f*20-10;
      p2s[i](1) = ((i/3)%3)*0.5f*20-10;

      p3s[i](0) = (rand()%10000-5000)/1000.f;
      p3s[i](1) = (rand()%10000-5000)/1000.f;
      p3s[i](2) = (rand()%10000-5000)/1000.f;
    }

    //instances to test
    Slam_Surface::Surface *insts[] = {new Slam_Surface::PolynomialSurface, new Slam_Surface::SurfaceNurbs};
    Eigen::Vector3f r1[sizeof(insts)/sizeof(Slam_Surface::Surface *)*NUM];
    Eigen::Vector3f r2[sizeof(insts)/sizeof(Slam_Surface::Surface *)*NUM];
    Eigen::Vector2f r3[sizeof(insts)/sizeof(Slam_Surface::Surface *)*NUM];

    int p=0;

    for(int i=0; i<(int)(sizeof(insts)/sizeof(Slam_Surface::Surface *)); i++)
    {
      EXPECT_EQ(insts[i]!=0,true);
      ROS_INFO("testing --> %s <--", insts[i]->getName());
      EXPECT_EQ(insts[i]->getSurfaceType(),i+1);

      insts[i]->init(params,-10,10, -10,10,1);
      //insts[i]->init(params,0,1, 0,1);
      if(1){ // test transform
        Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
        Eigen::Vector3f t = Eigen::Vector3f::Identity()*3;
        Eigen::AngleAxisf aa(0.4,t);
        //R = aa.toRotationMatrix();
        insts[i]->transform(R,t);
      }

      for(int j=0; j<NUM; j++)
      {
        //p3s[j] = insts[0]->project2world(p2s[j]);

        r1[p] = insts[i]->project2world(p2s[j]);
        r2[p] = insts[i]->normalAt(p2s[j]);
        r3[p] = insts[i]->nextPoint(p3s[j]);

        Eigen::Vector2f pt = insts[i]->nextPoint(r1[p]);

        std::cout<<pt<<"\n\n";
        std::cout<<p2s[j]<<"\n";
        std::cout<<"-------\n";

        EXPECT_NEAR( (insts[i]->project2world(pt)-r1[p]).squaredNorm(), 0.f, 0.01f);

        ++p;
      }
    }

    //check instances against each other
    p=NUM;
    for(int i=1; i<(int)(sizeof(insts)/sizeof(Slam_Surface::Surface *)); i++)
    {

      for(int j=0; j<NUM; j++)
      {

#if 1
        std::cout<<p2s[p%NUM]<<"\n\n";
        std::cout<<r1[p]<<"\n\n";
        std::cout<<r1[p%NUM]<<"\n";
        std::cout<<"-------\n";

        std::cout<<r2[p]<<"\n\n";
        std::cout<<r2[p%NUM]<<"\n";
        std::cout<<"-------\n";

        std::cout<<r3[p]<<"\n\n";
        std::cout<<r3[p%NUM]<<"\n";

        std::cout<< (insts[i]->project2world(r3[p])-p3s[j]).norm() <<"  "<< (insts[0]->project2world(r3[p%NUM])-p3s[j]).norm()<<"\n";
        std::cout<<"-------\n";
#else
        EXPECT_NEAR( (r1[p]-r1[p%NUM]).squaredNorm(), 0.f, 0.01f);
        EXPECT_NEAR( (r2[p]-r2[p%NUM]).squaredNorm(), 0.f, 0.01f);
        EXPECT_NEAR( (r3[p]-r3[p%NUM]).squaredNorm(), 0.f, 0.01f);
#endif

        ++p;
      }

    }

  }
}



#ifdef ENABLE_TEST_12
TEST(Surface, Compare2)
#else
void test12()
#endif
{
  boost::array<float, 6> params;
  for(int i=0; i<6; i++)
    params[i] = (rand()%10000-5000)/10000.f;

  Slam_Surface::SurfaceNurbs inst;

  inst.init(params,-10,10, -10,10, 1);

  Eigen::Vector3f p;
  p(0) = 3.;
  p(1) = 5.;
  p(2) = 0.;
  Eigen::Vector2f p2;
  p2(0) = 0.4;
  p2(1) = 0.7;
  //p=inst._project2world(p2);
  inst.nextPoint(p);
}

template<typename T>
void addBB(visualization_msgs::Marker &marker, const T&bb, const ::std_msgs::ColorRGBA &col, const Eigen::Matrix3f &tmp_rot, const Eigen::Vector3f &tmp_tr) {
  geometry_msgs::Point line_p;
  Eigen::Vector3f edges[8];

  bb.get8Edges(edges);
  for(int i=0; i<8; i++)
    edges[i] = tmp_rot*edges[i]+tmp_tr;

  for(int i=0; i<4; i++)
  {
    line_p.x = edges[2*i](0);
    line_p.y = edges[2*i](1);
    line_p.z = edges[2*i](2);
    marker.points.push_back(line_p);
    marker.colors.push_back(col);
    line_p.x = edges[2*i+1](0);
    line_p.y = edges[2*i+1](1);
    line_p.z = edges[2*i+1](2);
    marker.points.push_back(line_p);
    marker.colors.push_back(col);
  }

  int conv[]={0,1,4,5};
  for(int j=0; j<4; j++)
  {
    int i=conv[j];
    line_p.x = edges[i](0);
    line_p.y = edges[i](1);
    line_p.z = edges[i](2);
    marker.points.push_back(line_p);
    marker.colors.push_back(col);
    line_p.x = edges[i+2](0);
    line_p.y = edges[i+2](1);
    line_p.z = edges[i+2](2);
    marker.points.push_back(line_p);
    marker.colors.push_back(col);
  }

  for(int i=0; i<4; i++)
  {
    line_p.x = edges[i](0);
    line_p.y = edges[i](1);
    line_p.z = edges[i](2);
    marker.points.push_back(line_p);
    marker.colors.push_back(col);
    line_p.x = edges[i+4](0);
    line_p.y = edges[i+4](1);
    line_p.z = edges[i+4](2);
    marker.points.push_back(line_p);
    marker.colors.push_back(col);
  }
}

#ifdef ENABLE_TEST_17
TEST(BB, OOBB)
#else
void test17()
#endif
{
  {
    PlNurbsSurfaceSPf nurbs_;
    Matrix_Point3Df Pts(3,3);
    for(int i=0;i<3;++i){
      for(int j=0;j<3;++j){
        Pts(i,j) = PlPoint3Df((i-1)*(j+1), j*2,
                              0
        ) ;
      }
    }
    nurbs_.globalInterp(Pts,2,2);

    Eigen::Vector3f map[100][100];

    for(int i=0; i<1000; i++)  {

      for(int j=0; j<100; j++) {
        for(int k=0; k<100; k++) {
          PlPoint3Df p = nurbs_.pointAt(j/99.f,k/99.f);
          map[j][k](0) = p.x();
          map[j][k](1) = p.y();
          map[j][k](2) = p.z();
        }
      }

      //10 planes
      for(int m=0; m<10; m++) {
        Eigen::Vector3f n, p3;
        n(0) = (rand()%100)/100.f;
        n(1) = (rand()%100)/100.f;
        n(2) = (rand()%100)/100.f;
        p3(0) = (rand()%100)/100.f;
        p3(1) = (rand()%100)/100.f;
        p3(2) = (rand()%100)/100.f;
        n.normalize();

        int num=0;
        for(int j=0; j<99; j++) {
          for(int k=0; k<99; k++) {
            float f;
            f=n.dot(p3-map[j][k])/n.dot(map[j+1][k]-map[j][k]);
            if(f>=0&&f<1) num++;
            f=n.dot(p3-map[j][k])/n.dot(map[j][k+1]-map[j][k]);
            if(f>=0&&f<1) num++;
          }
          }

        std::cout<<num<<"\n";
      }
    }
  //nurbs_.writeVRML97("torus.wrl");
}


  ros::Time::init();

  rosbag::Bag bag_out;
  bag_out.open("merging17.bag", rosbag::bagmode::Write);
  ::std_msgs::ColorRGBA col;
  col.g = col.r = col.b=0;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/openni_rgb_optical_frame";
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = marker.color.g = marker.color.b =  marker.color.a = 1;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
  marker.id = 5;



  //no intersection
  for(int i=0; i<100; i++) {
    if( !(i&1) && !(i&2) && !(i&4) ) continue;

    pcl::PointCloud<pcl::PointXYZ> pc1, pc2;
    pcl::PointXYZ p;
    for(int j=0; j<100; j++) {
      p.x=(rand()%10000-5000)/10000.f;
      p.y=(rand()%10000-5000)/10000.f;
      p.z=(rand()%10000-5000)/10000.f;
      pc1.push_back(p);
    }
    for(int j=0; j<100; j++) {
      p.x=(rand()%10000-5000)/10000.f+(i&1?2.3f:0);
      p.y=(rand()%10000-5000)/10000.f+(i&2?2.3f:0);
      p.z=(rand()%10000-5000)/10000.f+(i&4?2.3f:0);
      pc2.push_back(p);
    }
    BoundingBox::OOBB bb1,bb2;
    bb1.set(pc1);
    bb2.set(pc2);

    bb1.debug();
    bb2.debug();

    bool in=bb1&bb2;
    if(in) {
      addBB(marker,bb1,col,Eigen::Matrix3f::Identity(),Eigen::Vector3f::Zero());
      col.r=1;
      addBB(marker,bb2,col,Eigen::Matrix3f::Identity(),Eigen::Vector3f::Zero());

      bag_out.write("/markers", ros::Time::now(), marker);
      bag_out.close();

      std::cout<<std::endl;

      exit(0);
    }
    EXPECT_FALSE(in);
  }

  //intersection
  for(int i=0; i<100; i++) {
    pcl::PointCloud<pcl::PointXYZ> pc1, pc2;
    pcl::PointXYZ p;
    for(int j=0; j<100; j++) {
      p.x=(rand()%10000-5000)/10000.f;
      p.y=(rand()%10000-5000)/10000.f;
      p.z=(rand()%10000-5000)/10000.f;
      pc1.push_back(p);
    }
    for(int j=0; j<100; j++) {
      p.x=(rand()%10000-5000)/10000.f+(i&1?.5f:0);
      p.y=(rand()%10000-5000)/10000.f+(i&2?.5f:0);
      p.z=(rand()%10000-5000)/10000.f+(i&4?.5f:0);
      pc2.push_back(p);
    }
    BoundingBox::OOBB bb1,bb2;
    bb1.set(pc1);
    bb2.set(pc2);

    EXPECT_TRUE(bb1&bb2);
  }

  {
    pcl::PointCloud<pcl::PointXYZ> pc1, pc2;
    pcl::PointXYZ p;
    for(int i=0; i<10; i++)
    for(int j=0; j<10; j++) {
      p.x=i;
      p.y=j;
      p.z=0;
      pc1.push_back(p);
    }
    for(int i=0; i<10; i++)
    for(int j=0; j<10; j++) {
      p.x=i;
      p.y=j/2+3;
      p.z=i-3;
      pc2.push_back(p);
    }
    BoundingBox::OOBB bb1,bb2;
    bb1.set(pc1);
    bb2.set(pc2);

    EXPECT_TRUE(bb1&bb2);
  }

}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

