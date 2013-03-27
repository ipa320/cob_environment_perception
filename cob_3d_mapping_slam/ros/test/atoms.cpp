/*
 * atoms.cpp
 *
 *  Created on: 17.05.2012
 *      Author: josh
 */

#define DEBUG_
#define ATOM_TESTING_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <cob_3d_mapping_slam/dof/rotation.h>
#include <cob_3d_mapping_slam/dof/tflink.h>
#include <cob_3d_mapping_slam/dof/euler.h>
#include <cob_3d_mapping_slam/dof/dof_variance.h>
#include <cob_3d_mapping_slam/rotation_from_correspondences.h>
#include <pcl/common/transformation_from_correspondences.h>

#include "gnuplot_i.hpp"
#include <gtest/gtest.h>

#if 0
#define TEST_EXPRESSION(a) EXPECT_EQ((a), meval::EvaluateMathExpression(#a))

TEST(MathExpressions, operatorRecognition){
  EXPECT_TRUE(meval::ContainsOperators("+"));
  EXPECT_TRUE(meval::ContainsOperators("-"));
  EXPECT_TRUE(meval::ContainsOperators("/"));
  EXPECT_TRUE(meval::ContainsOperators("*"));
  EXPECT_FALSE(meval::ContainsOperators("1234567890qwertyuiop[]asdfghjkl;'zxcvbnm,._=?8"));
}

TEST(MathExpressions, basicOperations){
  EXPECT_EQ(5, meval::EvaluateMathExpression("2+3"));
  EXPECT_EQ(5, meval::EvaluateMathExpression("2 + 3"));
  EXPECT_EQ(10, meval::EvaluateMathExpression("20/2"));
  EXPECT_EQ(-4, meval::EvaluateMathExpression("6 - 10"));
  EXPECT_EQ(24, meval::EvaluateMathExpression("6 * 4"));
}

TEST(MathExpressions, complexOperations){
  TEST_EXPRESSION(((3 + 4) / 2.0) + 10);
  TEST_EXPRESSION(7 * (1 + 2 + 3 - 2 + 3.4) / 12.7);
  TEST_EXPRESSION((1 + 2 + 3) - (8.0 / 10));
}

TEST(MathExpressions, UnaryMinus){
  TEST_EXPRESSION(-5);
}

TEST(MathExpressions, badInput){
  //TODO - figure out what good error behavior is and test for it properly
  //EXPECT_EQ(0, meval::EvaluateMathExpression("4.1.3 - 4.1"));
  //EXPECT_EQ(0, meval::EvaluateMathExpression("4.1.3"));
}

TEST(MathUtils, basicOperations){
  EXPECT_EQ(math_utils::clamp<int>(-10, 10, 20), 10);
  EXPECT_EQ(math_utils::clamp<int>(15, 10, 20), 15);
  EXPECT_EQ(math_utils::clamp<int>(25, 10, 20), 20);
}
#endif

#define CYCLES 1000
#define THR_ANGLE 0.05
#define NOISE_MAX 0.3f
#define NOISE_STEP (NOISE_MAX/20)
#define NUM_SAMPLES 1000
//helper functions

template<typename Matrix>
float MATRIX_DISTANCE(const Matrix &a, const Matrix &b, const float thr=THR_ANGLE) {
  Matrix c=a-b;
  float d=c.norm();
  if(d>thr) {
    c=a-b.transpose();
    d=std::min(d,c.norm());
  }
  if(d>thr) {
    std::cout<<"A\n"<<a<<"\n";
    std::cout<<"B\n"<<b<<"\n";
  }
  EXPECT_NEAR(d,0,thr);
  return d;
}
#if 0
Eigen::Matrix3f build_random_rotation(DOF6::Rotationf &rot, const int N=2, const float noise=0.f) {
  Eigen::Vector3f n, nn, v,n2;
  float a = M_PI*(rand()%1000)/1000.f; //angle

  v(0)=0.01f;v(2)=v(1)=1.f;     //"some" plane

  nn(0) = (rand()%1000)/1000.f;
  nn(1) = (rand()%1000)/1000.f;
  nn(2) = (rand()%1000)/1000.f;
  nn.normalize();

  Eigen::AngleAxisf aa(a,nn);

  std::cout<<"rot angle\n"<<a<<"\n";
  std::cout<<"rot axis\n"<<nn<<"\n";

  std::vector<Eigen::Vector3f> normal;
  std::vector<Eigen::Vector3f> normal2;
  for(int j=0; j<std::max(2,N); j++) {
    //seconds
    n(0) = (rand()%1000)/1000.f-0.5f;        //init normals
    n(1) = (rand()%1000)/1000.f-0.5f;
    n(2) = (rand()%1000)/1000.f-0.5f;
    n.normalize();

    n2 = aa.toRotationMatrix()*n;
    n2(0) += 2*noise*((rand()%1000)/1000.f-0.5f);        //add noise
    n2(1) += 2*noise*((rand()%1000)/1000.f-0.5f);
    n2(2) += 2*noise*((rand()%1000)/1000.f-0.5f);
    n2.normalize();

    normal.push_back(n);
    normal2.push_back(n2);
  }

  for(size_t i=0; i<normal.size(); i++) {
    rot.add1(normal[i],normal2[i],v);
  }

  rot.finish1();

  pcl::RotationFromCorrespondences rfc;
  pcl::TransformationFromCorrespondences tfc;
  for(size_t i=0; i<normal.size(); i++) {
    rot.add2(normal[i],normal2[i],1);
    // rfc.add(normal[i],normal2[i]);
    tfc.add(normal[i],normal2[i]);
  }

  static float dis1=0,dis2=0,dis3=0;
  float d1=0,d2=0,d3=0;
  d1=MATRIX_DISTANCE(rfc.getTransformation(),aa.toRotationMatrix());
  d2=MATRIX_DISTANCE(rot.toRotationMatrix(),aa.toRotationMatrix());
  Eigen::Matrix3f M=tfc.getTransformation().matrix().topLeftCorner<3,3>();
  d3=MATRIX_DISTANCE(M,aa.toRotationMatrix());
  dis1+=d1;
  dis2+=d2;
  dis3+=d3;
  std::cout<<"rfc: "<<d1<<"\n";
  std::cout<<"rot: "<<d2<<"\n";
  std::cout<<"tfc: "<<d3<<"\n";
  std::cout<<"dis1: "<<dis1<<"\n";
  std::cout<<"dis2: "<<dis2<<"\n";
  std::cout<<"dis3: "<<dis3<<"\n";

  return aa.toRotationMatrix();
}


TEST(DOF6, rotation_basic){
  time_t ti = time(NULL);
  ROS_INFO("init rotation_basic with %d",(int)ti);
  srand(ti);

  for(int i=0; i<CYCLES; i++) {
    DOF6::Rotationf rot;
    Eigen::Matrix3f tf = build_random_rotation(rot);

    //check
    MATRIX_DISTANCE(rot.toRotationMatrix(),tf);

  }
}

TEST(DOF6, rotation_noise){
  time_t ti = time(NULL);
  ROS_INFO("init rotation_noise with %d",(int)ti);
  srand(ti);

  std::vector<double> x, y, dy;
  Gnuplot plot("rotation_noise");
  plot.savetops("doc/rotation_noise");
  plot.set_title("rotation_noise");

  for(float n=0.f; n<NOISE_MAX; n+=NOISE_STEP) {
    float dis=0.f;
    for(int i=0; i<CYCLES; i++) {
      DOF6::Rotationf rot;
      Eigen::Matrix3f tf = build_random_rotation(rot, NUM_SAMPLES, n);

      //check
      //dis+=(MATRIX_DISTANCE(rot.toRotationMatrix(),tf,n+THR_ANGLE)>(n*sqrtf(3)+THR_ANGLE)?1.f:0.f)/CYCLES;
      dis+=MATRIX_DISTANCE(rot.toRotationMatrix(),tf,sqrtf(3)*n+THR_ANGLE);
    }

    x.push_back(n);
    y.push_back(dis);
    dy.push_back(1);
  }

  plot.set_style("boxes").plot_x(y,"deviaton").showonscreen();
}
#endif

Eigen::AngleAxisf createRandomAA()
{
  Eigen::Vector3f nn;
  float a = M_PI*(rand()%1000)/2000.f; //angle

  nn(0) = (rand()%1000)/1000.f;
  nn(1) = (rand()%1000)/1000.f;
  nn(2) = (rand()%1000)/1000.f;
  nn.normalize();

  //std::cout<<"rot angle\n"<<a<<"\n";
  //std::cout<<"rot axis\n"<<nn<<"\n";

  return Eigen::AngleAxisf(a,nn);
}

Eigen::Vector3f createRandomT()
{
  Eigen::Vector3f t;

  t(0) = (rand()%1000)/1000.f;
  t(1) = (rand()%1000)/1000.f;
  t(2) = (rand()%1000)/1000.f;
  t*=(rand()%1000)/1000.f;
  //t.fill(0);

  return t;
}

Eigen::Matrix4f build_random_tflink(DOF6::TFLinkvf &tflink, const int N=2, const float noise=0.f, const Eigen::AngleAxisf aa=createRandomAA(), const Eigen::Vector3f t=createRandomT()) {
  Eigen::Vector3f n, v,n2;

  v(0)=0.01f;v(2)=v(1)=1.f;     //"some" plane

  //std::cout<<"t\n"<<t<<"\n";

  std::vector<Eigen::Vector3f> normal;
  std::vector<Eigen::Vector3f> normal2;
  for(int j=0; j<std::max(2,N); j++) {
    //seconds
    n(0) = (rand()%1000)/1000.f-0.5f;        //init normals
    n(1) = (rand()%1000)/1000.f-0.5f;
    n(2) = (rand()%1000)/1000.f-0.5f;
    n.normalize();

    n2 = aa.toRotationMatrix()*n;
    n2(0) += 2*noise*((rand()%1000)/1000.f-0.5f);        //add noise
    n2(1) += 2*noise*((rand()%1000)/1000.f-0.5f);
    n2(2) += 2*noise*((rand()%1000)/1000.f-0.5f);
    n2.normalize();

    normal.push_back(n);
    normal2.push_back(n2);
  }

  pcl::TransformationFromCorrespondences tfc;
  for(size_t i=0; i<normal.size(); i++) {

    //std::cout<<"normal\n"<<normal2[i]<<"\n";
    //std::cout<<"t\n"<<(normal2[i].dot(t)*normal2[i])<<"\n";
    int s=rand()%3;
    float w = (rand()%100+1)/10.f;
    if((s==0 && normal2[i].dot(t)>0)) //plane
    {
      std::cout<<"PLANE\n";
      tflink(DOF6::TFLinkvf::TFLinkObj(normal[i],true,false,w),
             DOF6::TFLinkvf::TFLinkObj(normal2[i]*(1+(normal2[i].dot(t)))
                                       ,true,false,w));
    }
    else if((s==1 && normal2[i].dot(t)>0)) //plane
    {
      std::cout<<"NORMAL\n";
      tflink(DOF6::TFLinkvf::TFLinkObj(normal[i], true,true,w),
             DOF6::TFLinkvf::TFLinkObj(normal2[i],true,true,w));
    }
    else
    {
      std::cout<<"CUBE\n";
      tflink(DOF6::TFLinkvf::TFLinkObj(normal[i],false,false,w),
             DOF6::TFLinkvf::TFLinkObj(normal2[i]+t,false,false,w));
    }

    tfc.add(normal[i],normal2[i]+t); //always with t

  }

  tflink.finish();


  Eigen::Matrix4f r=Eigen::Matrix4f::Identity();
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++)
      r(i,j) = aa.toRotationMatrix()(i,j);
    r.col(3)(i)= t(i);
  }

  static float dis2=0,dis3=0;
  float d2=0,d3=0;

  d2=MATRIX_DISTANCE(tflink.getTransformation(),r,10000);
  d3=MATRIX_DISTANCE(tfc.getTransformation().matrix(),r,10000);

  dis2+=d2;
  dis3+=d3;

  std::cout<<tflink<<"\n";

  std::cout<<"tfl: "<<d2<<"\n";
  std::cout<<"tfc: "<<d3<<"\n";

  std::cout<<"dis2: "<<dis2<<"\n";
  std::cout<<"dis3: "<<dis3<<"\n";

  return r;
}

//TEST(DOF6, tflink_basic)
void t5()
{
  time_t ti = time(NULL);
  ROS_INFO("init tflink_basic with %d",(int)ti);
  srand(ti);

  for(int i=0; i<CYCLES; i++) {
    DOF6::TFLinkvf rot;
    Eigen::Matrix4f tf = build_random_tflink(rot,10);

    //check
    MATRIX_DISTANCE(rot.getTransformation(),tf);

  }
}

//TEST(DOF6, tflink_noise)
void t4()
{
  time_t ti = time(NULL);
  ROS_INFO("init tflink_ with %d",(int)ti);
  srand(ti);

  std::vector<double> x, y, dy;
  Gnuplot plot("rotation_noise");
  plot.savetops("doc/rotation_noise");
  plot.set_title("rotation_noise");

  for(float n=0.f; n<NOISE_MAX; n+=NOISE_STEP) {
    float dis=0.f;
    for(int i=0; i<CYCLES; i++) {
      DOF6::TFLinkvf rot;
      Eigen::Matrix4f tf = build_random_tflink(rot, NUM_SAMPLES, n);

      //check
      //dis+=(MATRIX_DISTANCE(rot.toRotationMatrix(),tf,n+THR_ANGLE)>(n*sqrtf(3)+THR_ANGLE)?1.f:0.f)/CYCLES;
      dis+=MATRIX_DISTANCE(rot.getTransformation(),tf,sqrtf(3)*n+THR_ANGLE);
    }

    x.push_back(n);
    y.push_back(dis);
    dy.push_back(1);
  }

  plot.set_style("boxes").plot_x(y,"deviaton").showonscreen();
}

//TEST(DOF6, tflink_adding)
void t3()
{
  time_t ti = time(NULL);
  ROS_INFO("init tflink_adding with %d",(int)ti);
  srand(ti);

//TODO:
}

//TEST(DOF6, euler)
void t2()
{
  time_t ti = time(NULL);
  ROS_INFO("init euler with %d",(int)ti);
  srand(ti);

  for(int i=0; i<10000; i++) {
    Eigen::Vector3f nn;
    float a = (rand()%1000)/500.f-1;
    nn(0) = (rand()%1000)/1000.f;
    nn(1) = (rand()%1000)/1000.f;
    nn(2) = (rand()%1000)/1000.f;
    nn.normalize();

    Eigen::AngleAxisf aa(a,nn);

    DOF6::EulerAnglesf e1 = aa.toRotationMatrix();

    MATRIX_DISTANCE((Eigen::Matrix3f)e1,aa.toRotationMatrix(),0.001);
  }
}

TEST(DOF6, Source)
//void t1()
{
  time_t ti = time(NULL);
  ROS_INFO("init Source with %d",(int)ti);
  srand(ti);

  for(int i=0; i<CYCLES; i++) {
    DOF6::TFLinkvf rot1, rot2;
    const Eigen::AngleAxisf aa=createRandomAA();
    const Eigen::Vector3f t=createRandomT();

    //tf1 should be tf2
    Eigen::Matrix4f tf1 = build_random_tflink(rot1,30,0.4,aa,t);
    Eigen::Matrix4f tf2 = build_random_tflink(rot2,30,0.2,aa,t);

    //check
    const float d1=MATRIX_DISTANCE(rot1.getTransformation(),tf1,0.4);
    const float d2=MATRIX_DISTANCE(rot2.getTransformation(),tf2,0.2);

    DOF6::DOF6_Source<DOF6::TFLinkvf,DOF6::TFLinkvf> abc(rot1.makeShared(), rot2.makeShared());


//    std::cout<<"rot\n"<<aa.toRotationMatrix()<<"\n";
//    std::cout<<"t\n"<<t<<"\n";
//
//    std::cout<<"rot\n"<<rot1.getRotation()<<"\n";
//    std::cout<<"t\n"<<rot1.getTranslation()<<"\n";
//
//    std::cout<<"rot\n"<<rot2.getRotation()<<"\n";
//    std::cout<<"t\n"<<rot2.getTranslation()<<"\n";
//
//    std::cout<<"rot\n"<<abc.getRotation().toRotMat()<<"\n";
//    std::cout<<"t\n"<<abc.getTranslation()<<"\n";
//
//
//    std::cout<<"getRotationVariance    "<<rot1.getRotationVariance()<<"\n";
//    std::cout<<"getTranslationVariance "<<rot1.getTranslationVariance()<<"\n";
//
//    std::cout<<"getRotationVariance    "<<rot2.getRotationVariance()<<"\n";
//    std::cout<<"getTranslationVariance "<<rot2.getTranslationVariance()<<"\n";
//
//    std::cout<<"getRotationVariance    "<<abc.getRotationVariance()<<"\n";
//    std::cout<<"getTranslationVariance "<<abc.getTranslationVariance()<<"\n";

    float d3=MATRIX_DISTANCE((Eigen::Matrix3f)abc.getRotation(),aa.toRotationMatrix(),0.2);
    EXPECT_NEAR((abc.getTranslation()-t).norm(),0,0.2);
    d3+=(abc.getTranslation()-t).norm();

    //EXPECT_LE(d3,std::max(d1,d2));
  }
}

//TEST(DOF6, Source_Combinations)
void abc1()
{
  time_t ti = time(NULL);
  ROS_INFO("init Source with %d",(int)ti);
  srand(ti);

  for(int i=0; i<CYCLES; i++) {
    DOF6::TFLinkvf rot1, rot2, res;
    Eigen::AngleAxisf aa=createRandomAA();
    Eigen::Vector3f t=createRandomT();
    Eigen::Matrix4f tf1 = build_random_tflink(rot1,30,0,aa,t);

    aa=createRandomAA();
    t=createRandomT();
    Eigen::Matrix4f tf2 = build_random_tflink(rot2,30,0,aa,t);

    res = rot1+rot2;

    std::cout<<"TF1\n"<<tf1<<"\n";
    std::cout<<"TF2\n"<<tf2<<"\n";
    std::cout<<"RES1\n"<<tf1*tf2<<"\n";

    std::cout<<"RES2\n"<<res.getRotation()<<"\n";
    std::cout<<"RES2\n"<<res.getTranslation()<<"\n";

    std::cout<<"T TF1\n"<<tf1.inverse()<<"\n";
    std::cout<<"T\n"<<rot1.transpose().getRotation()<<"\n";
    std::cout<<"T\n"<<rot1.transpose().getTranslation()<<"\n";
  }
}

int main(int argc, char **argv){

#if 0
  DOF6::TFLinkvf rot1,rot2;
  Eigen::Matrix4f tf1 = build_random_tflink(rot1,3);
  Eigen::Matrix4f tf2 = build_random_tflink(rot2,3);

  DOF6::DOF6_Source<DOF6::TFLinkvf,DOF6::TFLinkvf,float> abc(rot1.makeShared(), rot2.makeShared());
  abc.getRotation();
  abc.getTranslation();

  std::cout<<"tf1\n"<<tf1<<"\n";
  std::cout<<"tf2\n"<<tf2<<"\n";
  std::cout<<"tf1*2\n"<<tf1*tf2<<"\n";
  std::cout<<"tf2*1\n"<<tf2*tf1<<"\n";

  std::cout<<"tf1\n"<<rot1.getTransformation()<<"\n";
  std::cout<<"tf2\n"<<rot2.getTransformation()<<"\n";
  std::cout<<"tf1*2\n"<<(rot1+rot2).getTransformation()<<"\n";

  rot1.check();
  rot2.check();

  return 0;

  pcl::RotationFromCorrespondences rfc;
  Eigen::Vector3f n, nn, v,n2,n3,z,y,tv;
  float a = 0.1f;

  z.fill(0);y.fill(0);
  z(2)=1;y(1)=1;
  nn.fill(0);
  nn(0) = 1;
  Eigen::AngleAxisf aa(a,nn);

  nn.fill(100);

  n.fill(0);
  n(0) = 1;
  n2.fill(0);
  n2=n;
  n2(1) = 0.2;
  n3.fill(0);
  n3=n;
  n3(2) = 0.2;

  n2.normalize();
  n3.normalize();

  tv.fill(1);
  tv.normalize();

#if 0

#if 0
  rfc.add(n,aa.toRotationMatrix()*n+nn,
          1*n.cross(y),1*n.cross(z),
          1*(aa.toRotationMatrix()*n).cross(y),1*(aa.toRotationMatrix()*n).cross(z),
          1,1/sqrtf(3));
#else
  rfc.add(n,aa.toRotationMatrix()*n,
          0*n.cross(y),0*n.cross(z),
          0*(aa.toRotationMatrix()*n).cross(y),0*(aa.toRotationMatrix()*n).cross(z),
          1,0);
#endif

#if 1
  rfc.add(n2,aa.toRotationMatrix()*n2+nn,
          tv,tv,
          tv,tv,
          1,1);
#else
  rfc.add(n2,aa.toRotationMatrix()*n2+nn,
          1*n2.cross(y),1*n2.cross(z),
          1*(aa.toRotationMatrix()*n2).cross(y),1*(aa.toRotationMatrix()*n2).cross(z),
          1,1/sqrtf(3));
#endif

#else
  float f=1;
  Eigen::Vector3f cyl;
  cyl.fill(1);
  cyl(0)=1;
  Eigen::Matrix3f cylM;
  cylM.fill(0);
  cylM.diagonal() = cyl;
  rfc.add(n,aa.toRotationMatrix()*n,
          f*n.cross(y),f*n.cross(z),
          f*(aa.toRotationMatrix()*n).cross(y),f*(aa.toRotationMatrix()*n).cross(z),
          1,0);
  rfc.add(n2,aa.toRotationMatrix()*n2+nn,
          1*n2.cross(y),1*n2.cross(z),
          1*(aa.toRotationMatrix()*n2).cross(y),1*(aa.toRotationMatrix()*n2).cross(z),
          1,1);
#endif
  rfc.add(n3,aa.toRotationMatrix()*n3+nn,
          //tv,tv,
          //tv,tv,
          n3.cross(y),n3.cross(z),
          1*(aa.toRotationMatrix()*n3).cross(y),1*(aa.toRotationMatrix()*n3).cross(z),
          1,1);

  std::cout<<"comp matrix:\n"<<rfc.getTransformation()<<"\n\n";
  std::cout<<"real matrix:\n"<<aa.toRotationMatrix()<<"\n\n";

  return 0;

  //rfc.covariance_.normalize();
  rfc.covariance_ = (rfc.var_*rfc.covariance_.inverse().transpose()*rfc.covariance_);
  std::cout<<"comp matrix: "<<rfc.getTransformation()<<"\n\n";
  std::cout<<"real matrix: "<<aa.toRotationMatrix()*aa.toRotationMatrix()<<"\n\n";

  return 0;
#endif

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

//DOF6::TFLink<Eigen::Vector3f> abc;
