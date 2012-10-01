/*
 * registration_fastslam.h
 *
 *  Created on: Nov 17, 2011
 *      Author: goa-jh
 */

#ifndef REGISTRATION_FASTSLAM_H_
#define REGISTRATION_FASTSLAM_H_

#include <cob_vision_slam/3DEnvReconstruction/EnvReconstructionControlFlow.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace cob_3d_registration {

template <typename Point>
class Registration_FastSLAM : public GeneralRegistration<Point>
{
  BFL::AnalyticMeasurementModelGaussianUncertainty *m_MeasModel;
  BFL::SystemModel<BFL::ColumnVector> *m_SysModel;
  FastSLAM m_FastSLAM;
  BFL::ColumnVector m_RobotPose;

  MatrixWrapper::Matrix transformation_tof2base_pltf_;
  MatrixWrapper::Matrix transformation_camera2base_;

  boost::shared_ptr<AbstractFeatureVector> m_AllFeatures;
  KdTreeDataAssociation m_DataAssociation;
  AbstractLocalFeatureDetector* m_FeatureDetector;
  boost::shared_ptr<FastSLAMParticle> m_MapParticle;
  int step_;

  pcl::PointCloud<Point> register_;
  pcl::PointCloud<Point>  visual_pc_;

public:

  Registration_FastSLAM()
  :m_FastSLAM(1000, 1/*numPoints*/),
   m_RobotPose(3),
   m_DataAssociation(1, 0.02),
   transformation_tof2base_pltf_(4,4),
   transformation_camera2base_(4,4), step_(1)
  {
    m_FeatureDetector = new SURFDetector();
    ((SURFDetector*)m_FeatureDetector)->Init(100); //500

    CreateMeasurementModel();
    CreateSystemModel();
  }

  virtual ~Registration_FastSLAM() {
    delete m_FeatureDetector;
  }

  virtual boost::shared_ptr<const pcl::PointCloud<Point> > getMarkers() {return visual_pc_.makeShared();}

protected:

  virtual bool compute_features(){
    ROS_INFO("compute_features");
    m_AllFeatures.reset(new AbstractFeatureVector());

    ipa_SensorFusion::ColoredPointCloud cpc(this->input_image_->size());

    cpc.SetColorImage(*this->input_image_);

    m_FeatureDetector->DetectFeatures(cpc, *m_AllFeatures);

    if(this->register_.size()<1)
      AddFeaturesToGlobalMap(&*m_AllFeatures);

    visual_pc_.clear();

    //std::cout << "afl size:" << afl->size() << std::endl;
    //TODO: use colorImage0 but transform u and v to sharedImageSize
    //m_FeatureDetector->DetectFeatures(*colorImage0, afl);
    AbstractFeatureVector::iterator It;
    for (It=m_AllFeatures->begin(); It!=m_AllFeatures->end(); It++)
    {
      int x=(*It)->Get<double>(M_U);
      int y=(*It)->Get<double>(M_V);

      (*It)->Get<double>(M_X) = this->input_org_->points[ x+y*this->input_org_->width ].x;
      (*It)->Get<double>(M_Y) = this->input_org_->points[ x+y*this->input_org_->width ].z;
      (*It)->Get<double>(M_Z) = this->input_org_->points[ x+y*this->input_org_->width ].y;

      unsigned char R,G,B;
      cv::Mat3b m=*this->input_image_;


      visual_pc_.points.push_back(this->input_org_->points[ x+y*this->input_org_->width ]);

      R=m(y,x)[0];
      G=m(y,x)[1];
      B=m(y,x)[2];

      R=m(x,y)[0];
      G=m(x,y)[1];
      B=m(x,y)[2];

      (*It)->Get<double>(M_RED) = (double) R;
      (*It)->Get<double>(M_GREEN) = (double) G;
      (*It)->Get<double>(M_BLUE) = (double) B;
      (*It)->Get<int>(M_FEATUREID)=-1;
    }
    ((SURFDetector*)m_FeatureDetector)->FilterFeatures(&*m_AllFeatures, 3);

    return m_AllFeatures->size()>3;
  }

  virtual bool compute_corrospondences() {
    ROS_INFO("compute_corrospondences");
    BFL::ColumnVector input(3);
    input(1) = m_RobotPose(1);
    input(2) = m_RobotPose(2);
    input(3) = m_RobotPose(3);
    m_FastSLAM.UpdatePosition(*m_SysModel, input);
    int associationCtr = 0;
    unsigned int ctrAssociations = 0;
    m_DataAssociation.AssociateData(&*m_AllFeatures);

    ColumnVector theta(3);
    AbstractFeatureVector::iterator It;
    for (It=m_AllFeatures->begin(); It!=m_AllFeatures->end(); It++)
    {
      //Transformation from camera to robot coordinate system
      //TransformCameraToPltfBase(*It);
      Eigen::Vector4f v;
      v(0)=(*It)->Get<double>(M_X);
      v(1)=(*It)->Get<double>(M_Y);
      v(2)=(*It)->Get<double>(M_Z);
      v(3)=0;

      v = this->transformation_ * v;

      (*It)->Get<double>(M_X) = v(0);
      (*It)->Get<double>(M_Y) = v(1);
      (*It)->Get<double>(M_Z) = v(2);

      if((*It)->Get<int>(M_FEATUREID) != -1)
      {
        associationCtr++;
        //m_DataAssociation.AddFeature(*It);
        m_FastSLAM.UpdateKnownFeature(*m_MeasModel, *m_SysModel, *It, ctrAssociations);

      }
      /*else
                    {
                            //m_DataAssociation.AddFeature(*It);
                            m_FastSLAM.UpdateUnknownFeature(*m_MeasModel, theta, (*It)->Get<int>(M_FEATUREID), height);
                    }*/
    }

    for(int i=0; i<m_FastSLAM.m_Particles.size(); i++) {
      std::cout<<m_FastSLAM.m_Particles[i].WeightGet()<<" "<<(*m_FastSLAM.m_Particles[i].ValueGet()->GetPose())<<std::endl;
    }

    m_MapParticle = m_FastSLAM.Resample(ctrAssociations);
    //m_MapParticle = m_FastSLAM.GetBestParticle();

    std::cout << "ctrAssociations: " << ctrAssociations << std::endl;
    std::cout << m_AllFeatures->size() << " features extracted" << std::endl;
    std::cout << associationCtr << " associated by descriptor" << std::endl;
    std::cout << m_MapParticle->m_RejectedFeatures.size() << " rejected in particle" << std::endl;
    std::cout << associationCtr - m_MapParticle->m_RejectedFeatures.size() << " remaining associations" << std::endl;
    //std::cout << "map size: " << m_MapParticle->GetMap()->size() << std::endl;
    std::cout << "Original robot pose: " << m_RobotPose << std::endl;

    /// Remove class associations from rejected particles
    for (unsigned int i=0; i<m_MapParticle->m_RejectedFeatures.size(); i++)
    {
      /// Remove class label if necessary
      m_MapParticle->m_RejectedFeatures[i]->Get<int>(M_FEATUREID) = -1;
    }

    unsigned int i = 0;
    std::vector<int> mask(m_AllFeatures->size(),0);
    for (It=m_AllFeatures->begin(), i=0; It!=m_AllFeatures->end(); It++, i++)
    {
      //TransformCameraToPltfBase(*It);

      /// Add new feature only if it is not masked
      /// It is important that the features ID corresponds
      /// to the position in the particle map
      if((*It)->Get<int>(M_FEATUREID) == -1)
      {
        if ( mask[i] == 0) m_FastSLAM.AddUnknownFeature(*m_MeasModel, *It, step_);
      }
      //              multimap<int,boost::shared_ptr<AbstractEKF> >* map = m_MapParticle->GetMap();
      //              multimap<int,boost::shared_ptr<AbstractEKF> >::iterator bla;
      /*(*It)->Get<double>(M_X) = m_FastSLAM.m_Particles[0].ValueGet()->GetMap()->find((*It)->Get<int>(M_FEATUREID))->second->PostGet()->ExpectedValueGet()(1);
                    (*It)->Get<double>(M_Y) = m_FastSLAM.m_Particles[0].ValueGet()->GetMap()->find((*It)->Get<int>(M_FEATUREID))->second->PostGet()->ExpectedValueGet()(2);
                    (*It)->Get<double>(M_Z) = ((EKF2DLandmark*)(m_FastSLAM.m_Particles[0].ValueGet()->GetMap()->find((*It)->Get<int>(M_FEATUREID))->second.get()))->m_Height;*/

    }
    //RenderCorrespondences2D(*afl);
    m_DataAssociation.AddNewFeatures(&*m_AllFeatures, mask);
    step_++;
    return true;
  }

  virtual bool compute_transformation() {
    ROS_INFO("compute_transformation");
    BFL::ColumnVector* position = m_MapParticle->GetPose();

    Eigen::Matrix2f rot=Eigen::Matrix2f::Identity();

    rot(0,0) =  cosf((*position)(3));
    rot(0,1) = -sinf((*position)(3));
    rot(1,0) =  sinf((*position)(3));
    rot(1,1) =  cosf((*position)(3));

    //if(this->register_.size()>0)
    {
      m_RobotPose = *position;
      this->transformation_.row(0)(0) = rot.col(0)(0);
      this->transformation_.row(0)(1) = rot.col(0)(1);
      this->transformation_.row(1)(0) = rot.col(1)(0);
      this->transformation_.row(1)(1) = rot.col(1)(1);
      this->transformation_.col(3)(0) = (*position)(1);
      this->transformation_.col(3)(1) = (*position)(2);
    }

    ROS_INFO("phi %f", (*position)(3));
    std::cout << this->transformation_<<"\n";
static bool first=true;
    if(!first)
    {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    T.row(0)(0) = rot.col(0)(0);
    T.row(0)(2) = rot.col(0)(1);
    T.row(2)(0) = rot.col(1)(0);
    T.row(2)(2) = rot.col(1)(1);
    T.col(3)(0) = (*position)(1);
    T.col(3)(2) = (*position)(2);

    boost::shared_ptr<pcl::PointCloud<Point> > transformed_pc(new pcl::PointCloud<Point>);
    pcl::transformPointCloud(*this->input_, *transformed_pc, T);
    register_ += *transformed_pc;

    //this->transformation_.col(3)(0) = -(*position)(1);
    //this->transformation_.col(3)(2) = -(*position)(2);
    }
    first=false;

    //*phi = (*position)(3);
    return true;
  }

  virtual boost::shared_ptr<pcl::PointCloud<Point> > getMap() {
    return register_.makeShared(); //dummy
  }

private:

  unsigned long CreateMeasurementModel()
  {
    BFL::ColumnVector measNoise_Mu(2);
    measNoise_Mu(1) = 0.0;
    measNoise_Mu(2) = 0.0;

    BFL::SymmetricMatrix measNoise_Cov(2);
    measNoise_Cov(1,1) = pow(0.1,2); //0.5
    measNoise_Cov(1,2) = 0;
    measNoise_Cov(2,1) = 0;
    measNoise_Cov(2,2) = pow(0.1,2); //0.5
    BFL::Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);

    /// create the model
    MeasPDF2DLandmark *meas_pdf = new MeasPDF2DLandmark(measurement_Uncertainty);

    m_MeasModel = new BFL::AnalyticMeasurementModelGaussianUncertainty(meas_pdf);
    return ipa_CameraSensors::RET_OK;
  }

  unsigned long CreateSystemModel()
  {
    BFL::ColumnVector sysNoise_Mu(3);
    sysNoise_Mu(1) = 0.0;
    sysNoise_Mu(2) = 0.0;
    sysNoise_Mu(3) = 0.0;

    BFL::SymmetricMatrix sysNoise_Cov(3);
    sysNoise_Cov(1,1) = pow(0.05,2); //0.01
    sysNoise_Cov(1,2) = 0.0;
    sysNoise_Cov(1,3) = 0.0;
    sysNoise_Cov(2,1) = 0.0;
    sysNoise_Cov(2,2) = pow(0.05,2); //0.01
    sysNoise_Cov(2,3) = 0.0;
    sysNoise_Cov(3,1) = 0.0;
    sysNoise_Cov(3,2) = 0.0;
    //sysNoise_Cov(3,3) = pow(0.025,2);
    sysNoise_Cov(3,3) = pow(0.017*3,2); //0.017


    BFL::Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    /// create the nonlinear system model
    SysPDF2DLaserNav *sys_pdf = new SysPDF2DLaserNav(system_Uncertainty);
    m_SysModel = new BFL::SystemModel<BFL::ColumnVector>(sys_pdf);
    return ipa_CameraSensors::RET_OK;
  }

  unsigned long AddFeaturesToGlobalMap(AbstractFeatureVector *afl)
  {
    std::cout << "AddFeaturesToGlobalMap" << std::endl;

    for (unsigned int i=0; i<m_FastSLAM.m_Particles.size(); i++)
    {
      m_FastSLAM.m_Particles[i].ValueGet()->SetPose(m_RobotPose);
    }
    //transformation_camera2base_ = m_RobotSim.GetTrafo();
    //CalculateTransformationTOF2PltfBase();

    AbstractFeatureVector::iterator It;
    std::vector<int> mask(afl->size(),0);
    for (It=afl->begin(); It!=afl->end(); It++)
    {
      ColumnVector theta(3);

      Eigen::Vector4f v;
      v(0)=(*It)->Get<double>(M_X);
      v(1)=(*It)->Get<double>(M_Y);
      v(2)=(*It)->Get<double>(M_Z);
      v(3)=0;

      v = this->transformation_ * v;

      (*It)->Get<double>(M_X) = v(0);
      (*It)->Get<double>(M_Y) = v(1);
      (*It)->Get<double>(M_Z) = v(2);

      //TransformCameraToPltfBase(*It);
      //cout<<(*It)->Get<double>(M_X) <<" " <<(*It)->Get<double>(M_Y) <<std::endl;
      (*It)->Get<int>(M_FEATUREID)=-1;
      m_FastSLAM.AddUnknownFeature(*m_MeasModel, *It, 0);
      (*It)->Get<double>(M_X) = m_FastSLAM.m_Particles[0].ValueGet()->GetMap()->find((*It)->Get<int>(M_FEATUREID))->second.operator->()->PostGet()->ExpectedValueGet()(1);
      (*It)->Get<double>(M_Y) = m_FastSLAM.m_Particles[0].ValueGet()->GetMap()->find((*It)->Get<int>(M_FEATUREID))->second.operator->()->PostGet()->ExpectedValueGet()(2);
      (*It)->Get<double>(M_Z) = ((EKF2DLandmark*)(m_FastSLAM.m_Particles[0].ValueGet()->GetMap()->find((*It)->Get<int>(M_FEATUREID))->second.get()))->m_Height;
    }
    m_MapParticle = m_FastSLAM.m_Particles[0].ValueGet();
    m_DataAssociation.AddNewFeatures(afl, mask);
    return ipa_CameraSensors::RET_OK;
  }


};
}

#endif /* REGISTRATION_FASTSLAM_H_ */
