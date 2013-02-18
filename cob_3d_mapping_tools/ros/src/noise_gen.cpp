/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name:
 * ROS package name:
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author:
 * Supervised by:
 *
 * Date of creation:
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>


// ROS service includes

// external includes
//--
#include <boost/random/normal_distribution.hpp>
#include <boost/random.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
//####################
//#### node class ####
class noise_gen
{
  //
public:
  // create a handle for this node, initialize node
  ros::NodeHandle n;

  // topics to publish
  ros::Publisher pub_;

  // topics to subscribe, callback is called for new messages arriving
  ros::Subscriber sub_;

  // service servers
  //--

  // service clients
  //--

  // global variables
  //--






  // Constructor
  noise_gen()
  {

    std::string in_topic,out_topic;
    in_topic = "cloud_in";
    out_topic = "cloud_out";


    pub_ = n.advertise<sensor_msgs::PointCloud2>(out_topic, 1);
    sub_ = n.subscribe(in_topic, 1, &noise_gen::cloud_callback, this);
  }

  // Destructor
  ~noise_gen()
  {
  }

  double random_number_kinect(double z)
  {


    double mean = 0.0;

    float sigma;



    sigma= ((5*z*z)/100000);

    //    std::cout<<"Z---"<<z<<std::endl;

    //    std::cout<<"SIGMA---"<<sigma<<std::endl;





    typedef boost::normal_distribution<double> NormalDistribution;
    typedef boost::mt19937 RandomGenerator;
    typedef boost::variate_generator<RandomGenerator&,NormalDistribution> GaussianGenerator;
    /** Initiate Random Number generator with current time */
    static RandomGenerator rng(static_cast<unsigned> (time(0)));

    /* Choose Normal Distribution */
    NormalDistribution gaussian_dist(mean, sigma);

    /* Create a Gaussian Random Number generator
     *  by binding with previously defined
     *  normal distribution object
     */
    GaussianGenerator generator(rng, gaussian_dist);
    return generator();
  }

  double random_number_static(double sigma)
  {


    double mean = 0.0;






    typedef boost::normal_distribution<double> NormalDistribution;
    typedef boost::mt19937 RandomGenerator;
    typedef boost::variate_generator<RandomGenerator&,NormalDistribution> GaussianGenerator;
    /** Initiate Random Number generator with current time */
    static RandomGenerator rng(static_cast<unsigned> (time(0)));

    /* Choose Normal Distribution */
    NormalDistribution gaussian_dist(mean, sigma);

    /* Create a Gaussian Random Number generator
     *  by binding with previously defined
     *  normal distribution object
     */
    GaussianGenerator generator(rng, gaussian_dist);
    return generator();
  }

  // topic callback functions
  // function will be called when a new message arrives on a topic
  void cloud_callback(const sensor_msgs::PointCloud2 cloud_in)
  {




    bool bin_mode = true;
    double sigma = 0.002;


    sensor_msgs::PointCloud2 out_msg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_work (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud_in,*cloud_work);



    for (size_t i = 0; i < cloud_work->points.size(); ++i) {
      //      cloud_work->points[i].x +=random_number();
      //      cloud_work->points[i].y +=random_number();

      if ( isnan( cloud_work->points[i].z)==true)
      {
        continue;
      }
      else
      {
        if(bin_mode==true)
        {
//          double z_float=  cloud_work->points[i].z+random_number_kinect(cloud_work->points[i].z);
          double z_float=  cloud_work->points[i].z+random_number_static(sigma);


          double z_bin =round(1090-(348/z_float));

          cloud_work->points[i].z = (-348/(z_bin-1090));
        }
        else
          cloud_work->points[i].z += random_number_static(sigma);
      }

    }
    pcl::toROSMsg(*cloud_work,out_msg);

    pub_.publish(out_msg);

  }


};



//#######################
//#### main programm ####
int main (int argc, char** argv)
{
  ros::init (argc, argv, "eval_node");
  noise_gen nodeClass;

  while(nodeClass.n.ok())
  {

    ros::spinOnce();
  }

  //    ros::spin();

  return 0;
}


