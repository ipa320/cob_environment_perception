
#include <signal.h>
#include <boost/filesystem.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include <cob_3d_mapping_tools/keyboard_console_listener.hpp>

typedef cob_3d_mapping_tools::KeyboardConsoleListener KCL;

bool end, next;
rosbag::Bag *p_bag;

void signal_handler(int sig)
{
  KCL::get().reset(sig);
  if(p_bag) p_bag->close();
  exit(1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_tf_delayer");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/registration/pc_aligned",1);


  std::cout << "Reads a bagfile containing pointclouds and world transformations" << std::endl;
  std::cout << "Syntax is: <file_in.bag> [<pointcloud topic(=/registration/pc_aligned)>]" << std::endl;

  if (argc != 3 && argc != 2) return (-1);

  rosbag::Bag bag;
  p_bag = &bag;
  rosbag::View view;
  rosbag::View::iterator vit;
  tf::tfMessage::ConstPtr last_tf;
  sensor_msgs::PointCloud2::ConstPtr last_pc;
  //tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  try { bag.open (argv[1], rosbag::bagmode::Read); }
  catch (rosbag::BagException) { std::cerr << "Error opening file " << argv[1] << std::endl; return (-1); }

  view.addQuery(bag, rosbag::TopicQuery( argc==2 ? "/registration/pc_aligned" : argv[2] ));
  view.addQuery(bag, rosbag::TypeQuery("tf/tfMessage"));
  vit = view.begin();


  KCL::get().init();
  signal(SIGINT,signal_handler);

  while(1)
  {
    // wait for user input, then publish
    KCL::get().waitForIt(KCL::KEYS::S);
    // iterate bag to next cloud
    while(last_pc==NULL)
    {
      if(vit == view.end())
      {
        KCL::get().reset();
        bag.close();
        std::cout << "Done" << std::endl;
        return 0;
      }

      last_tf = vit->instantiate<tf::tfMessage>();
      if (last_tf != NULL)
      {
        tf_broadcaster.sendTransform(last_tf->transforms);
        ros::spinOnce ();
      }
      last_pc = vit->instantiate<sensor_msgs::PointCloud2>();
      ++vit;
    }
    std::cout << "Cloud: " << last_pc->header.stamp << std::endl;
    pub.publish(last_pc);
    last_pc.reset();
    last_tf.reset();
  }

  return 0;
}
