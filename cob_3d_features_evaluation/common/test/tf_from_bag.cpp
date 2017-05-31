#include <sstream>
#include <boost/filesystem.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <fstream>

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "tf_from_bag");
  if (argc < 5)
  {
    std::cerr << "Syntax is: " << argv[0] << " <file_in.bag> <output_file> <link1> <link2>" << std::endl;
    std::cerr << "Example: " << argv[0] << " data.bag tfs.csv /kinect /world" << std::endl;
    return (-1);
  }

  // TF
  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  rosbag::Bag bag;
  rosbag::View view;
  rosbag::View::iterator view_it;

  try
  {
    bag.open (argv[1], rosbag::bagmode::Read);
  } 
  catch (rosbag::BagException) 
  {
    std::cerr << "Error opening file " << argv[1] << std::endl;
    return (-1);
  }
  
  std::ofstream ofstr(argv[2]);

  view.addQuery (bag, rosbag::TypeQuery ("sensor_msgs/PointCloud2"));
  view.addQuery (bag, rosbag::TypeQuery ("tf/tfMessage"));
  view_it = view.begin ();
  
  const std::string DL="\t";
  const std::string NL="\n";
  
  ofstr<<"file"<<DL<<argv[1]<<NL;

  ros::Duration r (0.001);
  // Loop over the whole bag file
  while (view_it != view.end ())
  {
    // Handle TF messages first
    tf::tfMessage::ConstPtr tf = view_it->instantiate<tf::tfMessage> ();
    if (tf != NULL)
    {
      tf_broadcaster.sendTransform (tf->transforms);
      ros::spinOnce ();
	usleep(1000*33);
      
      tf::StampedTransform transform;
	try {
      tf_listener.lookupTransform(argv[3], argv[4], ros::Time(0), transform);
      
      ofstr<<"tf"<<DL<<transform.stamp_<<DL
      <<transform.getOrigin().x()<<DL<<transform.getOrigin().y()<<DL<<transform.getOrigin().z()<<DL
      <<transform.getRotation().x()<<DL<<transform.getRotation().y()<<DL<<transform.getRotation().z()<<DL<<transform.getRotation().w()<<NL;
      ofstr.flush();
	} catch(...) {}
    }
    // Increment the iterator
    ++view_it;
  }
  
  ofstr.close();

  return (0);
}
