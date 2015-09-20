#include <cob_3d_experience_mapping/mapping.h>

#include "../include/ros_node.hpp"

int main(int argc, char **argv) {
	if(argc<3) {
		DBG_PRINTF("specify input and output file\n");
		return 0;
	}

	typedef ROS_Node<As_Node> ns;
	typedef cob_3d_experience_mapping::ContextContainer<ns::TContext, ns::TGraph, ns::TMapStates, ns::TMapTransformations> CtxtContainer;
	
	ns::TContext ctxt_;	
	ns::TGraph graph_;
	ns::TMapStates cells_(graph_);
	ns::TMapTransformations trans_(graph_);
	
	CtxtContainer container(ctxt_, graph_, cells_, trans_);
	
	cob_3d_experience_mapping::serialization::restore_content<boost::archive::binary_iarchive>(container, argv[1], true);
	cob_3d_experience_mapping::serialization::save_content<boost::archive::xml_oarchive>(container, argv[2], false);
	
	return 0;
}
