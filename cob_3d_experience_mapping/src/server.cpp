#include <cob_3d_experience_mapping/mapping.h>

#include "../include/ros_node.hpp"

typedef ROS_Node<As_Node> ns;
typedef cob_3d_experience_mapping::ContextContainer<ns::TContext, ns::TGraph, ns::TMapStates, ns::TMapTransformations> CtxtContainer;

typedef ns::State State;

HIBERLITE_EXPORT_CLASS(CtxtContainer)

int main(int argc, char **argv) {
	/*if(argc<3) {
		DBG_PRINTF("specify input and output file\n");
		return 0;
	}*/
	
	
	hiberlite::Database db("sample.db");
	//register bean classes
	db.registerBeanClass<CtxtContainer>();
	//drop all tables beans will use
	db.dropModel();
	//create those tables again with proper schema
	db.createModel();
	
	ns::TContext ctxt_;	
	ns::TGraph graph_;
	ns::TMapStates states_(graph_);
	ns::TMapTransformations trans_(graph_);
	
	CtxtContainer container(&ctxt_, &graph_, &states_, &trans_);
	
	cob_3d_experience_mapping::algorithms::init<ns::Transformation>(graph_, ctxt_, states_, trans_);
	
	hiberlite::bean_ptr<CtxtContainer> p=db.copyBean(container);	//create a managed copy of the object
	
	//cob_3d_experience_mapping::serialization::restore_content<boost::archive::binary_iarchive>(container, argv[1], true);
	//cob_3d_experience_mapping::serialization::save_content<boost::archive::xml_oarchive>(container, argv[2], false);
	
	return 0;
}
