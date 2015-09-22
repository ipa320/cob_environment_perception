#include <cob_3d_experience_mapping/mapping.h>
#include <cob_3d_experience_mapping/helpers/network.h>

#include "../include/ros_node.hpp"

typedef ROS_Node<As_Node> ns;
typedef cob_3d_experience_mapping::ClientIdTsGenerator<ns::State, ns::Feature, uint8_t /*client id*/> TClientIdGenerator;
typedef cob_3d_experience_mapping::Context<ns::Scalar /*energy*/, ns::State /*state*/, ns::Feature, Eigen::Matrix<float,1,2>/*energy weight*/, ns::Transformation/*tranformation*/, TClientIdGenerator > TContext;
typedef cob_3d_experience_mapping::IncrementalContextContainer<TContext, ns::TGraph, ns::TMapStates, ns::TMapTransformations> CtxtContainer;

typedef ns::State State;

HIBERLITE_EXPORT_CLASS(CtxtContainer)

int main(int argc, char **argv) {
	/*if(argc<3) {
		DBG_PRINTF("specify input and output file\n");
		return 0;
	}*/
	
    boost::asio::io_service io_service;

    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), 12347);
    boost::asio::ip::tcp::acceptor acceptor(io_service, endpoint);
    
    bool compression = true;
    typedef boost::archive::binary_iarchive TArchive;
    

	
	hiberlite::Database db("sample.db");
	//register bean classes
	db.registerBeanClass<CtxtContainer>();
	//drop all tables beans will use
	db.dropModel();
	//create those tables again with proper schema
	db.createModel();
	
	//create indexes
	//TODO:
	
	
	TContext ctxt_;	
	ns::TGraph graph_;
	ns::TMapStates states_(graph_);
	ns::TMapTransformations trans_(graph_);
	
	CtxtContainer container(&ctxt_, &graph_, &states_, &trans_, 1);
	
	cob_3d_experience_mapping::algorithms::init<ns::Transformation>(graph_, ctxt_, states_, trans_);
	
	hiberlite::bean_ptr<CtxtContainer> p=db.copyBean(container);	//create a managed copy of the object
	
	//cob_3d_experience_mapping::serialization::restore_content<boost::archive::binary_iarchive>(container, argv[1], true);
	//cob_3d_experience_mapping::serialization::save_content<boost::archive::xml_oarchive>(container, argv[2], false);

    for (;;)
    {
      boost::asio::ip::tcp::iostream stream;
      boost::system::error_code ec;
      acceptor.accept(*stream.rdbuf(), ec);
      if (!ec)
      {
		if(compression)
			cob_3d_experience_mapping::serialization::import_content_compr<TArchive>(container, stream);
		else
			cob_3d_experience_mapping::serialization::import_content<TArchive>(container, stream);
      }
    }
	
	return 0;
}
