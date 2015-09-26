#pragma once

#include <boost/asio/ip/tcp.hpp>


//! interfaces and implementations of cob_3d_experience_mapping
namespace cob_3d_experience_mapping{
//! interfaces and implementations needed to serialize map with boost
namespace serialization {
	
	template<class _TClientId, class _TID>
	struct NetworkHeader {
		typedef _TClientId TClientId;
		typedef _TID TID;
		
		TClientId client_;
		TID ts_states_;
		TID ts_fts_;
		TID ts_trans_;
		bool compression_;
		
		NetworkHeader():
			client_((TClientId)-1), ts_states_((TID)-1), ts_fts_((TID)-1), ts_trans_((TID)-1), compression_(true)
		{ }
		
		NetworkHeader(const TClientId &client, const TID &ts_states, const TID &ts_fts, const TID &ts_trans, const bool compression = true):
			client_(client), ts_states_(ts_states), ts_fts_(ts_fts), ts_trans_(ts_trans), compression_(compression)
		{ }
		
		UNIVERSAL_SERIALIZE()
		{
		   assert(version==CURRENT_SERIALIZATION_VERSION);
		   
		   ar & UNIVERSAL_SERIALIZATION_NVP(client_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(ts_states_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(ts_fts_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(ts_trans_);
		   ar & UNIVERSAL_SERIALIZATION_NVP(compression_);
		}
		   
	};
	
template<class TArchiveIn, class TArchiveOut, class TContent>
void sync_content_client(TContent &s, const char * addr, const char * port, const int timeout_secs=120){
    // make an archive
    boost::asio::ip::tcp::iostream stream;
    
    //set timeout
    stream.expires_from_now(boost::posix_time::seconds(timeout_secs));
    
    //connect to server and upload our data
    stream.connect(addr, port);
    
    assert(stream.good());
    
    //send request header without compression
    typename TContent::TNetworkHeader request_header = s.get_network_header();
	export_content<TArchiveOut>(request_header, stream);
		
    /*if(request_header.compression_)
		export_content_compr<TArchiveOut>(s, stream);	
    else*/
		export_content<TArchiveOut>(s, stream);
		
	DBG_PRINTF("sent data\n");
		
    //retrieve request header without compression
    typename TContent::TNetworkHeader response_header;
	import_content<TArchiveIn>(response_header, stream);
	s.set_network_header(response_header);
	
	DBG_PRINTF("got header\n");
	
    /*if(response_header.compression_)
		import_content_compr<TArchiveIn>(s, stream);
	else*/
		import_content<TArchiveIn>(s, stream);
		
	DBG_PRINTF("got data\n");
}
	
template<class TArchiveIn, class TArchiveOut, class TContent>
void sync_content_server_import_header(TContent &s, std::iostream &stream){    
    assert(stream.good());
    
    //retrieve request header without compression
    typename TContent::TNetworkHeader response_header;
	import_content<TArchiveIn>(response_header, stream);
	s.set_network_header(response_header);
}
	
template<class TArchiveIn, class TArchiveOut, class TContent>
void sync_content_server_import(TContent &s, std::iostream &stream){    
    assert(stream.good());
    
	DBG_PRINTF("waiting for data\n");
	
    /*if(s.get_network_header().compression_)
		import_content_compr<TArchiveIn>(s, stream);
	else*/
		import_content<TArchiveIn>(s, stream);
		
	DBG_PRINTF("got data\n");
}
	
template<class TArchiveIn, class TArchiveOut, class TContent>
void sync_content_server_export(TContent &s, std::iostream &stream){    
    assert(stream.good());
    
    //send request header without compression
    typename TContent::TNetworkHeader request_header = s.get_network_header();
	export_content<TArchiveOut>(request_header, stream);
	
	DBG_PRINTF("sent header\n");
		
    /*if(request_header.compression_)
		export_content_compr<TArchiveOut>(s, stream);	
    else*/
		export_content<TArchiveOut>(s, stream);
		
	DBG_PRINTF("sent data\n");
}

}
}

