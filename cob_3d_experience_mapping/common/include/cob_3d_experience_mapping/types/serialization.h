#pragma once

#include <fstream>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
/*
 * possible arhives:
 *  - boost::archive::xml_oarchive
 *  - boost::archive::text_oarchive
 *  - boost::archive::binary_oarchive
 */
 
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "../helpers/db_serialization.h"

//! interfaces and implementations of cob_3d_experience_mapping
namespace cob_3d_experience_mapping{
//! interfaces and implementations needed to serialize map with boost
namespace serialization {

template<class TArchive, class TContent>
void export_content(const TContent &s, std::ostream &ostr){
    {
		TArchive oa(ostr);
		oa << BOOST_SERIALIZATION_NVP(s);
	}
    ostr.flush();
}

template<class TArchive, class TContent>
void import_content(TContent &s, std::istream &istr)
{
    TArchive ia(istr);

    // restore the schedule from the archive
    ia >> BOOST_SERIALIZATION_NVP(s);
}

template<class TArchive, class TContent>
void export_content_compr(const TContent &s, std::ostream &_ostr){
    {
		boost::iostreams::filtering_ostream ostr;
		ostr.push(boost::iostreams::gzip_compressor());
		ostr.push(_ostr);
    
		TArchive oa(ostr);
		oa << BOOST_SERIALIZATION_NVP(s);
	}
    _ostr.flush();
}

template<class TArchive, class TContent>
void import_content_compr(TContent &s, std::istream &_istr)
{
	boost::iostreams::filtering_istream istr;
    istr.push(boost::iostreams::gzip_decompressor());
    istr.push(_istr);
    
    TArchive ia(istr);

    // restore the schedule from the archive
    ia >> BOOST_SERIALIZATION_NVP(s);
}



template<class TArchive, class TContent>
void save_content(const TContent &s, const char * filename, const bool compression=false){
    // make an archive
    std::ofstream ofs(filename);
    assert(ofs.good());
    if(compression)
		export_content_compr<TArchive>(s, ofs);	
    else
		export_content<TArchive>(s, ofs);
}
//default archive
template<class TContent>
void save_content(const TContent &s, const char * filename, const bool compression=false){
	save_content<boost::archive::xml_oarchive>(s,filename,compression);
}

template<class TArchive, class TContent>
void
restore_content(TContent &s, const char * filename, const bool compression=false)
{
    // open the archive
    std::ifstream ifs(filename);
    assert(ifs.good());
    if(compression)
		import_content_compr<TArchive>(s, ifs);
	else
		import_content<TArchive>(s, ifs);
}
//default archive
template<class TContent>
void restore_content(TContent &s, const char * filename, const bool compression=false) {
	restore_content<boost::archive::xml_iarchive>(s,filename,compression);
}

template<class T>
class serializable_shared_ptr : public boost::shared_ptr<T>
{
	public:
		serializable_shared_ptr() {}
		
		serializable_shared_ptr(const boost::shared_ptr<T> &o) :
			boost::shared_ptr<T>(o)
		{}
		
		UNIVERSAL_SERIALIZE()
		{
			if(!this->get())
				this->reset(new T());
			this->get()->serialize<Archive, make_nvp>(ar, version);
		}
};


}	//namespaces
}
