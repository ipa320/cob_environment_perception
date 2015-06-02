#pragma once

#include <fstream>
#include <boost/serialization/nvp.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

/*namespace boost {
namespace serialization {

template<class Archive, class Type>
void serialize(Archive & ar, Type & obj, const unsigned int version)
{
    obj.serialize(ar,version);
}

} // namespace serialization
} // namespace boost*/


template<class TContent>
void save_schedule(const TContent &s, const char * filename){
    // make an archive
    std::ofstream ofs(filename);
    assert(ofs.good());
    boost::archive::xml_oarchive oa(ofs);
    oa << BOOST_SERIALIZATION_NVP(s);
}

template<class TContent>
void
restore_schedule(TContent &s, const char * filename)
{
    // open the archive
    std::ifstream ifs(filename);
    assert(ifs.good());
    boost::archive::xml_iarchive ia(ifs);

    // restore the schedule from the archive
    ia >> BOOST_SERIALIZATION_NVP(s);
}
