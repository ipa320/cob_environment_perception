#pragma once

#include "../../../../libs/hiberlite/include/hiberlite.h"


#define UNIVERSAL_SERIALIZE() \
	friend class hiberlite::access;\
	\
	template<class Archive>\
	void hibernate(Archive & ar)\
	{		serialize<Archive, hiberlite::sql_nvp>(ar,0); }\
	\
	template<class Archive>\
	void serialize(Archive & ar, const unsigned int version)\
	{		serialize<Archive, boost::serialization::nvp>(ar,version); }\
	template<class Archive, template <class> class make_nvp>\
	void serialize(Archive & ar, const unsigned int version)
	

#if (defined _MSC_VER && _MSC_VER< 1600)
#include <boost/typeof/typeof.hpp>
#define UNIVERSAL_SERIALIZATION_NVP(Field) make_nvpBOOST_TYPEOF(Field) >(BOOST_PP_STRINGIZE(Field),Field)
#else
#define UNIVERSAL_SERIALIZATION_NVP(Field) make_nvp< decltype(Field) >(BOOST_PP_STRINGIZE(Field),Field)
#endif

/*
 * an example for member function:
 * 	
UNIVERSAL_SERIALIZE()
{
	size_t num=0;
	ar & UNIVERSAL_SERIALIZATION_NVP(num);
	float bb=0;
	ar & UNIVERSAL_SERIALIZATION_NVP(bb);
}
 * 
 */
