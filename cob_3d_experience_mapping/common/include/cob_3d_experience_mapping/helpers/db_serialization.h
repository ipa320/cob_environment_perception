#pragma once

#include "../../../../libs/hiberlite/include/hiberlite.h"


#define UNIVERSAL_SERIALIZE_EXT(ext, ext_args, args_types, args) \
	friend class hiberlite::access;\
	\
	template<class Archive ext>\
	void hibernate(Archive & ar args_types)\
	{		serialize<Archive, hiberlite::sql_nvp ext_args>(ar,0 args); }\
	\
	template<class Archive ext>\
	void serialize(Archive & ar, const unsigned int version args_types)\
	{		serialize<Archive, boost::serialization::nvp ext_args>(ar,version args); }\
	template<class Archive, template <class> class make_nvp ext>\
	void serialize(Archive & ar, const unsigned int version args_types)
	
#define US_COMMA ,
#define US_REF &

#define UNIVERSAL_SERIALIZE() UNIVERSAL_SERIALIZE_EXT(,,,)


#ifndef BOOST_NO_FUNCTION_TEMPLATE_ORDERING
#define UNIVERSAL_SERIALIZATION_NVP_MODIFIER const
#else
#define UNIVERSAL_SERIALIZATION_NVP_MODIFIER 
#endif

#if (defined _MSC_VER && _MSC_VER< 1600)
#include <boost/typeof/typeof.hpp>
#define UNIVERSAL_SERIALIZATION_NVP(Field) ((UNIVERSAL_SERIALIZATION_NVP_MODIFIER make_nvp< BOOST_TYPEOF(Field) >) make_nvp< BOOST_TYPEOF(Field) >(BOOST_PP_STRINGIZE(Field),Field))
#else
#define UNIVERSAL_SERIALIZATION_NVP(Field) ((UNIVERSAL_SERIALIZATION_NVP_MODIFIER make_nvp< decltype(Field) >) make_nvp< decltype(Field) >(BOOST_PP_STRINGIZE(Field),Field))
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
