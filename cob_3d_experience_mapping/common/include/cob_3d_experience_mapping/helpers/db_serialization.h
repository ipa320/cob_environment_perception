#pragma once

#include "../defs.h"
#include "../../../../libs/hiberlite/include/hiberlite.h"
#include <boost/utility/enable_if.hpp>


#define UNIVERSAL_SERIALIZE_SHORT()  \
	friend class hiberlite::access;\
	\
	template<class Archive>\
	/*\
	 * @brief serialization for hiberlite (SQLite)\
	 */\
	void hibernate(Archive & ar)\
	{		serialize<Archive, hiberlite::sql_nvp>(ar, CURRENT_SERIALIZATION_VERSION); }\
	\
	template<class Archive>\
	/*\
	 * @brief serialization for boost\
	 */\
	void serialize(Archive & ar, const unsigned int version)\
	{		serialize<Archive, boost::serialization::nvp>(ar,version); }
	
#define UNIVERSAL_SERIALIZE()  \
	UNIVERSAL_SERIALIZE_SHORT()\
	/*\
	 * @brief universal serialization function for boost and hiberlite\
	 */\
	template<class Archive, template <class> class make_nvp>\
	void serialize(Archive & ar, const unsigned int version)


#ifndef BOOST_NO_FUNCTION_TEMPLATE_ORDERING
#define UNIVERSAL_SERIALIZATION_NVP_MODIFIER const
#else
#define UNIVERSAL_SERIALIZATION_NVP_MODIFIER 
#endif


#include <boost/typeof/typeof.hpp>
#define UNIVERSAL_SERIALIZATION_NVP(Field) ((UNIVERSAL_SERIALIZATION_NVP_MODIFIER make_nvp< BOOST_TYPEOF(Field) >) make_nvp< BOOST_TYPEOF(Field) >(BOOST_PP_STRINGIZE(Field),Field))
#define UNIVERSAL_SERIALIZATION_NVP_NAMED(Name, Field) ((UNIVERSAL_SERIALIZATION_NVP_MODIFIER make_nvp< BOOST_TYPEOF(Field) >) make_nvp< BOOST_TYPEOF(Field) >(Name,Field))

//some template matching to get loading/saving value for boost and hiberlite
template<class Archive, typename Enable = void>
struct UNIVERSAL_CHECK {
	static bool is_loading(Archive &ar) {
		return ar.is_loading();
	}
	static bool is_saving(Archive &ar) {
		return ar.is_saving();
	}
};

template<class Archive>
struct UNIVERSAL_CHECK<Archive, typename boost::enable_if<std::is_base_of<boost::archive::detail::basic_oarchive, Archive>>::type> {
	static bool is_loading(Archive &ar) {
		return Archive::is_loading::value;
	}
	static bool is_saving(Archive &ar) {
		return Archive::is_saving::value;
	}
};

template<class Archive>
struct UNIVERSAL_CHECK<Archive, typename boost::enable_if<std::is_base_of<boost::archive::detail::basic_iarchive, Archive>>::type> {
	static bool is_loading(Archive &ar) {
		return Archive::is_loading::value;
	}
	static bool is_saving(Archive &ar) {
		return Archive::is_saving::value;
	}
};
//template magic ended...


/*
 * an example for member function:
 * 	
UNIVERSAL_SERIALIZE()
{
	size_t num=0;
	ar & UNIVERSAL_SERIALIZATION_NVP(num);
	float bb=0;
	ar & UNIVERSAL_SERIALIZATION_NVP(bb);

	UNIVERSAL_CHECK<Archive>::is_loading(ar);
}
 * 
 */
