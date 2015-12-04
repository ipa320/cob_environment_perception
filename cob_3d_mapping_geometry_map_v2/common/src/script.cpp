
#include <angelscript.h>

#include <cob_3d_mapping_geometry_map_v2/types/object.h>

using namespace cob_3d_geometry_map;

bool register_obj(asIScriptEngine *engine, Object *obj, const std::string &inst_name) {
	assert(obj);
	
	int r = engine->RegisterGlobalProperty( ("Map::"+obj->type()+" "+inst_name).c_str(), obj);
	assert(r>=0);
	
	return r>=0;
}
