#include <cob_3d_mapping_geometry_map_v2/types/context.h>
#include <scriptstdstring.h>
#include <scriptbuilder.h>
#include <iostream>
#include <boost/make_shared.hpp>

#include <cob_3d_mapping_geometry_map_v2/types/plane.h>

using namespace cob_3d_geometry_map;

void DbgMessageCallback(const asSMessageInfo *msg)
{
	const char *msgType = 0;
	if( msg->type == 0 ) msgType = "Error  ";
	if( msg->type == 1 ) msgType = "Warning";
	if( msg->type == 2 ) msgType = "Info   ";

	std::cout<<msg->section<<" ("<<msg->row<<", "<<msg->col<<") : "<<msgType<<" : "<<msg->message<<std::endl;
}

GlobalContext::GlobalContext() {
	script_engine_ = asCreateScriptEngine();
	
	// Set the message callback to receive information on errors in human readable form.
	int r = script_engine_->SetMessageCallback(asFUNCTION(DbgMessageCallback), 0, asCALL_CDECL);
	assert( r >= 0 );
	
	RegisterStdString(script_engine_);
	
	// Register the function that we want the scripts to call 
	//r = script_engine_->RegisterGlobalFunction("void print(const string &in)", asFUNCTION(print), asCALL_CDECL); assert( r >= 0 );
}

void GlobalContext::add_scene(const cob_3d_mapping_msgs::PlaneScene &scene)
{
	locals_.push_back(boost::make_shared<Context>());
	locals_.back()->add_scene(locals_.back(), scene);
}


void Context::add_scene(const Context::Ptr &this_ctxt, const cob_3d_mapping_msgs::PlaneScene &scene)
{
	//TDO: add image first
	
	for(size_t i=0; i<scene.planes.size(); i++) {
		add(Object::Ptr( new Plane(this_ctxt, scene.planes[i]) ));
	}
}

void Context::add(const Object::Ptr &obj)
{
	objs_.push_back(obj);
}
