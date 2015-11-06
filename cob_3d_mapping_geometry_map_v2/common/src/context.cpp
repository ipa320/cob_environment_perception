#include <cob_3d_mapping_geometry_map_v2/types/context.h>
#include <scriptstdstring.h>
#include <scriptbuilder.h>
#include <iostream>
#include <boost/make_shared.hpp>

#include <cob_3d_mapping_geometry_map_v2/types/plane.h>

#include <cob_3d_mapping_geometry_map_v2/visualization/marker.h>
#include <cob_3d_mapping_geometry_map_v2/visualization/vis_obj.h>

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
		
	cob_3d_visualization::RvizMarkerManager::get().createTopic("/marker").setFrameId("/camera_depth_frame").clearOld();
}

void GlobalContext::add_scene(const cob_3d_mapping_msgs::PlaneScene &scene)
{
	Context::Ptr ctxt = boost::make_shared<Context>();
	ctxt->add_scene(ctxt, scene);
	
	//TODO: register
	
	scene_ = ctxt;	//at the moment just replacing
}

bool GlobalContext::registerClassifier(Classifier *c) {
	assert(c);
	
	for(ClassifierSet::const_iterator it=classifiers_.begin(); it!=classifiers_.end(); it++)
		if((*it)->class_id()==c->class_id() || (*it)->name()==c->name()) {
			ROS_WARN("classsifier %s (%d) was already registered under %s (%d)", 
				c->name().c_str(), c->class_id(),  (*it)->name().c_str(), (*it)->class_id());
			return false;
		}
		
	classifiers_.push_back(Classifier::Ptr(c));
}

void GlobalContext::visualize_markers() {
	if(!cob_3d_visualization::RvizMarkerManager::get().needed()) {
		ROS_DEBUG("skipt marker visualization as no subscribers are present");
		return;
	}
		
	std::vector<boost::shared_ptr<Visualization::Object> > vis_objs;
	scene_->visualize(vis_objs);
	
	Visualization::Marker stream;
	for(size_t i=0; i<vis_objs.size(); i++)
		vis_objs[i]->serialize(stream);
}

void Context::add_scene(const Context::Ptr &this_ctxt, const cob_3d_mapping_msgs::PlaneScene &scene)
{
	//TODO: add image first
	
	for(size_t i=0; i<scene.planes.size(); i++) {
		add(Object::Ptr( new Plane(this_ctxt, scene.planes[i]) ));
	}
}

void Context::add(const Object::Ptr &obj)
{
	objs_.push_back(obj);
}

void Context::visualize(std::vector<boost::shared_ptr<Visualization::Object> > &vis_objs) {
	for(size_t i=0; i<objs_.size(); i++)
		objs_[i]->visualization(vis_objs);
}
