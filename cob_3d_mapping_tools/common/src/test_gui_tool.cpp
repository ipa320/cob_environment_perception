/****************************************************************
 *
 * Copyright (c) 2011
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_environment_perception_intern
 * ROS package name: cob_3d_mapping_tools
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: 08/2012
 * ToDo:
 *
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <iostream>
#include <string>
#include <map>

namespace Gui
{
  namespace ResourceType
  {
    typedef std::string PointRGB;
    typedef int Image;
  }

  namespace ViewType
  {
    enum types
    {
      COLOR, DEPTH_Z, DEPTH_3D, HISTOGRAM, CURVATURE
    };
  }


  class View
  {
  public:
    View(const std::string& name) : name_(name) { }
    virtual ~View() { }

    virtual void show()=0;
    virtual ViewType::types type()=0;
    //virtual void hide()=0;
    //virtual void onDataChanged(void*)=0;

  protected:
    std::string name_;
  };


  template<typename ResourceT, ViewType::types t>
  struct ViewFactory
  {
    static View* newView(const std::string& name, ResourceT* r);
  };


  template<typename DataT>
  class Resource
  {
  public:
    Resource(const std::string& name, const DataT& data)
      : name_(name)
      , data_(data)
      { }
    ~Resource() { }

    template<ViewType::types t>
    View* createView(const std::string& name)
    {
      return views_.insert(
        std::pair<std::string, View*>(name, ViewFactory<Resource<DataT>,t >::newView(name, this))).first->second;
    }

    std::string name_;
    DataT data_;
    std::map<std::string, View*> views_;
  };



  template<typename ResourceT>
  class ColorView : public View
  {
  public:
    ColorView(const std::string& name, ResourceT* r)
      : View(name)
      , r_ptr(r)
      { }
    ~ColorView() { }

    void show() { std::cout << "Nothing to see here" << std::endl; }
    ViewType::types type() { return ViewType::COLOR; }

    ResourceT* r_ptr;
  };

  template<>
  void ColorView<Resource<ResourceType::Image> >::show()
  { std::cout << name_ << ": int is " << r_ptr->data_ << std::endl; }

  template<>
  void ColorView<Resource<ResourceType::PointRGB> >::show()
  { std::cout << name_ << ": string is " << r_ptr->data_ << std::endl; }



  template<typename ResourceT>
  class DepthView : public View
  {
  public:
    DepthView(const std::string& name, ResourceT* r)
      : View(name)
      , r_ptr(r)
      { }
    ~DepthView() { }

    void show() { std::cout << "Nothing to see here" << std::endl; }
    ViewType::types type() { return ViewType::DEPTH_Z; }

    ResourceT* r_ptr;
  };

  template<typename ResourceT>
  struct ViewFactory<ResourceT, ViewType::COLOR>
  {
    static View* newView(const std::string& name, ResourceT* r) { return (new ColorView<ResourceT>(name, r)); }
  };

  template<typename ResourceT>
  struct ViewFactory<ResourceT, ViewType::DEPTH_Z>
  {
    static View* newView(const std::string& name, ResourceT* r) { return (new DepthView<ResourceT>(name, r)); }
  };


  class ResourceManager
  {
  private:
    ResourceManager() { }
  public:
    ~ResourceManager() { }

    std::map<std::string, Resource<ResourceType::PointRGB> > rgb;
    std::map<std::string, Resource<ResourceType::Image> > image;

    friend class Core;
  };

  template<typename T>
  struct map_selector
  {
    static Resource<T>* insert(ResourceManager* rm, const Resource<T>& r)
      { std::cout << "ERROR: this is not possible" << std::endl; return NULL; }
  };

  template<>
  struct map_selector<ResourceType::PointRGB>
  {
    static Resource<ResourceType::PointRGB>* insert(ResourceManager* rm, const Resource<ResourceType::PointRGB>& r)
    {
      return &( (rm->rgb.insert(std::pair<std::string, Resource<ResourceType::PointRGB> >(r.name_, r)) ).first->second);
    }
  };

  template<>
  struct map_selector<ResourceType::Image>
  {
    static Resource<ResourceType::Image>* insert(ResourceManager* rm, const Resource<ResourceType::Image>& r)
    {
      return &( (rm->image.insert(std::pair<std::string, Resource<ResourceType::Image> >(r.name_, r)) ).first->second);
    }
  };



  class Core
  {
  private:
    Core() { }

  public:
    ~Core() { }

    static Core* Create()
    {
      static Core c_;
      return &c_;
    }

    template<typename DataT>
    Resource<DataT>* addResource(const std::string& name, const DataT& data)
    {
      Resource<DataT> r = Resource<DataT>(name, data);
      return map_selector<DataT>::insert(&rm, r);
    }
  private:
    ResourceManager rm;
  };
}



int main(int argc, char** argv)
{
  Gui::Core* gui = Gui::Core::Create();

  std::string mystring = "not ready yet";
  Gui::Resource<Gui::ResourceType::PointRGB>* res1 = gui->addResource("res1",mystring);
  Gui::Resource<Gui::ResourceType::Image>* res2 = gui->addResource("res2",3);
  std::cout << res1->name_ << ": " << res1->data_ << std::endl;
  std::cout << res2->name_ << ": " << res2->data_ << std::endl;


  Gui::View* v1 = res1->createView<Gui::ViewType::COLOR>("color1");
  std::cout << "New View: " << v1->type() << std::endl;
  v1->show();
  Gui::View* v2 = res2->createView<Gui::ViewType::COLOR>("color2");
  std::cout << "New View: " << v2->type() << std::endl;
  v2->show();
  Gui::View* v3 = res1->createView<Gui::ViewType::DEPTH_Z>("depth1");
  std::cout << "New View: " << v3->type() << std::endl;
  v3->show();

  
  //boost::function<void (
  //v1->registerMouseCallback(
  //gui.spin();
}
