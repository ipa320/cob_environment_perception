#include <boost/bind.hpp>
#include "cob_3d_mapping_tools/gui/impl/core.hpp"

class MainApp : public wxApp
{
  typedef Gui::ResourceTypes::Image Img;
  typedef Gui::ViewTypes::Color Col;

  void OnClick(wxMouseEvent& event, Gui::Resource<Img>* res)
  {
    static int click_count = 0;
    if (event.LeftDClick())
    {
      wxPoint p = event.GetPosition();
      cv::Vec3b& rgb = (*res->getData())(p.y,p.x);
      int h,s,v;
      dc.rgb2hsv(rgb(2),rgb(1),rgb(0),h,s,v);
      std::cout <<"RGB:"<< (int)rgb(2)<<","<<(int)rgb(1)<<","<<(int)rgb(0)<<" HSV:"<<(int)h<<","<<(int)s<<","<<(int)v<<std::endl;
      uint8_t r,g,b;
      dc.hsv2rgb(h,s,v,r,g,b);
      std::cout << "NEW:"<< (int)r<<","<<(int)g<<","<<(int)b<<std::endl;
      dc.addColor(rgb(2),rgb(1),rgb(0));
      ++click_count;
    }
    else if (event.RightDClick())
    {
      uint8_t r,g,b;
      dc.getColor(r,g,b);
      std::cout << "From histogram after "<<click_count<<" points: "<< (int)r<<","<<(int)g<<","<<(int)b<<std::endl;
      click_count = 0;
    }
  }

  bool OnInit()
  {
    if (this->argc < 2) { std::cout << "Please provide an image" << std::endl; exit(0); }
    
    std::string file(wxString(this->argv[1]).mb_str());
    Gui::Resource<Img>* res = Gui::Core::rMan()->create<Img>("res", file);
    Gui::View<Img,Col>* view = res->createView<Col>("Image");

    boost::function<void (wxMouseEvent&, Gui::Resource<Img>*)> f=boost::bind(&MainApp::OnClick, this, _1, _2);
    view->registerMouseCallback(f);
    view->show();
    return true;
  }
  //Gui::Core* c;
  Gui::Tools::DominantColor dc;
};

IMPLEMENT_APP(MainApp)
