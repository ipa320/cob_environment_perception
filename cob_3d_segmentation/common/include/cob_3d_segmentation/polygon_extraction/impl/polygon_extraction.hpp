
template <typename TPoint>
int PolygonExtraction::getPos(int *ch, const int xx, const int yy, const int w, const int h) {
  int p=0;
  const int i=0;
  for(int x=-1; x<=1; x++) {
    for(int y=-1; y<=1; y++) {
      if( xx+x>=0 && yy+y>=0 && xx+x<w && yy+y<h &&
          (x||y) && ch[TPoint::getInd(xx+x,yy+y)]>0)
      {
        p |= (1<<Contour2D::SplineMap[ (y+1)*3 + x+1]);
      }
    }

  }
  return p;
}

bool PolygonExtraction::hasMultiplePositions(unsigned int i)
{
  return !((i != 0) && ((i & (~i + 1)) == i));
}

template<typename TPoint, typename TPolygon>
void PolygonExtraction::outline(const int w, const int h, std::vector<TPoint> out, TPolygon &poly)
{
  std::sort(out.begin(),out.end());
  //std::cout <<"--> D 1"<<std::endl;
  if(ch_size_<w*h) {
    delete [] ch_;
    ch_ = new int[w*h];
    ch_size_=w*h;
  }
  memset(ch_,0,sizeof(int)*ch_size_);
  for(size_t j=0; j<out.size(); j++) {
    ch_[ TPoint::getInd(out[j].x,out[j].y) ]=(int)j+1;
  }

  if(outline_check_size_<out.size()) {
    delete [] outline_check_;
    outline_check_ = new bool[out.size()];
    outline_check_size_=out.size();
  }
  memset(outline_check_,false,out.size());

  int n=-1;
  while(n+1<(int)out.size())
  {
    ++n;
    if(outline_check_[n])
      continue;

    poly.addPolygon();

    int x=out[n].x;
    int y=out[n].y;
    int bf=8;
    int v=0;
    int start_x=x, start_y=y;

    poly.addPoint(x,y);
    //int num=0;
    std::stack<std::pair<int,Contour2D::spline2D> > forked_states;
    std::stack<typename std::vector<TPoint>::size_type> forked_points;
    while(1)
    {
      if(x<0 || y<0 || x>=w || y>=h || ch_[ TPoint::getInd(x,y) ]<1) {
        break;
      }

      int ch_old = ch_[ TPoint::getInd(x,y) ];
      outline_check_[ch_[ TPoint::getInd(x,y) ]-1]=true;
      ch_[ TPoint::getInd(x,y) ]=-2;

      int p=getPos<TPoint>(ch_,x,y,w,h);

      if(p==0|| (!Contour2D::g_Splines[bf][p].x&&!Contour2D::g_Splines[bf][p].y) )
      {
        // There is no valid next point:
        if (forked_states.size() == 0 || (std::abs(x-start_x)+std::abs(y-start_y)) <= 4) { break; }
        //std::cout << "--> Back to fork!" << std::endl;
        // Go back to last forked state
        v = forked_states.top().second.v;
        x = forked_states.top().second.x;
        y = forked_states.top().second.y;
        bf = forked_states.top().second.bf;
        ch_ [ TPoint::getInd(x,y) ] = forked_states.top().first;
        forked_states.pop();
        poly.removeLastPoints(forked_points.top());
        forked_points.pop();
        continue;
      }
      if (hasMultiplePositions((unsigned int)p))
      {
        Contour2D::spline2D s = {v, x, y, bf};
        forked_states.push(std::pair<int,Contour2D::spline2D>(ch_old, s));
        forked_points.push(0);
      }

      v+=v+Contour2D::g_Splines[bf][p].v;
      x+=Contour2D::g_Splines[bf][p].x;
      y+=Contour2D::g_Splines[bf][p].y;
      bf=Contour2D::g_Splines[bf][p].bf;
      //++num;

      if(std::abs(v)>5) {
        v=0;
        ++(forked_points.top());
        poly.addPoint(x,y);
      }
    }

    if(poly.polys_.back().size() < 4 || (std::abs(x-start_x)+std::abs(y-start_y))>4 )
    {
      poly.removePolygon();
    }

  }

  for(size_t j=0; j<out.size(); j++) {
    ch_[ TPoint::getInd(out[j].x,out[j].y) ]=0;
  }

}
