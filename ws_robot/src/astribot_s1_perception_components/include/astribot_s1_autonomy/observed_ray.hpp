#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

namespace astribot_s1_autonomy
{
// Traverse cells actually crossed by a finite observed ray in continuous grid
// coordinates. Keep its hit cell; absent beams provide no clearing evidence.
template<class Visitor>
void observedRay(double x0, double y0, double x1, double y1,
                 double min_range, double max_range, int width, int height, Visitor visit)
{
  if(width<=0 || height<=0 || !std::isfinite(x0) || !std::isfinite(y0) ||
     !std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(min_range) ||
     !std::isfinite(max_range) || min_range<0 || max_range<=min_range) return;
  const double dx=x1-x0, dy=y1-y0, length=std::hypot(dx,dy);
  if(!std::isfinite(length) || length<=min_range)return;
  double begin=min_range/length, end=std::min(1.,max_range/length);
  auto clip=[&](double origin,double delta,double upper) {
    if(delta==0)return origin>=0 && origin<upper;
    double a=-origin/delta,b=(upper-origin)/delta;
    if(a>b)std::swap(a,b);
    begin=std::max(begin,a);end=std::min(end,b);
    return begin<end;
  };
  if(!clip(x0,dx,width) || !clip(y0,dy,height))return;
  const auto start_x=std::clamp(x0+begin*dx,0.,std::nextafter(double(width),0.));
  const auto start_y=std::clamp(y0+begin*dy,0.,std::nextafter(double(height),0.));
  int x=std::floor(start_x),y=std::floor(start_y);
  const int hit_x=(x1>=0 && x1<width)?int(std::floor(x1)):-1;
  const int hit_y=(y1>=0 && y1<height)?int(std::floor(y1)):-1;
  const int sx=(dx>0)-(dx<0),sy=(dy>0)-(dy<0);
  const double inf=std::numeric_limits<double>::infinity();
  double tx=sx?((sx>0?x+1:x)-x0)/dx:inf;
  double ty=sy?((sy>0?y+1:y)-y0)/dy:inf;
  const double stepx=sx?1./std::abs(dx):inf,stepy=sy?1./std::abs(dy):inf;
  for(int steps=0;steps<width+height+2 && x>=0 && x<width && y>=0 && y<height;++steps) {
    const double next=std::min(tx,ty);
    if(next>begin+1e-12 && !(x==hit_x && y==hit_y))visit(x,y);
    if(next>=end-1e-12)break;
    const bool cross_x=tx<=ty+1e-12,cross_y=ty<=tx+1e-12;
    if(cross_x){x+=sx;tx+=stepx;}
    if(cross_y){y+=sy;ty+=stepy;}
    begin=next;
  }
}
}
