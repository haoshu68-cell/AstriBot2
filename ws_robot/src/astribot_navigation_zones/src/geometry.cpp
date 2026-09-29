#include "astribot_navigation_zones/geometry.hpp"
#include <algorithm>
#include <limits>
#include <regex>
#include <set>
#include <stdexcept>
namespace astribot_navigation_zones {
namespace {
constexpr double eps=1e-9;
void require(bool ok,const char *reason){if(!ok)throw std::invalid_argument(reason);}
double cross(Point a,Point b,Point c){return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);}
double distance(Point p,Point a,Point b){double dx=b[0]-a[0],dy=b[1]-a[1];double t=std::clamp(((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy),0.,1.);return std::hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy);}
bool intersects(Point a,Point b,Point c,Point d){
 if(distance(c,a,b)<=eps||distance(d,a,b)<=eps||distance(a,c,d)<=eps||distance(b,c,d)<=eps)return true;
 return ((cross(a,b,c)>0)!=(cross(a,b,d)>0))&&((cross(c,d,a)>0)!=(cross(c,d,b)>0));
}
bool inside(const std::vector<Point>& p,Point q){bool in=false;for(size_t i=0,j=p.size()-1;i<p.size();j=i++)if((p[i][1]>q[1])!=(p[j][1]>q[1])&&q[0]<(p[j][0]-p[i][0])*(q[1]-p[i][1])/(p[j][1]-p[i][1])+p[i][0])in=!in;return in;}
}
std::vector<Region> parseRegions(const Json & value){
 require(value.is_array()&&value.size()<=64,"ZONES.CAPACITY");std::set<std::string> ids;std::vector<Region> out;
 for(const auto & j:value){Region r;r.id=j.at("id");r.name=j.at("name");r.type=j.at("type");r.enabled=j.at("enabled").get<bool>();r.margin=j.value("margin_m",.1);r.width=j.value("width_m",0.);
 require(std::regex_match(r.id,std::regex("[A-Za-z0-9_-]{1,96}"))&&ids.insert(r.id).second,"ZONES.INVALID_ID");
 require(!r.name.empty()&&r.name.size()<=128,"ZONES.INVALID_NAME");require(r.type=="wall"||r.type=="polygon","ZONES.INVALID_TYPE");
 require(std::isfinite(r.margin)&&r.margin>=0&&r.margin<=2&&std::isfinite(r.width)&&r.width>=0&&r.width<=2,"ZONES.INVALID_MARGIN");
 const auto & points=j.at("points");require(points.is_array()&&(r.type=="wall"?points.size()==2:points.size()>=3&&points.size()<=32),"ZONES.INVALID_POINTS");
 for(const auto & p:points){require(p.is_array()&&p.size()==2,"ZONES.INVALID_POINT");Point q={p[0].get<double>(),p[1].get<double>()};for(auto v:q)require(std::isfinite(v)&&std::abs(v)<=10000,"ZONES.INVALID_POINT");for(auto prior:r.points)require(std::hypot(q[0]-prior[0],q[1]-prior[1])>1e-4,"ZONES.DUPLICATE_POINT");r.points.push_back(q);}
 if(r.type=="wall")require(r.width>=.001,"ZONES.WALL_TOO_THIN");else{
 double area=0;const size_t n=r.points.size();for(size_t i=0;i<n;++i){auto a=r.points[i],b=r.points[(i+1)%n];area+=a[0]*b[1]-a[1]*b[0];for(size_t j=i+1;j<n;++j){if(j==i+1||(i==0&&j==n-1))continue;require(!intersects(a,b,r.points[j],r.points[(j+1)%n]),"ZONES.SELF_INTERSECTION");}}
 require(std::abs(area)>.0001,"ZONES.DEGENERATE_POLYGON");r.width=0.;
 }out.push_back(r);
 }return out;
}
Json encodeRegions(const std::vector<Region>& regions){Json out=Json::array();for(const auto&r:regions)out.push_back({{"id",r.id},{"name",r.name},{"type",r.type},{"enabled",r.enabled},{"points",r.points},{"width_m",r.width},{"margin_m",r.margin}});return out;}
std::array<double,4> bounds(const Region&r,double extra){std::array<double,4>b={INFINITY,INFINITY,-INFINITY,-INFINITY};for(auto p:r.points){b[0]=std::min(b[0],p[0]);b[1]=std::min(b[1],p[1]);b[2]=std::max(b[2],p[0]);b[3]=std::max(b[3],p[1]);}auto m=r.margin+r.width/2+extra;b[0]-=m;b[1]-=m;b[2]+=m;b[3]+=m;return b;}
bool blocked(const std::vector<Region>& regions,double x,double y,double radius){
 if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(radius)||radius<0)return true;
 for(const auto&r:regions){if(!r.enabled)continue;auto b=bounds(r,radius);if(x<b[0]||x>b[2]||y<b[1]||y>b[3])continue;Point p={x,y};
 if(r.type=="polygon"&&inside(r.points,p))return true;
 const size_t n=r.type=="wall"?1:r.points.size();for(size_t i=0;i<n;++i)if(distance(p,r.points[i],r.points[(i+1)%r.points.size()])<=r.margin+r.width/2+radius+eps)return true;
 }return false;
}
bool sweptCircleBlocked(const std::vector<Region>& r,double x,double y,double yaw,double vx,double vy,double wz,double radius,double horizon){
 for(auto v:{x,y,yaw,vx,vy,wz,radius,horizon})if(!std::isfinite(v))return true;
 if(radius<0||horizon<0||horizon>30)return true;const double speed=std::hypot(vx,vy);const int steps=std::max(1,static_cast<int>(std::ceil(horizon/.02)));const double dt=horizon/steps;
 for(int i=0;i<=steps;++i){const double t=i*dt;double dx,dy;if(std::abs(wz)<1e-8){dx=vx*t;dy=vy*t;}else{dx=(vx*std::sin(wz*t)+vy*(std::cos(wz*t)-1))/wz;dy=(vx*(1-std::cos(wz*t))+vy*std::sin(wz*t))/wz;}
 if(blocked(r,x+std::cos(yaw)*dx-std::sin(yaw)*dy,y+std::sin(yaw)*dx+std::cos(yaw)*dy,radius+speed*dt/2))return true;
 }return false;
}
}
