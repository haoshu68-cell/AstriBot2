#pragma once
// Scaled compensated products/sums plus a square-root residual correction.
// Preserve Python math.hypot/math.dist rounding at policy admission thresholds;
// no tolerance is added. Explicit fma captures each product's rounding error.
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
namespace astribot::navigation::policy {
namespace detail {
template<std::size_t N> inline double compensated_norm(const std::array<double,N>& values) {
  double maximum=0.;bool has_nan=false;
  for(double v:values){maximum=std::max(maximum,std::abs(v));has_nan=has_nan||std::isnan(v);}
  if(std::isinf(maximum))return maximum;
  if(has_nan)return std::numeric_limits<double>::quiet_NaN();
  if(maximum==0.)return 0.;
  int exponent=0;std::frexp(maximum,&exponent);
  if(exponent<-1023) {
    // Scaling a subnormal by 2**(-exponent) can overflow. Preserve Python's
    // ratio/sum branch, including its final subnormal rounding.
    double sum=1.,error=0.;
    for(double v:values) {
      const double ratio=std::abs(v)/maximum,square=ratio*ratio,previous=sum;
      sum+=square;error+=(previous-sum)+square;
    }
    return maximum*std::sqrt(sum-1.+error);
  }
  const double scale=std::ldexp(1.,-exponent);
  double sum=0.,error=0.;
  for(double v:values) {
    const double x=v*scale;
    const double square=x*x,product_error=std::fma(x,x,-square);
    const double next=sum+square;
    const double addition_error=sum>=square?(sum-next)+square:(square-next)+sum;
    error+=addition_error+product_error;sum=next;
  }
  const double root=std::sqrt(sum);
  const double corrected=root+(std::fma(-root,root,sum)+error)/(2.*root);
  return corrected/scale;
}
}
inline double euclidean_norm(double x,double y){return detail::compensated_norm(std::array<double,2>{x,y});}
inline double euclidean_norm(double x,double y,double z){return detail::compensated_norm(std::array<double,3>{x,y,z});}
}  // namespace astribot::navigation::policy
