#pragma once

#include <boost/multiprecision/cpp_int.hpp>
#include <cmath>
#include <type_traits>
#include <utility>
#include <variant>

namespace astribot::navigation::policy {
using Integer = boost::multiprecision::cpp_int;

// Image pixels retain Python's integer/float distinction. Metric geometry still
// uses double; only image bounds require exact comparisons against big integers.
class PixelScalar {
 public:
  PixelScalar(Integer value) : value_(std::move(value)) {}
  PixelScalar(double value) : value_(value) {}
  template<class T, std::enable_if_t<std::is_integral_v<T> && !std::is_same_v<T, bool>, int> = 0>
  PixelScalar(T value) : value_(Integer(value)) {}
  PixelScalar(bool) = delete;

  bool is_integer() const { return std::holds_alternative<Integer>(value_); }
  const Integer& integer() const { return std::get<Integer>(value_); }
  double floating() const { return std::get<double>(value_); }

  bool is_finite() const {
    if (!is_integer()) return std::isfinite(floating());
    // CPython's int -> binary64 conversion raises OverflowError at the halfway
    // point between DBL_MAX and 2**1024 (the tie rounds toward the even 2**1024).
    static const Integer overflow = (Integer(1) << 1024) - (Integer(1) << 970);
    return integer() > -overflow && integer() < overflow;
  }

  friend bool operator==(const PixelScalar& a, const PixelScalar& b) { return compare(a,b)==0; }
  friend bool operator!=(const PixelScalar& a, const PixelScalar& b) { return compare(a,b)!=0; }
  friend bool operator<(const PixelScalar& a, const PixelScalar& b) { return compare(a,b)==-1; }
  friend bool operator>(const PixelScalar& a, const PixelScalar& b) { return compare(a,b)==1; }
  friend bool operator<=(const PixelScalar& a, const PixelScalar& b) { const int c=compare(a,b); return c==-1||c==0; }
  friend bool operator>=(const PixelScalar& a, const PixelScalar& b) { const int c=compare(a,b); return c==1||c==0; }

 private:
  const std::variant<Integer,double> value_;

  // 2 denotes unordered, preserving normal NaN comparison behavior even though
  // ImageBox rejects nonfinite coordinates before evaluating its bounds.
  static int compare_integer_float(const Integer& integer, double value) {
    if (std::isnan(value)) return 2;
    if (std::isinf(value)) return value>0 ? -1 : 1;
    const Integer truncated(value);
    if (integer < truncated) return -1;
    if (integer > truncated) return 1;
    if (std::trunc(value)==value) return 0;
    return value>0 ? -1 : 1;
  }
  static int compare(const PixelScalar& a, const PixelScalar& b) {
    if (a.is_integer() && b.is_integer()) return a.integer()<b.integer() ? -1 : a.integer()>b.integer() ? 1 : 0;
    if (a.is_integer()) return compare_integer_float(a.integer(),b.floating());
    if (b.is_integer()) { const int c=compare_integer_float(b.integer(),a.floating()); return c==2 ? 2 : -c; }
    if (std::isnan(a.floating()) || std::isnan(b.floating())) return 2;
    return a.floating()<b.floating() ? -1 : a.floating()>b.floating() ? 1 : 0;
  }
};
}  // namespace astribot::navigation::policy
