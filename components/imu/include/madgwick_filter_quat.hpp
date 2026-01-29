/* madgwick_filter_quat.hpp
 *  - brief subclass to expose quaternion values
 */
#pragma once

#include "madgwick_filter.hpp" // original file

namespace espp {
class MadgwickFilterQuat : public espp::MadgwickFilter {
public:
  using espp::MadgwickFilter::MadgwickFilter; // inherit constructors

  void get_quaternion(float &w, float &x, float &y, float &z) const {
    w = q0;
    x = q1;
    y = q2;
    z = q3;
  }
};
} // namespace espp
