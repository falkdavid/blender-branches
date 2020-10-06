/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_double3.hh"

namespace blender {

struct double2 {
  double x, y;

  double2() = default;

  double2(const double *ptr) : x{ptr[0]}, y{ptr[1]}
  {
  }

  double2(double x, double y) : x(x), y(y)
  {
  }

  double2(const double3 &other) : x(other.x), y(other.y)
  {
  }

  operator double *()
  {
    return &x;
  }

  operator const double *() const
  {
    return &x;
  }

  double length() const
  {
    return len_v2_db(*this);
  }

  bool is_zero() const
  {
    return x == 0.0 && y == 0.0;
  }

  bool compare_zero() const
  {
    return IS_EQ(x, 0.0) && IS_EQ(y, 0.0);
  }

  friend double2 operator+(const double2 &a, const double2 &b)
  {
    return {a.x + b.x, a.y + b.y};
  }

  friend double2 operator-(const double2 &a, const double2 &b)
  {
    return {a.x - b.x, a.y - b.y};
  }

  friend double2 operator*(const double2 &a, double b)
  {
    return {a.x * b, a.y * b};
  }

  friend double2 operator/(const double2 &a, double b)
  {
    BLI_assert(b != 0.0);
    return {a.x / b, a.y / b};
  }

  friend double2 operator*(double a, const double2 &b)
  {
    return b * a;
  }

  friend bool operator==(const double2 &a, const double2 &b)
  {
    return a.x == b.x && a.y == b.y;
  }

  friend bool operator!=(const double2 &a, const double2 &b)
  {
    return a.x != b.x || a.y != b.y;
  }

  void operator+=(const double2 &b)
  {
    this->x += b.x;
    this->y += b.y;
  }

  void operator*=(const double b)
  {
    this->x *= b;
    this->y *= b;
  }

  void normalize()
  {
    double l = length();
    if (l == 0.0) {
      x = 0.0;
      y = 0.0;
    }
    else {
      x /= l;
      y /= l;
    }
  }

  void normalize(const double length)
  {
    if (length == 0.0) {
      x = 0.0;
      y = 0.0;
    }
    else {
      normalize();
      *this *= length;
    }
  }

  friend std::ostream &operator<<(std::ostream &stream, const double2 &v)
  {
    stream << "(" << v.x << ", " << v.y << ")";
    return stream;
  }

  static double dot(const double2 &a, const double2 &b)
  {
    return a.x * b.x + a.y * b.y;
  }

  static double2 interpolate(const double2 &a, const double2 &b, double t)
  {
    return a * (1 - t) + b * t;
  }

  static double2 abs(const double2 &a)
  {
    return double2(fabs(a.x), fabs(a.y));
  }

  static double distance(const double2 &a, const double2 &b)
  {
    return (a - b).length();
  }

  static double distance_squared(const double2 &a, const double2 &b)
  {
    return double2::dot(a, b);
  }

  static double cross(const double2 &a, const double2 &b)
  {
    return cross_v2v2_db(a, b);
  }

  static double2 rotate(const double2 &a, const double angle)
  {
    const double co = cos(angle);
    const double si = sin(angle);
    return double2(co * a.x - si * a.y, si * a.x + co * a.y);
  }

  static double2 invert(const double2 &a)
  {
    return double2(-a.x, -a.y);
  }

  static bool compare(const double2 &a, const double2 &b, const double limit)
  {
    return std::abs(a.x - b.x) <= limit && std::abs(a.y - b.y) <= limit;
  }

  /**
   * Return 1 if a, b, c are in counter-clockwise order (e.g. c lies to the left of the line a->b), -1 if
   * the order is clockwise and 0 when they are colinear.
   * Note: this is a nonrobust implementation.
   */
  static int orientation(const double2 &a, const double2 &b, const double2 &c)
  {
    double r = (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x);
    if (r > 0.0) {
      return 1;
    }
    else if (r < 0.0) {
      return -1;
    }
    return 0;
  }

  struct isect_result {
    enum {
      /* Segments are colinear. */
      LINE_LINE_COLINEAR = -1,
      /* Segments do not intersect. */
      LINE_LINE_NONE = 0,
      /* Segments intersect at the ends. */
      LINE_LINE_EXACT = 1,
      /* Segments intersect at a single point and not the ends. */
      LINE_LINE_CROSS = 2,
    } kind;
    /* Factor of intersection of the first segment where 0.0 is the start and 1.0 the end of the
     * segment. */
    double lambda;
    /* Factor of intersection of the second segment where 0.0 is the start and 1.0 the end of the
     * segment. */
    double mu;
  };

  static isect_result isect_seg_seg(const double2 &v1,
                                    const double2 &v2,
                                    const double2 &v3,
                                    const double2 &v4);
};

}  // namespace blender
