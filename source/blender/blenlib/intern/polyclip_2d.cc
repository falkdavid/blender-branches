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

/** \file
 * \ingroup bli
 */

#include <algorithm>
#include <iostream>

#include "BLI_double2.hh"
#include "BLI_map.hh"
#include "BLI_vector.hh"

#include "BLI_polyclip_2d.hh"

extern "C" {
#include "BLI_polyclip_2d.h"
}

#define MIN_MITER_LENGTH 0.0001
#define CUT_OFF_ANGLE 0.0001
namespace blender::polyclip {

static void generate_arc_from_point_to_point(VertList &list,
                                             VertList::iterator &it_from,
                                             VertList::iterator &it_to,
                                             const double2 center_pt,
                                             const uint subdivisions,
                                             const bool clockwise,
                                             const bool is_left)
{
  double2 from_pt = double2((*it_from).co);
  double2 to_pt = double2((*it_to).co);
  double2 vec_from = from_pt - center_pt;
  double2 vec_to = to_pt - center_pt;
  if (vec_from.is_zero() || vec_to.is_zero()) {
    return;
  }

  double dot = double2::dot(vec_from, vec_to);
  double det = double2::cross(vec_from, vec_to);
  double angle = clockwise ? M_PI - std::atan2(-det, -dot) : std::atan2(-det, -dot) + M_PI;

  uint num_points = (uint)std::floor((subdivisions + 2) * (angle / M_PI));
  if (num_points > 0) {
    double angle_incr = angle / (double)num_points;

    double2 vec_t;
    VertList::iterator it_last = it_to;
    if (clockwise) {
      // it_last = it_to;
      vec_t = vec_to;
    }
    else {
      // it_last = std::next(it_from);
      vec_t = vec_from;
    }

    for (uint i = 0; i < num_points - 1; i++) {
      double tmp_angle = (double)(i + 1) * angle_incr;
      double2 vec_p = center_pt + double2::rotate(vec_t, tmp_angle);

      Vert *new_point = new Vert(vec_p);
      list.insert(it_last, *new_point);
    }
  }
}

static void generate_semi_circle_from_point_to_point(VertList &list,
                                                     const double2 from_pt,
                                                     VertList::iterator &it_to,
                                                     const double2 center_pt,
                                                     const uint subdivisions,
                                                     const bool is_left)
{
  uint num_points = subdivisions + 2;
  double2 vec_center = from_pt - center_pt;
  if (vec_center.is_zero()) {
    return;
  }

  double angle_incr = M_PI / (double)(num_points - 1);
  for (int i = 1; i < num_points - 1; i++) {
    double angle = (double)i * angle_incr;
    double2 vec_p = center_pt + double2::rotate(vec_center, angle);

    Vert *new_point = new Vert(vec_p);
    list.insert(it_to, *new_point);
  }
}

static void generate_perimeter_cap(VertList &list,
                                   const double2 point,
                                   const double2 other_point,
                                   const double radius,
                                   const uint subdivisions,
                                   const CapType cap_type,
                                   const bool is_left)
{
  double2 cap_vec = other_point - point;
  cap_vec.normalize();

  double2 cap_nvec;
  if (cap_vec.is_zero()) {
    cap_nvec = double2(0, radius);
  }
  else {
    cap_nvec = double2(-cap_vec.y, cap_vec.x);
    cap_nvec *= radius;
  }

  double2 cap_nvec_inv = double2::invert(cap_nvec);
  double2 vec_perimeter = point + cap_nvec;
  double2 vec_perimeter_inv = point + cap_nvec_inv;

  Vert *p_pt = new Vert(vec_perimeter);
  Vert *p_pt_inv = new Vert(vec_perimeter_inv);

  list.push_back(*p_pt);
  VertList::iterator it_p_pt_inv = list.insert(list.end(), *p_pt_inv);

  if (cap_type == CAP_ROUND) {
    generate_semi_circle_from_point_to_point(
        list, vec_perimeter, it_p_pt_inv, point, subdivisions, is_left);
  }
}

/**
 * Calculate the offset (outline) of a polyline as new polyline.
 * \param subdivisions: Number of subdivions for the start and end caps
 * \param pline_radius: polyline radius. Every vertex has an additional radius that is a factor of
 * this radius.
 * \return: offset polyline.
 */
Polyline polyline_offset(Polyline &pline,
                         const uint subdivisions,
                         const double pline_radius,
                         CapType start_cap_t,
                         CapType end_cap_t)
{
  /* sanity check */
  if (pline.num_verts < 1) {
    return Polyline();
  }

  VertList perimeter_right_side, perimeter_left_side;

  Vert first = pline.verts.front();
  Vert last = pline.verts.back();

  double first_radius = pline_radius * first.radius;
  double last_radius = pline_radius * last.radius;

  Vert first_next;
  Vert last_prev;
  if (pline.num_verts > 1) {
    first_next = *std::next(pline.verts.begin(), 1);
    last_prev = *std::prev(pline.verts.end(), 2);
  }
  else {
    first_next = first;
    last_prev = last;
  }

  double2 first_pt = double2(first.co);
  double2 last_pt = double2(last.co);
  double2 first_next_pt = double2(first_next.co);
  double2 last_prev_pt = double2(last_prev.co);

  /* edgecase if single point */
  if (pline.num_verts == 1) {
    first_next_pt.x += 1.0;
    last_prev_pt.x -= 1.0;
  }

  /* generate points for start cap */
  generate_perimeter_cap(perimeter_right_side,
                         first_pt,
                         first_next_pt,
                         first_radius,
                         subdivisions,
                         start_cap_t,
                         false);

  /* generate perimeter points  */
  auto it = std::next(pline.verts.begin());
  for (uint i = 1; i < pline.num_verts - 1; ++i, ++it) {
    Vert curr = *it;
    Vert prev = *std::prev(it);
    Vert next = *std::next(it);
    double r = pline_radius * curr.radius;

    double2 curr_pt = double2(curr.co);
    double2 next_pt = double2(next.co);
    double2 prev_pt = double2(prev.co);

    double2 vec_prev = curr_pt - prev_pt;
    double2 vec_next = next_pt - curr_pt;
    double prev_length = vec_prev.length();
    double next_length = vec_next.length();

    if (vec_prev.compare_zero()) {
      vec_prev = double2(1.0, 0.0);
    }
    else {
      vec_prev.normalize();
    }

    if (vec_next.compare_zero()) {
      vec_next = double2(1.0, 0.0);
    }
    else {
      vec_next.normalize();
    }

    /* Rotate 90 degrees. */
    double2 nvec_prev = double2(-vec_prev.y, vec_prev.x);
    double2 nvec_next = double2(-vec_next.y, vec_next.x);

    double2 vec_tangent = vec_prev + vec_next;
    if (vec_tangent.compare_zero()) {
      vec_tangent = double2(nvec_prev);
    }
    else {
      vec_tangent.normalize();
    }

    double2 vec_miter_left = double2(-vec_tangent.y, vec_tangent.x);

    /* calculate miter length */
    double an1 = double2::dot(vec_miter_left, nvec_prev);
    if (IS_EQ(an1, 0.0)) {
      an1 = 1.0;
    }

    double miter_length = r / an1;
    CLAMP_MIN(miter_length, MIN_MITER_LENGTH);
    vec_miter_left *= miter_length;
    double2 vec_miter_right = double2::invert(vec_miter_left);
    double angle = double2::dot(vec_next, nvec_prev);

    /* add two points if angle is close to beeing straight */
    if (std::abs(angle) < CUT_OFF_ANGLE) {
      nvec_prev.normalize(r);
      nvec_next.normalize(r);

      double2 nvec_prev_pt = curr_pt + nvec_prev;
      double2 nvec_next_pt = curr_pt + double2::invert(nvec_next);

      Vert *normal_prev = new Vert(nvec_prev_pt);
      Vert *normal_next = new Vert(nvec_next_pt);
      perimeter_left_side.push_back(*normal_prev);
      perimeter_right_side.push_back(*normal_next);
    }
    else {
      /* bend to the left */
      if (angle < 0.0) {
        nvec_prev.normalize(r);
        nvec_next.normalize(r);

        double2 nvec_prev_pt = curr_pt + nvec_prev;
        double2 nvec_next_pt = curr_pt + nvec_next;

        Vert *normal_prev = new Vert(nvec_prev_pt);
        Vert *normal_next = new Vert(nvec_next_pt);

        VertList::iterator it_prev = perimeter_left_side.insert(perimeter_left_side.end(),
                                                                *normal_prev);
        VertList::iterator it_next = perimeter_left_side.insert(perimeter_left_side.end(),
                                                                *normal_next);

        generate_arc_from_point_to_point(
            perimeter_left_side, it_prev, it_next, curr_pt, subdivisions, true, true);

        double2 miter_right_pt;
        if (miter_length < prev_length && miter_length < next_length) {
          miter_right_pt = curr_pt + vec_miter_right;
        }
        else {
          miter_right_pt = curr_pt + double2::invert(nvec_next);
        }

        Vert *miter_right = new Vert(miter_right_pt);
        perimeter_right_side.push_back(*miter_right);
      }
      /* bend to the right */
      else {
        nvec_prev.normalize(-r);
        nvec_next.normalize(-r);

        double2 nvec_prev_pt = curr_pt + nvec_prev;
        double2 nvec_next_pt = curr_pt + nvec_next;

        Vert *normal_prev = new Vert(nvec_prev_pt);
        Vert *normal_next = new Vert(nvec_next_pt);

        VertList::iterator it_prev = perimeter_right_side.insert(perimeter_right_side.end(),
                                                                 *normal_prev);
        VertList::iterator it_next = perimeter_right_side.insert(perimeter_right_side.end(),
                                                                 *normal_next);

        generate_arc_from_point_to_point(
            perimeter_right_side, it_prev, it_next, curr_pt, subdivisions, false, false);

        double2 miter_left_pt;
        if (miter_length < prev_length && miter_length < next_length) {
          miter_left_pt = curr_pt + vec_miter_left;
        }
        else {
          miter_left_pt = curr_pt + double2::invert(nvec_prev);
        }

        Vert *miter_left = new Vert(miter_left_pt);
        perimeter_left_side.push_back(*miter_left);
      }
    }
  }

  /* generate points for end cap */
  generate_perimeter_cap(
      perimeter_right_side, last_pt, last_prev_pt, last_radius, subdivisions, end_cap_t, false);

  /* merge both sides to one list */
  perimeter_right_side.reverse();
  perimeter_left_side.splice(perimeter_left_side.end(), perimeter_right_side);
  VertList perimeter_list = perimeter_left_side;

  /* close by creating a point close to the first (make a small gap) */
  Vert close_first = perimeter_list.front();
  Vert close_last = perimeter_list.back();
  double2 close_pt = double2::interpolate(close_first.co, close_last.co, 0.99f);

  if (double2::compare(close_pt, close_first.co, DBL_EPSILON) == false) {
    Vert *close_p_pt = new Vert(close_pt);
    perimeter_list.push_back(*close_p_pt);
  }

  return Polyline(perimeter_list);
}

} /* namespace blender::polyclip */

/* Wrapper for C. */
extern "C" {

/**
 * \param verts:
 *
 */
void BLI_polyline_offset(const double *verts,
                         uint num_verts,
                         const double radius,
                         const uint subdivisions,
                         uint start_cap_t,
                         uint end_cap_t,
                         double **r_offset_verts,
                         uint *r_num_offset_verts)
{
  blender::polyclip::Polyline pline;

  /* Fill pline with data from verts */
  for (uint i = 0; i < num_verts; i++) {
    blender::double3 co = blender::double3(verts[i * 4], verts[i * 4 + 1], verts[i * 4 + 2]);
    double radius = verts[i * 4 + 3];
    pline.verts.push_back(blender::polyclip::Vert(co, radius));
  }
  pline.num_verts = num_verts;

  blender::polyclip::Polyline offset_pline = polyline_offset(
      pline,
      subdivisions,
      radius,
      static_cast<blender::polyclip::CapType>(start_cap_t),
      static_cast<blender::polyclip::CapType>(end_cap_t));

  uint num_offset_verts = offset_pline.num_verts;
  double *offset_verts = (double *)MEM_callocN(sizeof(double) * num_offset_verts * 3, __func__);

  blender::polyclip::VertList::iterator it = offset_pline.verts.begin();
  for (uint i = 0; i < num_offset_verts; i++, it++) {
    blender::polyclip::Vert vert = *it;
    copy_v3_v3_db(&offset_verts[i * 3], vert.co);
  }

  *r_offset_verts = offset_verts;
  *r_num_offset_verts = num_offset_verts;
}

} /* extern "C" */