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

#include "BLI_polyclip_2d.h"

#define MIN_MITER_LENGTH 0.0001
#define CUT_OFF_ANGLE 0.0001
namespace blender::polyclip {

// static int generate_arc_from_point_to_point(ListBase *list,
//                                             tPerimeterPoint *from,
//                                             tPerimeterPoint *to,
//                                             float center_pt[3],
//                                             int subdivisions,
//                                             bool clockwise,
//                                             bool is_left)
static int generate_arc_from_point_to_point(VertList &list,
                                            VertList::iterator &it_from,
                                            VertList::iterator &it_to,
                                            const double2 center_pt,
                                            const uint subdivisions,
                                            const bool clockwise,
                                            const bool is_left)
{
  // float vec_from[2];
  // float vec_to[2];
  // sub_v2_v2v2(vec_from, &from->x, center_pt);
  // sub_v2_v2v2(vec_to, &to->x, center_pt);
  // if (is_zero_v2(vec_from) || is_zero_v2(vec_to)) {
  //   return 0;
  // }
  double2 from_pt = double2((*it_from).co);
  double2 to_pt = double2((*it_to).co);
  double2 vec_from = from_pt - center_pt;
  double2 vec_to = to_pt - center_pt;
  if (vec_from.is_zero() || vec_to.is_zero()) {
    return 0;
  }

  // float dot = dot_v2v2(vec_from, vec_to);
  // float det = cross_v2v2(vec_from, vec_to);
  // float angle = clockwise ? M_PI - atan2f(-det, -dot) : atan2f(-det, -dot) + M_PI;

  double dot = double2::dot(vec_from, vec_to);
  double det = double2::cross(vec_from, vec_to);
  double angle = clockwise ? M_PI - std::atan2(-det, -dot) : std::atan2(-det, -dot) + M_PI;

  // /* number of points is 2^(n+1) + 1 on half a circle (n=subdivisions)
  //  * so we multiply by (angle / pi) to get the right amount of
  //  * points to insert */
  // int num_points = (int)(((1 << (subdivisions + 1)) - 1) * (angle / M_PI));
  uint num_points = (uint)std::floor((subdivisions + 3) * (angle / M_PI));
  if (num_points > 0) {
    // float angle_incr = angle / (float)num_points;
    double angle_incr = angle / (double)num_points;

    // float vec_p[3];
    // float vec_t[3];
    // float tmp_angle;
    // tPerimeterPoint *last_point;
    // if (clockwise) {
    //   last_point = to;
    //   copy_v3_v3(vec_t, vec_to);
    // }
    // else {
    //   last_point = from;
    //   copy_v3_v3(vec_t, vec_from);
    // }
    double2 vec_t;
    VertList::iterator it_last;
    if (clockwise) {
      it_last = it_to;
      vec_t = double2(vec_to);
    }
    else {
      it_last = std::next(it_from, 1);
      vec_t = double2(vec_from);
    }

    for (uint i = 0; i < num_points - 1; i++) {
      // tmp_angle = (i + 1) * angle_incr;
      double tmp_angle = (double)(i + 1) * angle_incr;

      // rotate_v2_v2fl(vec_p, vec_t, tmp_angle);
      // add_v2_v2(vec_p, center_pt);
      // vec_p[2] = center_pt[2];
      double2 vec_p = double2::rotate(vec_t, tmp_angle);

      // tPerimeterPoint *new_point = new tPerimeterPoint(vec_p, is_left);
      // if (clockwise) {
      //   BLI_insertlinkbefore(list, last_point, new_point);
      // }
      // else {
      //   BLI_insertlinkafter(list, last_point, new_point);
      // }
      Vert *new_point = new Vert(vec_p);
      list.insert(it_last, new_point);

      if (!clockwise) {
        it_last++;
      }
    }

    return num_points - 1;
  }

  return 0;
}

// static int generate_semi_circle_from_point_to_point(
//     ListBase *list, tPerimeterPoint *from, tPerimeterPoint *to, int subdivisions, bool is_left)
static int generate_semi_circle_from_point_to_point(VertList &list,
                                                    VertList::iterator &it_from,
                                                    Vert &to,
                                                    const double2 center_pt,
                                                    const uint subdivisions,
                                                    const bool is_left)
{
  // int num_points = (1 << (subdivisions + 1)) + 1;
  // float center_pt[3];
  // interp_v3_v3v3(center_pt, &from->x, &to->x, 0.5f);
  uint num_points = subdivisions + 3;

  // float vec_center[2];
  // sub_v2_v2v2(vec_center, &from->x, center_pt);
  // if (is_zero_v2(vec_center)) {
  //   return 0;
  // }
  double2 from_pt = double2((*it_from).co);
  double2 vec_center = from_pt - center_pt;
  if (vec_center.is_zero()) {
    return 0;
  }

  // float vec_p[3];  // temp vector to do the vector math
  // float angle_incr = M_PI / ((float)num_points - 1);
  double angle_incr = M_PI / (double)(num_points - 1);

  // tPerimeterPoint *last_point = from;
  for (int i = 1; i < num_points; i++) {
    // float angle = i * angle_incr;
    double angle = (double)i * angle_incr;

    // /* rotate vector around point to get perimeter points */
    // rotate_v2_v2fl(vec_p, vec_center, angle);
    // add_v2_v2(vec_p, center_pt);
    // vec_p[2] = center_pt[2];
    double2 vec_p = double2::rotate(vec_center, angle);

    // tPerimeterPoint *new_point = new tPerimeterPoint(vec_p, is_left);
    // BLI_insertlinkafter(list, last_point, new_point);

    // last_point = new_point;
    Vert *new_point = new Vert(vec_p);
    it_from++;
    list.insert(it_from, new_point);
  }

  return num_points - 1;
}

static int generate_perimeter_cap(VertList &list,
                                  const double2 point,
                                  const double2 other_point,
                                  const double radius,
                                  const uint subdivisions,
                                  const CapType cap_type,
                                  const bool is_left)
{
  // float cap_vec[2];
  // sub_v2_v2v2(cap_vec, other_point, point);
  // normalize_v2(cap_vec);
  double2 cap_vec = other_point - point;
  cap_vec.normalize();

  // float cap_nvec[2];
  // if (is_zero_v2(cap_vec)) {
  //   cap_nvec[0] = 0;
  //   cap_nvec[1] = radius;
  // }
  // else {
  //   cap_nvec[0] = -cap_vec[1];
  //   cap_nvec[1] = cap_vec[0];
  //   mul_v2_fl(cap_nvec, radius);
  // }
  double2 cap_nvec;
  if (cap_vec.is_zero()) {
    cap_nvec = double2(0, radius);
  }
  else {
    cap_nvec = double2(-cap_vec.y, cap_vec.x);
    cap_nvec *= radius;
  }
  // float cap_nvec_inv[2];
  // negate_v2_v2(cap_nvec_inv, cap_nvec);
  double2 cap_nvec_inv = double2::invert(cap_nvec);

  // float vec_perimeter[3];
  // copy_v3_v3(vec_perimeter, point);
  // add_v2_v2(vec_perimeter, cap_nvec);
  double2 vec_perimeter = point + cap_nvec;

  // float vec_perimeter_inv[3];
  // copy_v3_v3(vec_perimeter_inv, point);
  // add_v2_v2(vec_perimeter_inv, cap_nvec_inv);
  double2 vec_perimeter_inv = point + cap_nvec_inv;

  // tPerimeterPoint *p_pt = new tPerimeterPoint(vec_perimeter, is_left);
  // tPerimeterPoint *p_pt_inv = new tPerimeterPoint(vec_perimeter_inv, is_left);
  Vert *p_pt = new Vert(vec_perimeter);
  Vert *p_pt_inv = new Vert(vec_perimeter_inv);

  // BLI_addtail(list, p_pt);
  // BLI_addtail(list, p_pt_inv);
  list.push_back(p_pt);
  VertList::iterator it_p_pt = list.end();
  list.push_back(p_pt_inv);

  int num_points = 0;
  if (cap_type == CAP_ROUND) {
    num_points += generate_semi_circle_from_point_to_point(
        list, it_p_pt, *p_pt_inv, point, subdivisions, is_left);
  }

  return num_points + 2;
}

/**
 * Calculate the perimeter (outline) of a polyline as new polyline.
 * \param subdivisions: Number of subdivions for the start and end caps
 * \param pline_radius: polyline radius. Every vertex has an additional radius that is a factor of
 * this radius.
 * \return: perimeter polyline.
 */
static Polyline polyline_offset_intern(Polyline &pline,
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
  int num_perimeter_points = 0;

  Vert *first = pline.verts.front();
  Vert *last = pline.verts.back();

  double first_radius = pline_radius * first->radius;
  double last_radius = pline_radius * last->radius;

  Vert *first_next;
  Vert *last_prev;
  if (pline.num_verts > 1) {
    first_next = std::next(pline.verts.begin(), 1);
    last_prev = std::prev(pline.verts.end(), 1);
  }
  else {
    first_next = first;
    last_prev = last;
  }

  // float first_pt[3];
  // float last_pt[3];
  // float first_next_pt[3];
  // float last_prev_pt[3];
  // copy_v3_v3(first_pt, &first->co.x);
  // copy_v3_v3(last_pt, &last->co);
  // copy_v3_v3(first_next_pt, &first_next->x);
  // copy_v3_v3(last_prev_pt, &last_prev->x);

  double2 first_pt = double2(first->co);
  double2 last_pt = double2(last->co);
  double2 first_next_pt = double2(first_next->co);
  double2 last_prev_pt = double2(last_prev->co);

  // /* edgecase if single point */
  // if (pline->num_verts == 1) {
  //   first_next_pt[0] += 1.0f;
  //   last_prev_pt[0] -= 1.0f;
  // }

  /* edgecase if single point */
  if (pline.num_verts == 1) {
    first_next_pt.x += 1.0;
    last_prev_pt.x -= 1.0;
  }

  /* generate points for start cap */
  num_perimeter_points += generate_perimeter_cap(perimeter_right_side,
                                                 first_pt,
                                                 first_next_pt,
                                                 first_radius,
                                                 subdivisions,
                                                 start_cap_t,
                                                 false);

  /* generate perimeter points  */
  // float curr_pt[3], next_pt[3], prev_pt[3];
  // float vec_next[2], vec_prev[2];
  // float nvec_next[2], nvec_prev[2];
  // float nvec_next_pt[3], nvec_prev_pt[3];
  // float vec_tangent[2];

  // float vec_miter_left[2], vec_miter_right[2];
  // float miter_left_pt[3], miter_right_pt[3];

  // for (int i = 1; i < gps->tot_points - 1; i++) {
  for (auto it = std::next(pline.verts.begin()); it != std::prev(pline.verts.end()); ++it) {
    // bGPDspoint *curr = &gps->points[i];
    // bGPDspoint *prev = &gps->points[i - 1];
    // bGPDspoint *next = &gps->points[i + 1];
    // float radius = stroke_radius * curr->pressure;
    Vert curr = *it;
    Vert prev = *std::prev(it);
    Vert next = *std::next(it);
    double r = pline_radius * curr.radius;

    // copy_v3_v3(curr_pt, &curr->x);
    // copy_v3_v3(next_pt, &next->x);
    // copy_v3_v3(prev_pt, &prev->x);

    double2 curr_pt = double2(curr.co);
    double2 next_pt = double2(next.co);
    double2 prev_pt = double2(prev.co);

    // sub_v2_v2v2(vec_prev, curr_pt, prev_pt);
    // sub_v2_v2v2(vec_next, next_pt, curr_pt);
    // float prev_length = len_v2(vec_prev);
    // float next_length = len_v2(vec_next);

    double2 vec_prev = curr_pt - prev_pt;
    double2 vec_next = next_pt - curr_pt;
    double prev_length = vec_prev.length();
    double next_length = vec_next.length();

    // if (normalize_v2(vec_prev) == 0.0f) {
    //   vec_prev[0] = 1.0f;
    //   vec_prev[1] = 0.0f;
    // }
    // if (normalize_v2(vec_next) == 0.0f) {
    //   vec_next[0] = 1.0f;
    //   vec_next[1] = 0.0f;
    // }

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

    // nvec_prev[0] = -vec_prev[1];
    // nvec_prev[1] = vec_prev[0];

    // nvec_next[0] = -vec_next[1];
    // nvec_next[1] = vec_next[0];

    /* Rotate 90 degrees. */
    double2 nvec_prev = double2(-vec_prev.y, vec_prev.x);
    double2 nvec_next = double2(-vec_next.y, vec_next.x);

    // add_v2_v2v2(vec_tangent, vec_prev, vec_next);
    // if (normalize_v2(vec_tangent) == 0.0f) {
    //   copy_v2_v2(vec_tangent, nvec_prev);
    // }
    double2 vec_tangent = vec_prev + vec_next;
    if (vec_tangent.compare_zero()) {
      vec_tangent = double2(nvec_prev);
    }
    else {
      vec_tangent.normalize();
    }

    // vec_miter_left[0] = -vec_tangent[1];
    // vec_miter_left[1] = vec_tangent[0];
    double2 vec_miter_left = double2(-vec_tangent.y, vec_tangent.x);

    /* calculate miter length */
    // float an1 = dot_v2v2(vec_miter_left, nvec_prev);
    // if (an1 == 0.0f) {
    //   an1 = 1.0f;
    // }
    // float miter_length = radius / an1;
    // if (miter_length <= 0.0f) {
    //   miter_length = 0.01f;
    // }

    double an1 = double2::dot(vec_miter_left, nvec_prev);
    if (IS_EQ(an1, 0.0)) {
      an1 = 1.0;
    }
    double miter_length = r / an1;
    CLAMP_MIN(miter_length, MIN_MITER_LENGTH);

    // normalize_v2_length(vec_miter_left, miter_length);
    vec_miter_left *= miter_length;

    // copy_v2_v2(vec_miter_right, vec_miter_left);
    // negate_v2(vec_miter_right);
    double2 vec_miter_right = double2::invert(vec_miter_left);

    // float angle = dot_v2v2(vec_next, nvec_prev);
    double angle = double2::dot(vec_next, nvec_prev);
    /* add two points if angle is close to beeing straight */
    // if (fabsf(angle) < 0.0001f) {
    if (std::abs(angle) < CUT_OFF_ANGLE) {
      // normalize_v2_length(nvec_prev, radius);
      // normalize_v2_length(nvec_next, radius);
      nvec_prev.normalize(r);
      nvec_next.normalize(r);

      // copy_v3_v3(nvec_prev_pt, curr_pt);
      // add_v2_v2(nvec_prev_pt, nvec_prev);

      double2 nvec_prev_pt = curr_pt + nvec_prev;

      // copy_v3_v3(nvec_next_pt, curr_pt);
      // negate_v2(nvec_next);
      // add_v2_v2(nvec_next_pt, nvec_next);

      double2 nvec_next_pt = curr_pt + double2::invert(nvec_next);

      // tPerimeterPoint *normal_prev = new tPerimeterPoint(nvec_prev_pt, true);
      // tPerimeterPoint *normal_next = new tPerimeterPoint(nvec_next_pt, false);
      Vert *normal_prev = new Vert(nvec_prev_pt);
      Vert *normal_next = new Vert(nvec_next_pt);

      perimeter_left_side.push_back(normal_prev);
      perimeter_right_side.push_back(normal_next);

      // BLI_addtail(perimeter_left_side, normal_prev);
      // BLI_addtail(perimeter_right_side, normal_next);
      num_perimeter_points += 2;
    }
    else {
      /* bend to the left */
      if (angle < 0.0) {
        // normalize_v2_length(nvec_prev, radius);
        // normalize_v2_length(nvec_next, radius);
        nvec_prev.normalize(r);
        nvec_next.normalize(r);

        // copy_v3_v3(nvec_prev_pt, curr_pt);
        // add_v2_v2(nvec_prev_pt, nvec_prev);

        // copy_v3_v3(nvec_next_pt, curr_pt);
        // add_v2_v2(nvec_next_pt, nvec_next);

        double2 nvec_prev_pt = curr_pt + nvec_prev;
        double2 nvec_next_pt = curr_pt + nvec_next;

        // tPerimeterPoint *normal_prev = new tPerimeterPoint(nvec_prev_pt, true);
        // tPerimeterPoint *normal_next = new tPerimeterPoint(nvec_next_pt, true);
        Vert *normal_prev = new Vert(nvec_prev_pt);
        Vert *normal_next = new Vert(nvec_next_pt);

        // BLI_addtail(perimeter_left_side, normal_prev);
        // BLI_addtail(perimeter_left_side, normal_next);
        perimeter_left_side.push_back(normal_prev);
        VertList::iterator it_prev = perimeter_left_side.end();
        perimeter_left_side.push_back(normal_next);
        VertList::iterator it_next = perimeter_left_side.end();

        num_perimeter_points += 2;

        num_perimeter_points += generate_arc_from_point_to_point(
            perimeter_left_side, it_prev, it_next, curr_pt, subdivisions, true, true);

        double2 miter_right_pt;
        if (miter_length < prev_length && miter_length < next_length) {
          miter_right_pt = curr_pt + vec_miter_right;
          // copy_v3_v3(miter_right_pt, curr_pt);
          // add_v2_v2(miter_right_pt, vec_miter_right);
        }
        else {
          miter_right_pt = curr_pt + double2::invert(nvec_next);
          // copy_v3_v3(miter_right_pt, curr_pt);
          // negate_v2(nvec_next);
          // add_v2_v2(miter_right_pt, nvec_next);
        }

        // tPerimeterPoint *miter_right = new tPerimeterPoint(miter_right_pt, false);
        // BLI_addtail(perimeter_right_side, miter_right);
        // num_perimeter_points++;

        Vert *miter_right = new Vert(miter_right_pt);
        perimeter_right_side.push_back(miter_right);

        num_perimeter_points++;
      }
      /* bend to the right */
      else {
        // normalize_v2_length(nvec_prev, -radius);
        // normalize_v2_length(nvec_next, -radius);
        nvec_prev.normalize(-r);
        nvec_next.normalize(-r);

        // copy_v3_v3(nvec_prev_pt, curr_pt);
        // add_v2_v2(nvec_prev_pt, nvec_prev);

        // copy_v3_v3(nvec_next_pt, curr_pt);
        // add_v2_v2(nvec_next_pt, nvec_next);

        double2 nvec_prev_pt = curr_pt + nvec_prev;
        double2 nvec_next_pt = curr_pt + nvec_next;

        // tPerimeterPoint *normal_prev = new tPerimeterPoint(nvec_prev_pt, false);
        // tPerimeterPoint *normal_next = new tPerimeterPoint(nvec_next_pt, false);
        Vert *normal_prev = new Vert(nvec_prev_pt);
        Vert *normal_next = new Vert(nvec_next_pt);

        // BLI_addtail(perimeter_right_side, normal_prev);
        // BLI_addtail(perimeter_right_side, normal_next);
        perimeter_right_side.push_back(normal_prev);
        VertList::iterator it_prev = perimeter_right_side.end();
        perimeter_right_side.push_back(normal_next);
        VertList::iterator it_next = perimeter_right_side.end();

        num_perimeter_points += 2;

        num_perimeter_points += generate_arc_from_point_to_point(
            perimeter_right_side, it_prev, it_next, curr_pt, subdivisions, false, false);

        double2 miter_left_pt;
        if (miter_length < prev_length && miter_length < next_length) {
          miter_left_pt = curr_pt + vec_miter_left;
          // copy_v3_v3(miter_left_pt, curr_pt);
          // add_v2_v2(miter_left_pt, vec_miter_left);
        }
        else {
          miter_left_pt = curr_pt + double2::invert(nvec_prev);
          // copy_v3_v3(miter_left_pt, curr_pt);
          // negate_v2(nvec_prev);
          // add_v2_v2(miter_left_pt, nvec_prev);
        }

        // tPerimeterPoint *miter_left = new tPerimeterPoint(miter_left_pt, true);
        // BLI_addtail(perimeter_left_side, miter_left);
        // num_perimeter_points++;

        Vert *miter_left = new Vert(miter_left_pt);
        perimeter_left_side.push_back(miter_left);

        num_perimeter_points++;
      }
    }
  }

  /* generate points for end cap */
  num_perimeter_points += generate_perimeter_cap(
      perimeter_right_side, last_pt, last_prev_pt, last_radius, subdivisions, end_cap_t, false);

  /* merge both sides to one list */
  // BLI_listbase_reverse(perimeter_right_side);
  // BLI_movelisttolist(perimeter_left_side,
  //                    perimeter_right_side);  // perimeter_left_side contains entire list
  // ListBase *perimeter_list = perimeter_left_side;

  perimeter_right_side.reverse();
  perimeter_left_side.push_back(perimeter_right_side);
  VertList perimeter_list = VertList(perimeter_left_side);

  /* close by creating a point close to the first (make a small gap) */
  // float close_pt[3];
  // tPerimeterPoint *close_first = (tPerimeterPoint *)perimeter_list->first;
  // tPerimeterPoint *close_last = (tPerimeterPoint *)perimeter_list->last;
  // interp_v3_v3v3(close_pt, &close_last->x, &close_first->x, 0.99f);
  Vert *close_first = perimeter_list.front();
  Vert *close_last = perimeter_list.back();
  double2 close_pt = double2::interpolate(close_first->co, close_last->co, 0.99f);

  // if (compare_v3v3(close_pt, &close_first->x, FLT_EPSILON) == false) {
  //   tPerimeterPoint *close_p_pt = new tPerimeterPoint(close_pt, true);
  //   BLI_addtail(perimeter_list, close_p_pt);
  //   num_perimeter_points++;
  // }
  if (double2::compare(close_pt, close_first->co, DBL_EPSILON) == false) {
    Vert *close_p_pt = new Vert(close_pt);
    perimeter_list.push_back(close_p_pt);
    num_perimeter_points++;
  }

  /* free temp data */
  // BLI_freelistN(perimeter_right_side);
  // MEM_freeN(perimeter_right_side);

  // *r_num_perimeter_points = num_perimeter_points;
  // return perimeter_list;

  return Polyline(perimeter_list);
}

} /* namespace blender::polyclip */

/* Wrapper for C. */
extern "C" {

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

  blender::polyclip::Polyline offset_pline = polyline_offset_intern(
      pline,
      subdivisions,
      radius,
      static_cast<blender::polyclip::CapType>(start_cap_t),
      static_cast<blender::polyclip::CapType>(end_cap_t));

  double *offset_verts = NULL;
  uint num_offset_verts = offset_pline.num_verts;

  *r_offset_verts = offset_verts;
  *r_num_offset_verts = num_offset_verts;
}

} /* extern "C" */