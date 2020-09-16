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

namespace blender::polyclip {

static int generate_arc_from_point_to_point(Polyline *pline,
                                            double2 from,
                                            double2 to,
                                            ulong64 idx_from,
                                            ulong64 idx_to,
                                            double2 center_pt,
                                            int subdivisions,
                                            bool clockwise,
                                            bool is_left)
{
  // float vec_from[2];
  // float vec_to[2];
  // sub_v2_v2v2(vec_from, &from->x, center_pt);
  // sub_v2_v2v2(vec_to, &to->x, center_pt);
  double2 vec_from = from - center_pt;
  double2 vec_to = to - center_pt;

  // if (is_zero_v2(vec_from) || is_zero_v2(vec_to)) {
  //   return 0;
  // }

  if (vec_from.is_zero() || vec_to.is_zero()) {
    return 0;
  }

  // float dot = dot_v2v2(vec_from, vec_to);
  // float det = cross_v2v2(vec_from, vec_to);
  // float angle = clockwise ? M_PI - atan2f(-det, -dot) : atan2f(-det, -dot) + M_PI;

  double dot = double2::dot(vec_from, vec_to);
  double det = double2::cross(vec_from, vec_to);
  double angle = clockwise ? M_PI - atan2(-det, -dot) : atan2(-det, -dot) + M_PI;

  /* number of points is 2^(n+1) + 1 on half a circle (n=subdivisions)
   * so we multiply by (angle / pi) to get the right amount of
   * points to insert */
  // int num_points = (int)(((1 << (subdivisions + 1)) - 1) * (angle / M_PI));
  int num_points = (int)((subdivisions + 3) * (angle / M_PI));
  if (num_points > 0) {
    double angle_incr = angle / (double)num_points;

    ulong64 idx_last;
    double2 vec_t;
    if (clockwise) {
      idx_last = idx_to;
      vec_t = double2(vec_to);
    }
    else {
      idx_last = idx_from;
      vec_t = double2(vec_from);
    }

    double2 vec_p;
    for (int i = 0; i < num_points - 1; i++) {
      double tmp_angle = (i + 1) * angle_incr;

      // rotate_v2_v2fl(vec_p, vec_t, tmp_angle);
      // add_v2_v2(vec_p, center_pt);
      // vec_p[2] = center_pt[2];

      vec_p = double2::rotate(vec_t, tmp_angle);
      vec_p += center_pt;

      if (clockwise) {
        pline->verts.insert(pline->verts.begin() + idx_last, Vert(vec_p));
      }
      else {
      }

      // tPerimeterPoint *new_point = new_perimeter_point(vec_p, is_left);
      // if (clockwise) {
      //   BLI_insertlinkbefore(list, last_point, new_point);
      // }
      // else {
      //   BLI_insertlinkafter(list, last_point, new_point);
      // }

      // last_point = new_point;
    }

    return num_points - 1;
  }

  return 0;
}

static int generate_semi_circle_from_point_to_point(
    ListBase *list, tPerimeterPoint *from, tPerimeterPoint *to, int subdivisions, bool is_left)
{
  int num_points = (1 << (subdivisions + 1)) + 1;
  float center_pt[3];
  interp_v3_v3v3(center_pt, &from->x, &to->x, 0.5f);

  float vec_center[2];
  sub_v2_v2v2(vec_center, &from->x, center_pt);
  if (is_zero_v2(vec_center)) {
    return 0;
  }

  float vec_p[3];  // temp vector to do the vector math
  float angle_incr = M_PI / ((float)num_points - 1);

  tPerimeterPoint *last_point = from;
  for (int i = 1; i < num_points; i++) {
    float angle = i * angle_incr;

    /* rotate vector around point to get perimeter points */
    rotate_v2_v2fl(vec_p, vec_center, angle);
    add_v2_v2(vec_p, center_pt);
    vec_p[2] = center_pt[2];

    tPerimeterPoint *new_point = new_perimeter_point(vec_p, is_left);
    BLI_insertlinkafter(list, last_point, new_point);

    last_point = new_point;
  }

  return num_points - 1;
}

static int generate_perimeter_cap(const float point[4],
                                  const float other_point[4],
                                  float radius,
                                  ListBase *list,
                                  int subdivisions,
                                  short cap_type,
                                  bool is_left)
{
  float cap_vec[2];
  sub_v2_v2v2(cap_vec, other_point, point);
  normalize_v2(cap_vec);

  float cap_nvec[2];
  if (is_zero_v2(cap_vec)) {
    cap_nvec[0] = 0;
    cap_nvec[1] = radius;
  }
  else {
    cap_nvec[0] = -cap_vec[1];
    cap_nvec[1] = cap_vec[0];
    mul_v2_fl(cap_nvec, radius);
  }
  float cap_nvec_inv[2];
  negate_v2_v2(cap_nvec_inv, cap_nvec);

  float vec_perimeter[3];
  copy_v3_v3(vec_perimeter, point);
  add_v2_v2(vec_perimeter, cap_nvec);

  float vec_perimeter_inv[3];
  copy_v3_v3(vec_perimeter_inv, point);
  add_v2_v2(vec_perimeter_inv, cap_nvec_inv);

  tPerimeterPoint *p_pt = new_perimeter_point(vec_perimeter, is_left);
  tPerimeterPoint *p_pt_inv = new_perimeter_point(vec_perimeter_inv, is_left);

  BLI_addtail(list, p_pt);
  BLI_addtail(list, p_pt_inv);

  int num_points = 0;
  if (cap_type == CAP_ROUND) {
    num_points += generate_semi_circle_from_point_to_point(
        list, p_pt, p_pt_inv, subdivisions, is_left);
  }

  return num_points + 2;
}

/**
 * Calculate the perimeter (outline) of a stroke as list of tPerimeterPoint.
 * \param subdivisions: Number of subdivions for the start and end caps
 * \return: list of tPerimeterPoint
 */
static ListBase *gpencil_stroke_perimeter_ex(const bGPdata *gpd,
                                             const bGPDlayer *gpl,
                                             const bGPDstroke *gps,
                                             int subdivisions,
                                             int *r_num_perimeter_points)
{
  /* sanity check */
  if (gps->totpoints < 1) {
    return NULL;
  }

  float defaultpixsize = 1000.0f / gpd->pixfactor;
  float stroke_radius = ((gps->thickness + gpl->line_change) / defaultpixsize) / 2.0f;

  ListBase *perimeter_right_side = MEM_callocN(sizeof(ListBase), __func__);
  ListBase *perimeter_left_side = MEM_callocN(sizeof(ListBase), __func__);
  int num_perimeter_points = 0;

  bGPDspoint *first = &gps->points[0];
  bGPDspoint *last = &gps->points[gps->totpoints - 1];

  float first_radius = stroke_radius * first->pressure;
  float last_radius = stroke_radius * last->pressure;

  bGPDspoint *first_next;
  bGPDspoint *last_prev;
  if (gps->totpoints > 1) {
    first_next = &gps->points[1];
    last_prev = &gps->points[gps->totpoints - 2];
  }
  else {
    first_next = first;
    last_prev = last;
  }

  float first_pt[3];
  float last_pt[3];
  float first_next_pt[3];
  float last_prev_pt[3];
  copy_v3_v3(first_pt, &first->x);
  copy_v3_v3(last_pt, &last->x);
  copy_v3_v3(first_next_pt, &first_next->x);
  copy_v3_v3(last_prev_pt, &last_prev->x);

  /* edgecase if single point */
  if (gps->totpoints == 1) {
    first_next_pt[0] += 1.0f;
    last_prev_pt[0] -= 1.0f;
  }

  /* generate points for start cap */
  num_perimeter_points += generate_perimeter_cap(first_pt,
                                                 first_next_pt,
                                                 first_radius,
                                                 perimeter_right_side,
                                                 subdivisions,
                                                 gps->caps[0],
                                                 false);

  /* generate perimeter points  */
  float curr_pt[3], next_pt[3], prev_pt[3];
  float vec_next[2], vec_prev[2];
  float nvec_next[2], nvec_prev[2];
  float nvec_next_pt[3], nvec_prev_pt[3];
  float vec_tangent[2];

  float vec_miter_left[2], vec_miter_right[2];
  float miter_left_pt[3], miter_right_pt[3];

  for (int i = 1; i < gps->totpoints - 1; i++) {
    bGPDspoint *curr = &gps->points[i];
    bGPDspoint *prev = &gps->points[i - 1];
    bGPDspoint *next = &gps->points[i + 1];
    float radius = stroke_radius * curr->pressure;

    copy_v3_v3(curr_pt, &curr->x);
    copy_v3_v3(next_pt, &next->x);
    copy_v3_v3(prev_pt, &prev->x);

    sub_v2_v2v2(vec_prev, curr_pt, prev_pt);
    sub_v2_v2v2(vec_next, next_pt, curr_pt);
    float prev_length = len_v2(vec_prev);
    float next_length = len_v2(vec_next);

    if (normalize_v2(vec_prev) == 0.0f) {
      vec_prev[0] = 1.0f;
      vec_prev[1] = 0.0f;
    }
    if (normalize_v2(vec_next) == 0.0f) {
      vec_next[0] = 1.0f;
      vec_next[1] = 0.0f;
    }

    nvec_prev[0] = -vec_prev[1];
    nvec_prev[1] = vec_prev[0];

    nvec_next[0] = -vec_next[1];
    nvec_next[1] = vec_next[0];

    add_v2_v2v2(vec_tangent, vec_prev, vec_next);
    if (normalize_v2(vec_tangent) == 0.0f) {
      copy_v2_v2(vec_tangent, nvec_prev);
    }

    vec_miter_left[0] = -vec_tangent[1];
    vec_miter_left[1] = vec_tangent[0];

    /* calculate miter length */
    float an1 = dot_v2v2(vec_miter_left, nvec_prev);
    if (an1 == 0.0f) {
      an1 = 1.0f;
    }
    float miter_length = radius / an1;
    if (miter_length <= 0.0f) {
      miter_length = 0.01f;
    }

    normalize_v2_length(vec_miter_left, miter_length);

    copy_v2_v2(vec_miter_right, vec_miter_left);
    negate_v2(vec_miter_right);

    float angle = dot_v2v2(vec_next, nvec_prev);
    /* add two points if angle is close to beeing straight */
    if (fabsf(angle) < 0.0001f) {
      normalize_v2_length(nvec_prev, radius);
      normalize_v2_length(nvec_next, radius);

      copy_v3_v3(nvec_prev_pt, curr_pt);
      add_v2_v2(nvec_prev_pt, nvec_prev);

      copy_v3_v3(nvec_next_pt, curr_pt);
      negate_v2(nvec_next);
      add_v2_v2(nvec_next_pt, nvec_next);

      tPerimeterPoint *normal_prev = new_perimeter_point(nvec_prev_pt, true);
      tPerimeterPoint *normal_next = new_perimeter_point(nvec_next_pt, false);

      BLI_addtail(perimeter_left_side, normal_prev);
      BLI_addtail(perimeter_right_side, normal_next);
      num_perimeter_points += 2;
    }
    else {
      /* bend to the left */
      if (angle < 0.0f) {
        normalize_v2_length(nvec_prev, radius);
        normalize_v2_length(nvec_next, radius);

        copy_v3_v3(nvec_prev_pt, curr_pt);
        add_v2_v2(nvec_prev_pt, nvec_prev);

        copy_v3_v3(nvec_next_pt, curr_pt);
        add_v2_v2(nvec_next_pt, nvec_next);

        tPerimeterPoint *normal_prev = new_perimeter_point(nvec_prev_pt, true);
        tPerimeterPoint *normal_next = new_perimeter_point(nvec_next_pt, true);

        BLI_addtail(perimeter_left_side, normal_prev);
        BLI_addtail(perimeter_left_side, normal_next);
        num_perimeter_points += 2;

        num_perimeter_points += generate_arc_from_point_to_point(
            perimeter_left_side, normal_prev, normal_next, curr_pt, subdivisions, true, true);

        if (miter_length < prev_length && miter_length < next_length) {
          copy_v3_v3(miter_right_pt, curr_pt);
          add_v2_v2(miter_right_pt, vec_miter_right);
        }
        else {
          copy_v3_v3(miter_right_pt, curr_pt);
          negate_v2(nvec_next);
          add_v2_v2(miter_right_pt, nvec_next);
        }

        tPerimeterPoint *miter_right = new_perimeter_point(miter_right_pt, false);
        BLI_addtail(perimeter_right_side, miter_right);
        num_perimeter_points++;
      }
      /* bend to the right */
      else {
        normalize_v2_length(nvec_prev, -radius);
        normalize_v2_length(nvec_next, -radius);

        copy_v3_v3(nvec_prev_pt, curr_pt);
        add_v2_v2(nvec_prev_pt, nvec_prev);

        copy_v3_v3(nvec_next_pt, curr_pt);
        add_v2_v2(nvec_next_pt, nvec_next);

        tPerimeterPoint *normal_prev = new_perimeter_point(nvec_prev_pt, false);
        tPerimeterPoint *normal_next = new_perimeter_point(nvec_next_pt, false);

        BLI_addtail(perimeter_right_side, normal_prev);
        BLI_addtail(perimeter_right_side, normal_next);
        num_perimeter_points += 2;

        num_perimeter_points += generate_arc_from_point_to_point(
            perimeter_right_side, normal_prev, normal_next, curr_pt, subdivisions, false, false);

        if (miter_length < prev_length && miter_length < next_length) {
          copy_v3_v3(miter_left_pt, curr_pt);
          add_v2_v2(miter_left_pt, vec_miter_left);
        }
        else {
          copy_v3_v3(miter_left_pt, curr_pt);
          negate_v2(nvec_prev);
          add_v2_v2(miter_left_pt, nvec_prev);
        }

        tPerimeterPoint *miter_left = new_perimeter_point(miter_left_pt, true);
        BLI_addtail(perimeter_left_side, miter_left);
        num_perimeter_points++;
      }
    }
  }

  /* generate points for end cap */
  num_perimeter_points += generate_perimeter_cap(
      last_pt, last_prev_pt, last_radius, perimeter_right_side, subdivisions, gps->caps[1], false);

  /* merge both sides to one list */
  BLI_listbase_reverse(perimeter_right_side);
  BLI_movelisttolist(perimeter_left_side,
                     perimeter_right_side);  // perimeter_left_side contains entire list
  ListBase *perimeter_list = perimeter_left_side;

  /* close by creating a point close to the first (make a small gap) */
  float close_pt[3];
  tPerimeterPoint *close_first = (tPerimeterPoint *)perimeter_list->first;
  tPerimeterPoint *close_last = (tPerimeterPoint *)perimeter_list->last;
  interp_v3_v3v3(close_pt, &close_last->x, &close_first->x, 0.99f);

  if (compare_v3v3(close_pt, &close_first->x, FLT_EPSILON) == false) {
    tPerimeterPoint *close_p_pt = new_perimeter_point(close_pt, true);
    BLI_addtail(perimeter_list, close_p_pt);
    num_perimeter_points++;
  }

  /* free temp data */
  BLI_freelistN(perimeter_right_side);
  MEM_freeN(perimeter_right_side);

  *r_num_perimeter_points = num_perimeter_points;
  return perimeter_list;
}

} /* namespace blender::polyclip */