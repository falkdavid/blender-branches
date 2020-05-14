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
 *
 * The Original Code is Copyright (C) 2008, Blender Foundation
 * This is a new part of Blender
 */

/** \file
 * \ingroup bke
 */

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CLG_log.h"

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_heap.h"
#include "BLI_math_vector.h"
#include "BLI_wavl_tree.h"

#include "BLT_translation.h"

#include "DNA_gpencil_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"

#include "BKE_context.h"
#include "BKE_gpencil.h"
#include "BKE_gpencil_clip.h"
#include "BKE_gpencil_geom.h"
#include "BKE_main.h"

#include "DEG_depsgraph.h"

#define ISECT_LEFT 0
#define ISECT_RIGHT 1

enum {
  BRUTE_FORCE = 1,
  BRUTE_FORCE_WITH_AABB,
  BENTLEY_OTTMANN,
  BENTLEY_OTTMANN_WITH_AABB,
};

/* -------------------------------------------------------------------- */
/** \name Temporary data structures
 * \{ */

/**
 * Point 3d datastructure for clipping.
 */
typedef struct tClipPoint {
  /* Listbase functonality */
  struct tClipPoint *next, *prev;

  /* Coordinates
   * Note: although all algorithms on clip points only use x and y, the clip points are
   * represented in 3D view space to make transition between 2D and 3D easier */
  float x, y, z;

  /* Link to other intersection point
   * NULL if point is not an intersection
   *
   * [..]--[p1]--[p2]--[p3]--[..]  clipPath p
   *               | <- isect_link
   * [..]--[q1]--[q2]--[q3]--[..]  clipPath q
   */
  struct tClipPoint *isect_link;

  /* temporary value between 0 and 1 that represents the
   * normalized distance between the start of the edge and the intersection */
  float isect_dist;

  /*
   *    ^         ^
   * <--+---   ---+-->
   *    |         |
   *
   * 0 = left; 1 = right */
  bool isect_type;

  /* temporary flag */
  int flag;
} tClipPoint;

/* tClipPoint->flag */
typedef enum tClipPointFlag {
  CP_UNVISITED = 0,
  CP_VISITED = (1 << 0),
  CP_CHECKED = (1 << 1),
  CP_OUTER_EDGE = (1 << 2),
  CP_INNER_EDGE = (1 << 3),
} tClipPointFlag;

/**
 * Edge datastructure for clipping.
 */
typedef struct tClipEdge {
  /* Listbase functonality */
  struct tClipEdge *next, *prev;
  /* pointers to start and end */
  struct tClipPoint *start, *end;
  /* bounding box (min_x, max_x, min_y, max_y) */
  float aabb[4];
} tClipEdge;

/**
 * A polygonal chain datastructure for clipping.
 */
typedef struct tClipPath {
  /* Listbase functonality */
  struct tClipPath *next, *prev;
  /* list of tClipPoint */
  ListBase points;
  /* list of tClipEdge */
  ListBase edges;
  /* number of points and edges */
  int num_points, num_edges;
  /* bounding box (min_x, max_x, min_y, max_y) */
  float aabb[4];
} tClipPath;

/** \} */

/* -------------------------------------------------------------------- */
/** \name Utility functions
 * \{ */

/* axis aligned bounding box intersection check */
static bool isect_aabb_aabb_v4(const float a[4], const float b[4])
{
  return (a[0] < b[1] && a[2] < b[3] && b[0] < a[1] && b[2] < a[3]);
}

/* Given that A and B intersect, returns the intersection type (see tClipPoint) */
static bool gp_clip_point_isect_type(const tClipEdge *A, const tClipEdge *B)
{
  double a_start[2], a_end[2];
  double b_end[2];
  double v1[2], v2[2];
  copy_v2db_v2fl(a_start, &A->start->x);
  copy_v2db_v2fl(a_end, &A->end->x);
  copy_v2db_v2fl(b_end, &B->end->x);
  sub_v2_v2v2_db(v1, a_end, a_start);
  sub_v2_v2v2_db(v2, b_end, a_start);
  /* 0 = left; 1 = right */
  return (-v1[0] * v2[1] + v1[1] * v2[0]) > 0.0;
}

static void free_clip_path(tClipPath *path)
{
  if (path == NULL) {
    return;
  }

  BLI_freelistN(&path->edges);
  BLI_freelistN(&path->points);
  MEM_freeN(path);
}

static void gp_update_clip_edge_aabb(tClipEdge *edge)
{
  edge->aabb[0] = edge->start->x < edge->end->x ? edge->start->x : edge->end->x;
  edge->aabb[1] = edge->start->x > edge->end->x ? edge->start->x : edge->end->x;
  edge->aabb[2] = edge->start->y < edge->end->y ? edge->start->y : edge->end->y;
  edge->aabb[3] = edge->start->y > edge->end->y ? edge->start->y : edge->end->y;
}

/* creates and returns a new edge from A to B */
static tClipEdge *gp_new_clip_edge_from_clip_points(tClipPoint *A, tClipPoint *B)
{
  tClipEdge *new_edge = MEM_callocN(sizeof(tClipEdge), __func__);
  new_edge->start = A;
  new_edge->end = B;

  gp_update_clip_edge_aabb(new_edge);

  return new_edge;
}

/* Helper: create edge list for clip path from the points */
static void gp_create_clip_edges_clip_path(tClipPath *path)
{
  if (path->num_edges > 0) {
    BLI_freelistN(&path->edges);
    path->num_edges = 0;
  }

  tClipPoint *cpt_prev = path->points.first;
  for (tClipPoint *cpt_curr = cpt_prev->next; cpt_curr != NULL;
       cpt_prev = cpt_curr, cpt_curr = cpt_curr->next) {
    tClipEdge *new_edge = gp_new_clip_edge_from_clip_points(cpt_prev, cpt_curr);
    BLI_addtail(&path->edges, new_edge);
    path->num_edges++;
  }
}

/* Helper: if we inserted new points on the path, the edges are no longer in sync and need updating
 * for further use */
static void gp_update_clip_edges_clip_path(tClipPath *path)
{
  tClipEdge *curr_ceg = path->edges.first;
  BLI_assert(curr_ceg->start == path->points.first);

  int num_new_clip_edges = 0;
  LISTBASE_FOREACH (tClipPoint *, curr_cpt, &path->points) {
    if (curr_cpt == path->points.last) {
      break;
    }

    tClipPoint *next_cpt = curr_cpt->next;
    if (curr_ceg->end != next_cpt) {
      /* split the edge */
      tClipEdge *new_ceg = gp_new_clip_edge_from_clip_points(curr_cpt, next_cpt);
      BLI_insertlinkbefore(&path->edges, curr_ceg, new_ceg);
      num_new_clip_edges++;

      curr_ceg->start = next_cpt;
      gp_update_clip_edge_aabb(curr_ceg);
    }
    else {
      curr_ceg = curr_ceg->next;
    }
  }

  path->num_edges += num_new_clip_edges;
}

/* updates the aabb of path to include point */
static void gp_update_clip_path_aabb(tClipPath *path, tClipPoint *point)
{
  path->aabb[0] = point->x < path->aabb[0] ? point->x : path->aabb[0];
  path->aabb[1] = point->x > path->aabb[1] ? point->x : path->aabb[1];
  path->aabb[2] = point->y < path->aabb[2] ? point->y : path->aabb[2];
  path->aabb[3] = point->y > path->aabb[3] ? point->y : path->aabb[3];
}

/**
 * Helper: intersect edga A and edge B, create two intersection points, link them and insert the
 * two points into clip point list
 */
static tClipPoint *gp_isect_clip_edges(ListBase *clip_points, tClipEdge *edgeA, tClipEdge *edgeB)
{
  float *p0_a = &edgeA->start->x;
  float *p1_a = &edgeA->end->x;
  float *p0_b = &edgeB->start->x;
  float *p1_b = &edgeB->end->x;

  float isect_pt[2];
  int status = isect_seg_seg_v2_point_ex(p0_a, p1_a, p0_b, p1_b, -1e-6f, isect_pt);
  if (status == 1) {
    tClipPoint *cpt_isectA = MEM_callocN(sizeof(tClipPoint), __func__);
    copy_v2_v2(&cpt_isectA->x, isect_pt);
    cpt_isectA->isect_type = gp_clip_point_isect_type(edgeA, edgeB);

    tClipPoint *cpt_isectB = MEM_dupallocN(cpt_isectA);
    cpt_isectB->isect_type = !cpt_isectA->isect_type;

    cpt_isectA->z = edgeA->start->z;
    cpt_isectA->isect_dist = len_v2v2(p0_a, p1_a) / len_v2v2(p0_a, isect_pt);
    cpt_isectB->z = edgeB->start->z;
    cpt_isectB->isect_dist = len_v2v2(p0_b, p1_b) / len_v2v2(p0_b, isect_pt);

    cpt_isectA->isect_link = cpt_isectB;
    cpt_isectB->isect_link = cpt_isectA;

    /* insert intersection point at the right place */
    tClipPoint *cpt_insert = edgeA->start;
    while (cpt_insert->next != edgeA->end &&
           cpt_insert->next->isect_dist > cpt_isectA->isect_dist) {
      cpt_insert = cpt_insert->next;
    }
    BLI_insertlinkafter(clip_points, cpt_insert, cpt_isectA);

    cpt_insert = edgeB->start;
    while (cpt_insert->next != edgeB->end &&
           cpt_insert->next->isect_dist > cpt_isectB->isect_dist) {
      cpt_insert = cpt_insert->next;
    }
    BLI_insertlinkafter(clip_points, cpt_insert, cpt_isectB);

    return cpt_isectA;
  }

  return NULL;
}

/* Helper: create and return a clip path from a gp stroke */
static tClipPath *gp_clip_path_from_stroke(bGPDstroke *gps)
{
  tClipPath *new_path = MEM_callocN(sizeof(tClipPath), __func__);
  float init_aabb[4] = {FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX};
  copy_v4_v4(new_path->aabb, init_aabb);

  tClipPoint *cpt_first = MEM_callocN(sizeof(tClipPoint), __func__);
  bGPDspoint *pt_first = &gps->points[0];
  copy_v3_v3(&cpt_first->x, &pt_first->x);
  BLI_addtail(&new_path->points, cpt_first);
  new_path->num_points++;

  tClipPoint *cpt_prev = cpt_first;
  for (int i = 1; i < gps->totpoints; i++) {
    tClipPoint *cpt_curr = MEM_callocN(sizeof(tClipPoint), __func__);

    bGPDspoint *pt = &gps->points[i];
    copy_v3_v3(&cpt_curr->x, &pt->x);
    BLI_addtail(&new_path->points, cpt_curr);
    new_path->num_points++;

    tClipEdge *new_edge = gp_new_clip_edge_from_clip_points(cpt_prev, cpt_curr);
    BLI_addtail(&new_path->edges, new_edge);
    new_path->num_edges++;

    gp_update_clip_path_aabb(new_path, cpt_curr);
    cpt_prev = cpt_curr;
  }

  return new_path;
}

/* Helper: create and return a gp stroke from a clip path */
static bGPDstroke *gp_stroke_from_clip_path(tClipPath *path, int mat_idx, bool select)
{
  bGPDstroke *clip_stroke = BKE_gpencil_stroke_new(mat_idx, path->num_points, 1);

  tClipPoint *cpt_curr = path->points.first;
  for (int i = 0; i < path->num_points; i++) {
    bGPDspoint *pt = &clip_stroke->points[i];
    copy_v3_v3(&pt->x, &cpt_curr->x);

    /* Set pressure to zero and strength to one */
    pt->pressure = 0.0f;
    pt->strength = 1.0f;

    if (select) {
      pt->flag |= GP_SPOINT_SELECT;
    }

#if 0
    if (cpt_curr->flag & CP_OUTER_EDGE) {
      float red[4] = {1.0, 0.0, 0.0, 1.0};
      copy_v4_v4(pt->vert_color, red);
    }
    if (cpt_curr->flag & CP_INNER_EDGE) {
      float blue[4] = {0.0, 0.0, 1.0, 1.0};
      copy_v4_v4(pt->vert_color, blue);
    }
#endif

    cpt_curr = cpt_curr->next;
  }

  /* triangles cache needs to be recalculated */
  BKE_gpencil_stroke_geometry_update(clip_stroke);

  clip_stroke->flag |= GP_STROKE_CYCLIC;
  if (select) {
    clip_stroke->flag |= GP_STROKE_SELECT;
  }

  return clip_stroke;
}

static tClipPath *gp_stroke_to_view_space_clip_path(const bContext *C,
                                                    const bGPDlayer *gpl,
                                                    bGPDstroke *gps)
{
  BKE_gpencil_stroke_to_view_space(C, gpl, gps);
  tClipPath *clip_path = gp_clip_path_from_stroke(gps);
  return clip_path;
}

static bGPDstroke *gp_view_space_clip_path_to_stroke(
    const bContext *C, const bGPDlayer *gpl, tClipPath *path, int mat_idx, bool select)
{
  bGPDstroke *gps = gp_stroke_from_clip_path(path, mat_idx, select);
  BKE_gpencil_stroke_from_view_space(C, gpl, gps);
  return gps;
}

static void debug_print_clip_point(tClipPoint *cp)
{
  printf("tClipPoint: %p\n", cp);
  printf("x: %f, y: %f, z: %f\n", cp->x, cp->y, cp->z);
  if (cp->isect_link != NULL) {
    if (cp->isect_type == 0) {
      printf("isect type: left\n");
    }
    else {
      printf("isect type: right\n");
    }
  }
  else {
    printf("isect type: None\n");
  }
  printf("flag: ");
  if (cp->flag == CP_UNVISITED)
    printf("CP_UNVISITED ");
  if (cp->flag & CP_VISITED)
    printf("CP_VISITED ");
  if (cp->flag & CP_CHECKED)
    printf("CP_CHECKED ");
  if (cp->flag & CP_OUTER_EDGE)
    printf("CP_OUTER_EDGE ");
  if (cp->flag & CP_INNER_EDGE)
    printf("CP_INNER_EDGE ");
  printf("\n");
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Clip path utility functions
 * \{ */

static tClipPoint *gp_clip_path_find_min_x_point(tClipPath *clip_path)
{
  tClipPoint *cpt_min = NULL;
  float min_x = FLT_MAX;
  LISTBASE_FOREACH (tClipPoint *, curr, &clip_path->points) {
    if (curr->x < min_x) {
      min_x = curr->x;
      cpt_min = curr;
    }
  }
  return cpt_min;
}

static tClipPath *gp_clip_path_get_outer_edge(tClipPath *clip_path)
{
  /* outer edge path */
  tClipPath *new_path = MEM_callocN(sizeof(tClipPath), __func__);
  tClipPoint *cpt_min = gp_clip_path_find_min_x_point(clip_path);
  BLI_listbase_rotate_first(&clip_path->points, cpt_min);

  tClipPoint *out_start = MEM_dupallocN(cpt_min);
  BLI_addtail(&new_path->points, out_start);
  new_path->num_points++;

  tClipPoint *cpt_curr = cpt_min->next;
  while (cpt_curr != NULL) {
    tClipPoint *out_curr = MEM_dupallocN(cpt_curr);
    BLI_addtail(&new_path->points, out_curr);
    new_path->num_points++;

    if (cpt_curr->isect_link != NULL) {
      cpt_curr = cpt_curr->isect_link->next;
    }
    else {
      cpt_curr = cpt_curr->next;
    }
  }

  return new_path;
}

static tClipPoint *gp_find_best_insert_point(tClipPath *outer_edge, tClipPoint *target)
{
  tClipPoint *best_incert_cp = outer_edge->points.first;
  float best_dist = len_squared_v2v2(&best_incert_cp->x, &target->x);

  /* search clostest to the left */
  LISTBASE_FOREACH (tClipPoint *, curr, &outer_edge->points) {
    if (curr->x > target->x) {
      continue;
    }
    float dist = len_squared_v2v2(&curr->x, &target->x);
    if (dist < best_dist) {
      best_dist = dist;
      best_incert_cp = curr;
    }
  }

  return best_incert_cp;
}

static void gp_insert_hole_into_outer_edge(tClipPath *outer_edge,
                                           tClipPath *hole,
                                           tClipPoint *insert_cpt)
{
  tClipPoint *hole_first = hole->points.first;

  tClipPoint *hole_exit_start = MEM_dupallocN(hole_first);
  tClipPoint *hole_exit_end = MEM_dupallocN(insert_cpt);

  BLI_addtail(&hole->points, hole_exit_start);
  BLI_addtail(&hole->points, hole_exit_end);
  hole->num_points += 2;

  /* add hole to outer edge */
  if (insert_cpt->next != NULL) {
    tClipPoint *hole_exit_end_next = insert_cpt->next;
    insert_cpt->next = hole_first;
    hole_first->prev = insert_cpt;

    hole_exit_end->next = hole_exit_end_next;
    hole_exit_end_next->prev = hole_exit_end;
  }
  else {
    BLI_movelisttolist(&outer_edge->points, &hole->points);
  }

  outer_edge->num_points += hole->num_points;
  MEM_freeN(hole);
}

// static ListBase *gp_get_inner_edge(tClipPoint *cpt_start, int *r_num_inner_points)
// {
//   int num_points = 1;

//   ListBase *inner_edge = MEM_callocN(sizeof(ListBase), __func__);
//   LinkData *cpt_start_link = BLI_genericNodeN(cpt_start);
//   BLI_addtail(inner_edge, cpt_start_link);

//   LinkData *cpt_min_link = cpt_start_link;
//   float min_x = cpt_start->x;

//   tClipPoint *cpt_curr = cpt_start->isect_link->next;
//   while (cpt_curr != cpt_start) {
//     LinkData *cpt_curr_link = BLI_genericNodeN(cpt_curr);
//     BLI_addtail(inner_edge, cpt_curr_link);
//     cpt_curr->flag |= (CP_INNER_EDGE | CP_VISITED);

//     if (cpt_curr->x < min_x) {
//       min_x = cpt_curr->x;
//       cpt_min_link = cpt_curr_link;
//     }

//     if (cpt_curr->isect_link != NULL) {
//       cpt_curr = cpt_curr->isect_link->next;
//     }
//     else {
//       cpt_curr = cpt_curr->next;
//     }
//     num_points++;
//   }

//   if (num_points <= 2) {
//     return NULL;
//   }

//   /* turn min x point into first */
//   BLI_listbase_rotate_first(inner_edge, cpt_min_link);

//   *r_num_inner_points += num_points;
//   return inner_edge;
// }

// static tClipPoint *gp_next_isect(const tClipPoint *cpt, bool flag_visited)
// {
//   tClipPoint *cpt_curr = cpt->next;
//   while (cpt_curr->isect_link == NULL) {
//     if (flag_visited == true) {
//       cpt_curr->flag |= CP_VISITED;
//     }
//     cpt_curr = cpt_curr->next;
//   }
//   return cpt_curr;
// }

static tClipPath *gp_clip_path_get_outer_edge_with_holes(tClipPath *path)
{
  if (path == NULL || path->num_points == 0) {
    return NULL;
  }

  /* Nonzero-rule: The number of right and left intersections, with a ray, cast from a point
   * outside the path (or from inside a hole), and the path itself, should be equal. If we count up
   * for right intersections and down for left intersections, the result for points on the outside
   * should be exactly zero, meaning everything on the inside should be nonzero. */

  tClipPath *outer_edge = MEM_callocN(sizeof(tClipPath), __func__);
  tClipPoint *cpt_min = gp_clip_path_find_min_x_point(path);
  cpt_min->flag |= CP_CHECKED;
  BLI_listbase_rotate_first(&path->points, cpt_min);

  tClipPoint *out_start = MEM_dupallocN(cpt_min);
  BLI_addtail(&outer_edge->points, out_start);
  outer_edge->num_points++;

  tClipPoint *cpt_curr = cpt_min->next;
  while (cpt_curr != NULL) {
    cpt_curr->flag |= CP_CHECKED;
    tClipPoint *out_curr = MEM_dupallocN(cpt_curr);
    BLI_addtail(&outer_edge->points, out_curr);
    outer_edge->num_points++;

    if (cpt_curr->isect_link != NULL) {
      cpt_curr->isect_link->flag |= CP_CHECKED;
      cpt_curr = cpt_curr->isect_link->next;
    }
    else {
      cpt_curr = cpt_curr->next;
    }
  }

  /* mark all outer and inner edges */
  int rule_count = 0;
  int num_marked_outer = 0;
  LISTBASE_FOREACH (tClipPoint *, curr, &path->points) {
    if (curr->isect_link != NULL) {
      int prev_rule_count = rule_count;

      if (curr->isect_type == ISECT_LEFT) {
        rule_count--;
      }
      else {
        rule_count++;
      }

      if (prev_rule_count == 0 || rule_count == 0) {
        curr->flag |= CP_OUTER_EDGE;
        num_marked_outer++;
      }
      else {
        curr->flag |= CP_INNER_EDGE;
      }
    }
    if (rule_count == 0) {
      curr->flag |= CP_OUTER_EDGE;
      num_marked_outer++;
    }
    else {
      curr->flag |= CP_INNER_EDGE;
    }
  }
  free_clip_path(outer_edge);
  return path;

  LISTBASE_FOREACH (tClipPoint *, curr, &path->points) {
    /* create new path if marked as CP_OUTER_EDGE but not CP_CHECKED */
    if ((curr->flag & CP_OUTER_EDGE) && !(curr->flag & CP_CHECKED)) {
      curr->flag |= CP_CHECKED;
      tClipPath *hole = MEM_callocN(sizeof(tClipPath), __func__);

      tClipPoint *min_cpt = NULL;
      float min_x = FLT_MAX;

      cpt_curr = curr->next;
      while (cpt_curr->flag & CP_OUTER_EDGE && !(cpt_curr->flag & CP_CHECKED)) {
        cpt_curr->flag |= CP_CHECKED;
        tClipPoint *out_curr = MEM_dupallocN(cpt_curr);
        BLI_addtail(&hole->points, out_curr);
        hole->num_points++;

        if (cpt_curr->x < min_x) {
          min_x = cpt_curr->x;
          min_cpt = out_curr;
        }

        if (cpt_curr->isect_link != NULL) {
          cpt_curr->isect_link->flag |= CP_CHECKED;
          cpt_curr = cpt_curr->isect_link->next;
        }
        else {
          cpt_curr = cpt_curr->next;
        }
      }

      if (hole->num_points == 0) {
        MEM_freeN(hole);
        continue;
      }

      BLI_listbase_rotate_first(&hole->points, min_cpt);

      tClipPoint *insert_cpt = gp_find_best_insert_point(outer_edge, min_cpt);
      gp_insert_hole_into_outer_edge(outer_edge, hole, insert_cpt);
    }
  }

  return outer_edge;
}

// static ListBase *gp_clipPointList_to_outline_and_holes_list(ListBase *points,
//                                                             int tot_points,
//                                                             int *r_num_points)
// {
//   if (points == NULL || tot_points == 0) {
//     return NULL;
//   }
//   printf("Num tot points: %d\n", tot_points);
//   int point_count = 0;
//   ListBase outer_edge = {NULL, NULL};
//   ListBase outer_isect_points = {NULL, NULL};
//   /* sort found inner edges (holes) by their min x value */
//   Heap *inner_edges = BLI_heap_new();

//   /* Find min x point on outer edge */
//   tClipPoint *cpt_min = NULL;
//   float min_x = FLT_MAX;
//   LISTBASE_FOREACH (tClipPoint *, curr, points) {
//     if (curr->x < min_x) {
//       min_x = curr->x;
//       cpt_min = curr;
//     }
//   }
//   /* make min x point first */
//   BLI_listbase_rotate_first(points, cpt_min);

//   /* link ends together */
//   tClipPoint *cpt_first = points->first;
//   tClipPoint *cpt_last = points->last;
//   cpt_first->prev = cpt_last;
//   cpt_last->next = cpt_first;

//   /* Find outer edge */
//   int num_outer_points = 1;
//   BLI_addtail(&outer_edge, BLI_genericNodeN(cpt_first));
//   cpt_first->flag |= (CP_OUTER_EDGE | CP_VISITED);
//   if (cpt_first->isect_link != NULL) {
//     cpt_first->isect_link->flag |= (CP_OUTER_EDGE | CP_VISITED);
//     BLI_addtail(&outer_isect_points, BLI_genericNodeN(cpt_first));
//   }
//   tClipPoint *cpt_curr = cpt_first->next;
//   while (cpt_curr != cpt_first) {
//     BLI_addtail(&outer_edge, BLI_genericNodeN(cpt_curr));
//     cpt_curr->flag |= (CP_OUTER_EDGE | CP_VISITED);
//     if (cpt_curr->isect_link != NULL) {
//       cpt_curr->isect_link->flag |= (CP_OUTER_EDGE | CP_VISITED);
//       BLI_addtail(&outer_isect_points, BLI_genericNodeN(cpt_curr));
//       cpt_curr = cpt_curr->isect_link->next;
//     }
//     else {
//       cpt_curr = cpt_curr->next;
//     }
//     num_outer_points++;
//   }
//   printf("Num outer points: %d\n", num_outer_points);

//   /* Find inner edges */
//   int num_inner_points = 0;
//   /* We loop over all the outer intersection points and do a depth first search
//    * inwards to find inner edges.  */
//   LISTBASE_FOREACH (LinkData *, _curr_outer, &outer_isect_points) {
//     printf("\nNext outer intersection point!\n");
//     tClipPoint *curr_outer = (tClipPoint *)_curr_outer->data;
//     printf("curr_outer\n");
//     debug_print_clip_point(curr_outer);
//     /* get the next intersection point inside the outline */
//     tClipPoint *cpt_dfs_source = gp_next_isect(curr_outer, true);
//     printf("cpt_dfs_source\n");
//     debug_print_clip_point(cpt_dfs_source);
//     if (cpt_dfs_source->flag & CP_OUTER_EDGE || cpt_dfs_source->flag & CP_CHECKED) {
//       continue;
//     }

//     /* DFS stack with intersection points */
//     ListBase cpt_stack = {NULL, NULL};
//     BLI_addtail(&cpt_stack, BLI_genericNodeN(cpt_dfs_source));
//     int stack_size = 1;

//     /* loop list to keep track of intersection points in current loop */
//     ListBase cpt_loop = {NULL, NULL};

//     while (stack_size > 0) {
//       LinkData *curr_isect_link = cpt_stack.last;
//       tClipPoint *curr_isect = (tClipPoint *)curr_isect_link->data;
//       printf("\ncurr_isect\n");
//       debug_print_clip_point(curr_isect);

//       /* search ends if we hit the outer edge or checked node */
//       if (curr_isect->flag & CP_OUTER_EDGE || curr_isect->flag & CP_CHECKED) {
//         printf("Hit outer edge or checked!\n");
//         BLI_freelistN(&cpt_loop);
//         /* backtrack */
//         BLI_poptail(&cpt_stack);
//         stack_size--;
//         continue;
//       }

//       /* inner edge found */
//       if (curr_isect->flag & CP_VISITED &&
//           BLI_findptr(&cpt_loop, curr_isect->isect_link, offsetof(LinkData, data)) != NULL) {
//         printf("Inner edge found!\n");
//         ListBase *inner_edge = gp_get_inner_edge(curr_isect, &num_inner_points);
//         if (inner_edge != NULL) {
//           BLI_heap_insert(inner_edges, ((tClipPoint *)inner_edge->first)->x, inner_edge);
//         }

//         BLI_freelistN(&cpt_loop);
//         /* backtrack */
//         BLI_poptail(&cpt_stack);
//         stack_size--;
//         continue;
//       }

//       /* check intersection type */
//       if (curr_isect->isect_type == ISECT_LEFT) {
//         /* If the intersection points to the left, we follow it,
//          * unless it has already been visited. */
//         if (curr_isect->isect_link->next->flag & CP_VISITED) {
//           /* left has been visited so clear loop */
//           BLI_freelistN(&cpt_loop);
//           if (curr_isect->next->flag == CP_UNVISITED) {
//             printf("Left is visited, go forward!\n");
//             tClipPoint *next_isect = gp_next_isect(curr_isect, true);
//             BLI_addtail(&cpt_stack, BLI_genericNodeN(next_isect));
//             stack_size++;
//           }
//           else {
//             printf("Both visited, backtrack!\n");
//             curr_isect->flag |= (CP_CHECKED | CP_VISITED);
//             curr_isect->isect_link->flag |= (CP_CHECKED | CP_VISITED);
//             /* backtrack */
//             BLI_poptail(&cpt_stack);
//             stack_size--;
//             continue;
//           }
//         }
//         else {
//           printf("Left is unvisited, go left!\n");
//           BLI_addtail(&cpt_loop, BLI_genericNodeN(curr_isect));

//           tClipPoint *next_isect = gp_next_isect(curr_isect->isect_link, true);
//           BLI_addtail(&cpt_stack, BLI_genericNodeN(next_isect));
//           stack_size++;
//         }
//       }
//       else {
//         /* If the intersection points to the right, we continue on this path,
//          * unless it has already been visited. */
//         if (curr_isect->next->flag & CP_VISITED) {

//           BLI_freelistN(&cpt_loop);
//           if (curr_isect->isect_link->next->flag == CP_UNVISITED) {
//             printf("Forward is visited, go right!\n");
//             tClipPoint *next_isect = gp_next_isect(curr_isect->isect_link, true);
//             BLI_addtail(&cpt_stack, BLI_genericNodeN(next_isect));
//             stack_size++;
//           }
//           else {
//             printf("Both visited, backtrack!\n");
//             curr_isect->flag |= (CP_CHECKED | CP_VISITED);
//             curr_isect->isect_link->flag |= (CP_CHECKED | CP_VISITED);
//             /* backtrack */
//             BLI_poptail(&cpt_stack);
//             stack_size--;
//             continue;
//           }
//         }
//         else {
//           printf("Forward is unvisited, go forward!\n");
//           BLI_addtail(&cpt_loop, BLI_genericNodeN(curr_isect));

//           tClipPoint *next_isect = gp_next_isect(curr_isect, true);
//           BLI_addtail(&cpt_stack, BLI_genericNodeN(next_isect));
//           stack_size++;
//         }
//       }

//       curr_isect->flag |= CP_VISITED;
//       curr_isect->isect_link->flag |= CP_VISITED;
//     }
//     BLI_freelistN(&cpt_stack);
//     BLI_freelistN(&cpt_loop);
//   }
//   printf("Num inner points: %d\n", num_inner_points);
//   point_count = num_outer_points + num_inner_points;

//   ListBase *result_list = MEM_callocN(sizeof(ListBase), __func__);
//   /* duplicate all edge points */
//   tClipPoint *out_start = MEM_dupallocN(cpt_first);
//   BLI_addtail(result_list, out_start);
//   LinkData *cpt_curr_link = ((LinkData *)outer_edge.first)->next;
//   while (cpt_curr_link != NULL) {
//     cpt_curr = cpt_curr_link->data;
//     tClipPoint *out_curr = MEM_dupallocN(cpt_curr);
//     BLI_addtail(result_list, out_curr);
//     cpt_curr_link = cpt_curr_link->next;
//   }

//   BLI_freelistN(&outer_edge);

//   /* form single outline by connecting holes to outer edge */
//   while (!BLI_heap_is_empty(inner_edges)) {
//     ListBase *curr_link_hole = BLI_heap_pop_min(inner_edges);

//     ListBase curr_hole = {NULL, NULL};
//     tClipPoint *hole_start = ((LinkData *)curr_link_hole->first)->data;

//     /* duplicate all edge points */
//     tClipPoint *inner_start = MEM_dupallocN(hole_start);
//     BLI_addtail(&curr_hole, inner_start);
//     cpt_curr_link = ((LinkData *)curr_link_hole->first)->next;
//     while (cpt_curr_link != NULL) {
//       cpt_curr = cpt_curr_link->data;
//       tClipPoint *inner_curr = MEM_dupallocN(cpt_curr);
//       BLI_addtail(&curr_hole, inner_curr);
//       cpt_curr_link = cpt_curr_link->next;
//     }

//     tClipPoint *best_incert_cp = gp_find_best_insert_point(result_list, curr_hole.first);
//     gp_insert_hole_into_outer_edge(result_list, &curr_hole, best_incert_cp);
//     point_count += 2;

//     BLI_freelistN(curr_link_hole);
//   }
//   printf("Num points: %d\n", point_count);

//   cpt_first->prev = NULL;
//   cpt_last->next = NULL;

//   /* free temp data */
//   BLI_freelistN(points);
//   BLI_freelistN(&outer_isect_points);
//   BLI_heap_free(inner_edges, BLI_freelistN);

//   *r_num_points = point_count;

//   return result_list;
// }

/** \} */

/* -------------------------------------------------------------------- */
/** \name Intersection algorithm functions
 * \{ */

/*
 * TODO: This is the most primitive (brute force) way of finding line segment intersections.
 * We will need to implement a more efficient algorithm down the line.
 */
static int gp_edge_intersection_algorithm_brute_force(ListBase *edges, ListBase *clip_points)
{
  int num_intersections = 0;
  /* check every pair for intersection */
  LISTBASE_FOREACH (tClipEdge *, edgeA, edges) {
    tClipEdge *edgeB = NULL;
    for (edgeB = edgeA->next; edgeB != NULL; edgeB = edgeB->next) {
      /* check for intersection */
      if (gp_isect_clip_edges(clip_points, edgeA, edgeB) != NULL) {
        num_intersections++;
      }
    }
  }
  return num_intersections;
}

static int gp_edge_intersection_algorithm_brute_force_with_aabb(ListBase *edges,
                                                                ListBase *clip_points)
{
  int num_intersections = 0;
  /* check every pair for intersection */
  LISTBASE_FOREACH (tClipEdge *, edgeA, edges) {
    tClipEdge *edgeB = NULL;
    for (edgeB = edgeA->next; edgeB != NULL; edgeB = edgeB->next) {
      /* check bounding boxes */
      if (isect_aabb_aabb_v4(edgeA->aabb, edgeB->aabb)) {
        /* check for intersection */
        if (gp_isect_clip_edges(clip_points, edgeA, edgeB) != NULL) {
          num_intersections++;
        }
      }
    }
  }
  return num_intersections;
}

/**
 * Bentley-Ottmann implementation
 */

typedef struct tClipEvent {
  tClipPoint *pt;
  tClipEdge *edge;
  WAVL_Node *sweep_line_node;
  struct tClipEvent *isect_link;
  char type;
} tClipEvent;

enum CLIP_EVENT_TYPE {
  CLIP_EVENT_START = 1,
  CLIP_EVENT_END = 2,
  CLIP_EVENT_INTERSECTION = 3,
};

static inline tClipPoint *get_pair_event_clip_point(tClipEvent *evt)
{
  if (evt->edge->start == evt->pt) {
    return evt->edge->end;
  }
  return evt->edge->start;
}

static short gp_compare_clip_edge(void *A, void *B)
{
  tClipEdge *edgeA = (tClipEdge *)A;
  tClipEdge *edgeB = (tClipEdge *)B;
  if (edgeA->start->x < edgeB->start->x) {
    return -1;
  }
  else if (edgeA->start->x > edgeB->start->x) {
    return 1;
  }
  else {
    if (edgeA->start->y < edgeB->start->y) {
      return -1;
    }
    else if (edgeA->start->y > edgeB->start->y) {
      return 1;
    }
    else {
      if (edgeA->end->x < edgeB->end->x) {
        return -1;
      }
      else if (edgeA->end->x > edgeB->end->x) {
        return 1;
      }
      else {
        if (edgeA->end->y < edgeB->end->y) {
          return -1;
        }
        else if (edgeA->end->y > edgeB->end->y) {
          return 1;
        }
        else {
          return 0;
        }
      }
    }
  }
}

/**
 *  Helper: Check intersection for edges of two events
 */
static bool isect_clip_event_edges(ListBase *clip_points,
                                   Heap *event_queue,
                                   WAVL_Node *curr_node,
                                   WAVL_Node *other_node)
{
  if (curr_node != NULL && other_node != NULL) {
    tClipEdge *curr_edge = curr_node->data;
    tClipEdge *other_edge = other_node->data;
    if (curr_edge->start != other_edge->start && curr_edge->end != other_edge->end) {
      if (isect_aabb_aabb_v4(curr_edge->aabb, other_edge->aabb)) {
        tClipPoint *cpt_isectA = gp_isect_clip_edges(clip_points, curr_edge, other_edge);
        if (cpt_isectA != NULL) {
          tClipPoint *cpt_isectB = cpt_isectA->isect_link;

          tClipEvent *isectA_event = MEM_callocN(sizeof(tClipEvent), __func__);
          tClipEvent *isectB_event = MEM_callocN(sizeof(tClipEvent), __func__);
          isectA_event->pt = cpt_isectA;
          isectA_event->edge = curr_edge;
          isectA_event->sweep_line_node = curr_node;
          isectA_event->type = CLIP_EVENT_INTERSECTION;

          isectB_event->pt = cpt_isectB;
          isectB_event->edge = other_edge;
          isectB_event->sweep_line_node = other_node;
          isectB_event->type = CLIP_EVENT_INTERSECTION;

          isectA_event->isect_link = isectB_event;
          isectB_event->isect_link = isectA_event;

          /* TODO: figure out how we can sort by y if x is the same */
          BLI_heap_insert(event_queue, isectA_event->pt->x, isectA_event);
          return true;
        }
      }
    }
  }
  return false;
}

/**
 * Finds all the intersections of the edges and inserts the newly created points in the
 * clip_points list at the right places
 */
static int gp_bentley_ottmann_algorithm(ListBase *edges, int num_edges, ListBase *clip_points)
{
  int num_events = num_edges * 2;
  Heap *event_queue = BLI_heap_new_ex(num_events);
  WAVL_Tree *sweep_line_tree = BLI_wavlTree_new();

  /* Create events and add them to min-heap */
  LISTBASE_FOREACH (tClipEdge *, edge, edges) {
    tClipEvent *start_event = MEM_callocN(sizeof(tClipEvent), __func__);
    tClipEvent *end_event = MEM_callocN(sizeof(tClipEvent), __func__);
    if (edge->start->x < edge->end->x) {
      start_event->pt = edge->start;
    }
    else if (edge->start->x > edge->end->x) {
      start_event->pt = edge->end;
    }
    else {
      if (edge->start->y < edge->end->y) {
        start_event->pt = edge->start;
      }
      else if (edge->start->y > edge->end->y) {
        start_event->pt = edge->end;
      }
      else {
        /* edge is single point (should not happen) */
        start_event->pt = edge->start;
      }
    }
    start_event->edge = edge;
    start_event->type = CLIP_EVENT_START;

    end_event->pt = get_pair_event_clip_point(start_event);
    end_event->edge = edge;
    end_event->type = CLIP_EVENT_END;

    /* TODO: figure out how we can sort by y if x is the same */
    BLI_heap_insert(event_queue, start_event->pt->x, start_event);
    BLI_heap_insert(event_queue, end_event->pt->x, end_event);
  }

  int num_intersections = 0;
  while (!BLI_heap_is_empty(event_queue)) {
    tClipEvent *event = BLI_heap_pop_min(event_queue);
    tClipEdge *event_edge = event->edge;

    if (event->type == CLIP_EVENT_INTERSECTION) {
      /* swap the line segments in the sweep_line datastruct
       * and check neighbours for intersections */
      tClipEvent *isect_event = event->isect_link;
      tClipEdge *edge_isect = isect_event->edge;

      WAVL_Node *nodeA = event->sweep_line_node;
      WAVL_Node *nodeB = isect_event->sweep_line_node;

      tClipPoint *cpt_isectA = event->pt;
      tClipPoint *cpt_isectB = cpt_isectA->isect_link;

      event_edge->start = cpt_isectA;
      edge_isect->start = cpt_isectB;

      /* swap data in tree */
      nodeA->data = edge_isect;
      nodeB->data = event_edge;

      WAVL_Node *pair_nodeB = BLI_wavlTree_predecessor_ex(nodeB);
      WAVL_Node *pair_nodeA = BLI_wavlTree_successor_ex(nodeA);
      if (pair_nodeB == nodeA) {
        pair_nodeB = BLI_wavlTree_successor_ex(nodeB);
        pair_nodeA = BLI_wavlTree_predecessor_ex(nodeA);
      }

      if (isect_clip_event_edges(clip_points, event_queue, nodeB, pair_nodeB)) {
        num_intersections++;
      }
      if (isect_clip_event_edges(clip_points, event_queue, nodeA, pair_nodeA)) {
        num_intersections++;
      }
    }
    else if (event->type == CLIP_EVENT_START) {
      /* insert edge into sweep_line datastruct and check neighbours for intersections */
      WAVL_Node *node = BLI_wavlTree_insert(sweep_line_tree, gp_compare_clip_edge, event_edge);
      WAVL_Node *below_node = BLI_wavlTree_predecessor_ex(node);
      WAVL_Node *above_node = BLI_wavlTree_successor_ex(node);

      if (isect_clip_event_edges(clip_points, event_queue, node, below_node)) {
        num_intersections++;
      }

      if (isect_clip_event_edges(clip_points, event_queue, node, above_node)) {
        num_intersections++;
      }
    }
    else if (event->type == CLIP_EVENT_END) {
      /* remove edge from sweep_line datastruct; check the previous neighbours for intersection */
      WAVL_Node *node = event->sweep_line_node;
      WAVL_Node *below_node = BLI_wavlTree_predecessor_ex(node);
      WAVL_Node *above_node = BLI_wavlTree_successor_ex(node);
      BLI_wavlTree_delete_node(sweep_line_tree, NULL, node);

      if (isect_clip_event_edges(clip_points, event_queue, below_node, above_node)) {
        num_intersections++;
      }
    }

    MEM_freeN(event);
  }

  return num_intersections;
}

/**
 * Main intersection algorithm entry point
 * \param clip_paths: list of tClipPaths (must have at least one tClipPath).
 *                    Intersection points will be inserted into the tClipPaths.
 * \returns number of intersection found
 */
// static int gp_clip_paths_find_intersections(ListBase *clip_paths)
// {
//   int num_intersections = 0;

//   /* TODO: iterate over all clip paths and clip them individualy as well as with each other */

//   return num_intersections;
// }

static int gp_edge_intersection_algorithm(tClipPath *clip_path)
{
  int num_intersections = gp_edge_intersection_algorithm_brute_force(&clip_path->edges,
                                                                     &clip_path->points);
  /* we insert two points, one on each of two intersecting edges */
  clip_path->num_points += 2 * num_intersections;
  gp_update_clip_edges_clip_path(clip_path);
  return num_intersections;
}

/** \} */

bGPDstroke *BKE_gpencil_stroke_clip_self(const bContext *C,
                                         const bGPDlayer *gpl,
                                         bGPDstroke *gps,
                                         int algorithm)
{
  tClipPath *clip_path = gp_stroke_to_view_space_clip_path(C, gpl, gps);

  if (algorithm == BRUTE_FORCE) {
    int num_intersections = gp_edge_intersection_algorithm_brute_force(&clip_path->edges,
                                                                       &clip_path->points);
    /* we insert two points, one on each of two intersecting edges */
    clip_path->num_points += 2 * num_intersections;
    // gp_update_clip_edges_clip_path(clip_path);
    if (num_intersections == 0) {
      free_clip_path(clip_path);
      return gps;
    }
  }
  else if (algorithm == BRUTE_FORCE_WITH_AABB) {
    int num_intersections = gp_edge_intersection_algorithm_brute_force_with_aabb(
        &clip_path->edges, &clip_path->points);
    /* we insert two points, one on each of two intersecting edges */
    clip_path->num_points += 2 * num_intersections;
    // gp_update_clip_edges_clip_path(clip_path);
    if (num_intersections == 0) {
      free_clip_path(clip_path);
      return gps;
    }
  }

  /* create new stroke */
  bGPDstroke *outer_edge_stroke = gp_view_space_clip_path_to_stroke(
      C, gpl, clip_path, gps->mat_nr, true);

  /* free temp data */
  free_clip_path(clip_path);

  return outer_edge_stroke;
}

/**
 * Finds the outline projected from the current view of a cyclic stroke and returns it as a new
 * stroke. Note: This will fill in any holes as they are ignored.
 */
bGPDstroke *BKE_gpencil_stroke_to_outline(const bContext *C, const bGPDlayer *gpl, bGPDstroke *gps)
{
  tClipPath *clip_path = gp_stroke_to_view_space_clip_path(C, gpl, gps);

  int num_isect_points = gp_edge_intersection_algorithm(clip_path);
  if (num_isect_points == 0) {
    free_clip_path(clip_path);
    return gps;
  }

  tClipPath *outline_clip_path = gp_clip_path_get_outer_edge(clip_path);

  /* create new stroke */
  bGPDstroke *outer_edge_stroke = gp_view_space_clip_path_to_stroke(
      C, gpl, outline_clip_path, gps->mat_nr, true);

  /* free temp data */
  free_clip_path(clip_path);
  free_clip_path(outline_clip_path);

  return outer_edge_stroke;
}

bGPDstroke *BKE_gpencil_fill_stroke_to_outline_with_holes(const bContext *C,
                                                          const bGPDlayer *gpl,
                                                          bGPDstroke *gps)
{
  tClipPath *clip_path = gp_stroke_to_view_space_clip_path(C, gpl, gps);

  /* intersection algorithm */
  int num_isect_points = gp_edge_intersection_algorithm(clip_path);
  if (num_isect_points == 0) {
    free_clip_path(clip_path);
    BKE_gpencil_stroke_from_view_space(C, gpl, gps);
    return NULL;
  }

  tClipPath *outline_clip_path = gp_clip_path_get_outer_edge_with_holes(clip_path);

  /* create new stroke */
  bGPDstroke *outline_stroke = gp_view_space_clip_path_to_stroke(
      C, gpl, outline_clip_path, gps->mat_nr, true);

  /* free temp data */
  free_clip_path(clip_path);
  free_clip_path(outline_clip_path);

  return outline_stroke;
}

// /*
//  * Takes two stroke outlines and applies a union boolean operation.
//  * Returns either the unified stroke or NULL if the strokes do not intersect.
//  */
// bGPDstroke *BKE_gpencil_stroke_outline_union(const RegionView3D *rv3d,
//                                              const bGPDlayer *gpl,
//                                              bGPDstroke *gps_A,
//                                              bGPDstroke *gps_B)
// {
//   ListBase clip_points = {NULL, NULL};
//   ListBase edges = {NULL, NULL};
//   /* convert stroke to point and edge data strusture */
//   gp_stroke_to_points_and_edges(gps_A, rv3d->viewmat, &edges, &clip_points);
//   gp_stroke_to_points_and_edges(gps_B, rv3d->viewmat, &edges, &clip_points);
//   int num_isect_points = gp_edge_intersection_algorithm(&edges, &clip_points);
//   if (num_isect_points == 0) {
//     BLI_freelistN(&edges);
//     BLI_freelistN(&clip_points);
//     return NULL;
//   }

//   /* walk along the outline */
//   ListBase outline_points = {NULL, NULL};
//   int num_outline_points = gp_get_outer_edge(&clip_points, &outline_points);

//   /* create new stroke */
//   bGPDstroke *outline_stroke = gp_clip_points_to_gp_stroke(
//       &outline_points, num_outline_points, rv3d->viewinv, gps_A->mat_nr, true);

//   /* free temp data */
//   BLI_freelistN(&edges);
//   BLI_freelistN(&clip_points);
//   BLI_freelistN(&outline_points);

//   outline_stroke->flag |= GP_STROKE_CYCLIC;

//   /* Delete the old strokes */
//   BLI_remlink(&gpl->actframe->strokes, gps_A);
//   BLI_remlink(&gpl->actframe->strokes, gps_B);
//   BKE_gpencil_free_stroke(gps_A);
//   BKE_gpencil_free_stroke(gps_B);

//   return outline_stroke;
// }
