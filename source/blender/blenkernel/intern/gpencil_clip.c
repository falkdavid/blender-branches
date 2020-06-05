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
#include "BLI_heap_cmp.h"
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

void gp_update_clip_edge_aabb(tClipEdge *edge)
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

static tClipPoint *gp_create_and_insert_isect_point(ListBase *clip_points,
                                                    tClipEdge *edgeA,
                                                    tClipEdge *edgeB,
                                                    const float isect_pt[2])
{
  tClipPoint *cpt_isectA = MEM_callocN(sizeof(tClipPoint), __func__);
  copy_v2_v2(&cpt_isectA->x, isect_pt);
  cpt_isectA->isect_type = gp_clip_point_isect_type(edgeA, edgeB);

  tClipPoint *cpt_isectB = MEM_dupallocN(cpt_isectA);
  cpt_isectB->isect_type = !cpt_isectA->isect_type;

  cpt_isectA->z = edgeA->start->z;
  cpt_isectA->isect_dist = len_v2v2(&edgeA->start->x, &edgeA->end->x) /
                           len_v2v2(&edgeA->start->x, isect_pt);
  cpt_isectB->z = edgeB->start->z;
  cpt_isectB->isect_dist = len_v2v2(&edgeB->start->x, &edgeB->end->x) /
                           len_v2v2(&edgeB->start->x, isect_pt);

  cpt_isectA->isect_link = cpt_isectB;
  cpt_isectB->isect_link = cpt_isectA;

  /* insert intersection point at the right place */
  tClipPoint *cpt_insert = edgeA->start;
  while (cpt_insert->next != edgeA->end && cpt_insert->next->isect_dist > cpt_isectA->isect_dist) {
    cpt_insert = cpt_insert->next;
  }
  BLI_insertlinkafter(clip_points, cpt_insert, cpt_isectA);

  cpt_insert = edgeB->start;
  while (cpt_insert->next != edgeB->end && cpt_insert->next->isect_dist > cpt_isectB->isect_dist) {
    cpt_insert = cpt_insert->next;
  }
  BLI_insertlinkafter(clip_points, cpt_insert, cpt_isectB);

  return cpt_isectA;
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
    return gp_create_and_insert_isect_point(clip_points, edgeA, edgeB, isect_pt);
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
//           BLI_heap_cmp_insert(inner_edges, ((tClipPoint *)inner_edge->first)->x, inner_edge);
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

/**
 * Brute force algorithms
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

static void print_edge(const char *pre, tClipEdge *edge)
{
  tClipPoint *edge_end = edge->x_dir ? edge->end : edge->start;
  printf("%s (%p) ", pre, edge);
  printf("from (%.12f, %.12f) ", edge->sweep_pt->x, edge->sweep_pt->y);
  printf("to (%.12f, %.12f) ", edge_end->x, edge_end->y);
  printf("(x dir = %s)\n", edge->x_dir ? "right" : "left");
}

short gp_compare_points(const float A[2], const float B[2])
{
  if (A[0] < B[0]) {
    return -1;
  }
  if (A[0] > B[0]) {
    return 1;
  }
  if (A[1] < B[1]) {
    return -1;
  }
  if (A[1] > B[1]) {
    return 1;
  }
  return 0;
}

inline bool gp_edge_is_vertical(tClipEdge *edge)
{
  return edge->start->x == edge->end->x;
}

/* returns -1 if C is to the left of AB, 1 if its on the right and 0 if its on the line */
inline short gp_point_is_right(const float A[2], const float B[2], const float C[2])
{
  float r = (B[0] - A[0]) * (C[1] - A[1]) - (B[1] - A[1]) * (C[0] - A[0]);
  if (r > 0.0f)
    return -1;
  if (r < 0.0f)
    return 1;
  return 0;
}

short gp_compare_clip_points(void *A, void *B)
{
  tClipPoint *pointA = (tClipPoint *)A;
  tClipPoint *pointB = (tClipPoint *)B;
  if (pointA == pointB) {
    return 0;
  }
  return gp_compare_points(&pointA->x, &pointB->x);
}

float gp_y_intercept_edge(tClipEdge *edge, float x)
{
  tClipPoint *edge_start = edge->x_dir ? edge->start : edge->end;
  tClipPoint *edge_end = edge->x_dir ? edge->end : edge->start;
  if (x <= edge_start->x) {
    return edge_start->y;
  }
  if (x >= edge_end->x) {
    return edge_end->y;
  }

  float x_dist = edge_end->x - edge_start->x;
  if (x_dist == 0.0f) {
    /* edge is vertical, return smallest y */
    return edge->aabb[2];
  }
  float dx0 = x - edge_start->x;
  float dx1 = edge_end->x - x;

  float fac, ifac;
  if (dx0 > dx1) {
    ifac = dx0 / x_dist;
    fac = 1.0f - ifac;
  }
  else {
    fac = dx1 / x_dist;
    ifac = 1.0f - fac;
  }

  return edge_start->y * fac + edge_end->y * ifac;
}

short gp_y_compare_clip_edges(void *A, void *B)
{
  /* XXX: check the bounding boxes first to check if an edge is above or below.
   * Otherwise calculate the y-intercept of B of the x coordintate of the sweep point of A */

  tClipEdge *edgeA = (tClipEdge *)A;
  tClipEdge *edgeB = (tClipEdge *)B;
  if (edgeA == edgeB) {
    return 0;
  }

  /* max_y A < min_y B -> edge A is entirely below edge B */
  if (edgeA->aabb[3] < edgeB->aabb[2]) {
    return -1;
  }
  /* min_y A < max_y B -> edge A is entirely above edge B */
  if (edgeA->aabb[2] > edgeB->aabb[3]) {
    return 1;
  }

  float current_x = edgeA->sweep_pt->x;
  float current_y = edgeA->sweep_pt->y;

  /* calculate the y intersept at the current sweep x
   * note: the sweep x of an edge can only be <= to the current x*/
  float y_icept = gp_y_intercept_edge(edgeB, current_x);
  if (current_y < y_icept) {
    return -1;
  }
  if (current_y > y_icept) {
    return 1;
  }
  /* y intercept is the same for both edges
   * note: if edgeB is vertical, the y intercept is at the bottom point */

  /**
   * We need to check for 4 cases:
   *  - the two start points are the same
   *  - the two end points are the same
   *  - the start and end point are the same
   *  - its an intersection point (the sweep points are the same)
   */

  // tClipPoint *endA = edgeA->x_dir ? edgeA->end : edgeA->start;
  // tClipPoint *endB = edgeB->x_dir ? edgeB->end : edgeB->start;
  // if (gp_compare_clip_points(edgeA->sweep_pt, endB) == 0) {
  // }
  // if (gp_compare_clip_points(edgeA->sweep_pt, startB) == 0) {
  // }

  // /* handle case for end event */
  // if (edgeA->sweep_pt == endA) {
  //   if (current_x < endB->x) {
  //     return -1;
  //   }
  //   if (current_x > endB->x) {
  //     return 1;
  //   }
  //   tClipPoint *startB = edgeB->x_dir ? edgeB->start : edgeB->end;
  //   if (current_x < startB->x) {
  //     return -1;
  //   }
  //   if (current_x > startB->x) {
  //     return 1;
  //   }
  // }

  tClipPoint *startA = edgeA->x_dir ? edgeA->start : edgeA->end;
  /* if y intersept is equal, check endpoints */
  short cmp = gp_point_is_right(&startA->x, &endA->x, &endB->x);
  if (cmp != 0) {
    return cmp;
  }

  if (endA->x < endB->x) {
    return -1;
  }
  if (endA->x > endB->x) {
    return 1;
  }

  tClipPoint *startB = edgeB->x_dir ? edgeB->start : edgeB->end;
  if (startA->y < startB->y) {
    return -1;
  }
  if (startA->y > startB->y) {
    return 1;
  }
  if (startA->x < startB->x) {
    return -1;
  }
  if (startA->x > startB->x) {
    return 1;
  }

  return 0;
}

short gp_compare_clip_events(void *A, void *B)
{
  tClipEvent *eventA = (tClipEvent *)A;
  tClipEvent *eventB = (tClipEvent *)B;
  if (eventA->pt->x < eventB->pt->x) {
    return -1;
  }
  if (eventA->pt->x > eventB->pt->x) {
    return 1;
  }

  if (eventA->pt->y < eventB->pt->y) {
    return -1;
  }
  if (eventA->pt->y > eventB->pt->y) {
    return 1;
  }

  if (eventA->type < eventB->type) {
    return -1;
  }
  if (eventA->type > eventB->type) {
    return 1;
  }

  return 0;
}

/**
 *  Helper: Check intersection for two edges and create event
 */
static tClipEvent *isect_clip_event_edges(tClipEvent *curr_event,
                                          tClipEdge *curr_edge,
                                          tClipEdge *other_edge)
{
  if (isect_aabb_aabb_v4(curr_edge->aabb, other_edge->aabb)) {
    float *p0_a = &curr_edge->start->x;
    float *p1_a = &curr_edge->end->x;
    float *p0_b = &other_edge->start->x;
    float *p1_b = &other_edge->end->x;

    float isect_pt[2];
    int status = isect_seg_seg_v2_point_ex(p0_a, p1_a, p0_b, p1_b, -FLT_MIN, isect_pt);
    /* if there is an exact intersection point and the point is in not before the sweep line */
    if (status == 1 && isect_pt[0] >= curr_event->pt->x) {
      tClipPoint *cpt_isect = MEM_callocN(sizeof(tClipPoint), __func__);
      copy_v2_v2(&cpt_isect->x, isect_pt);

      /* create new events */
      tClipEvent *isect_event = MEM_callocN(sizeof(tClipEvent), __func__);
      isect_event->pt = cpt_isect;
      isect_event->edge = curr_edge;
      isect_event->isect_link_edge = other_edge;
      isect_event->type = CLIP_EVENT_INTERSECTION;
      printf("Created new isect event at (%.4f, %.4f)!\n", isect_pt[0], isect_pt[1]);
      print_edge("curr_edge", curr_edge);
      print_edge("other_edge", other_edge);
      return isect_event;
    }
  }
  return NULL;
}
#define DEBUG_BO 1
/**
 * Finds all the intersections of the edges and inserts the newly created points in the
 * clip_points list at the correct places
 */
static int gp_bentley_ottmann_algorithm(ListBase *edges, int num_edges, ListBase *clip_points)
{
  int num_events = num_edges * 2;
#ifdef DEBUG_BO
  printf("Num edges: %d, Num events: %d\n", num_edges, num_events);
#endif
  Heap *event_queue = BLI_heap_cmp_new_ex(num_events);
  WAVL_Tree *sweep_line_tree = BLI_wavlTree_new();

  /* Create events and add them to min-heap */
  LISTBASE_FOREACH (tClipEdge *, edge, edges) {
    tClipEvent *start_event = MEM_callocN(sizeof(tClipEvent), __func__);
    tClipEvent *end_event = MEM_callocN(sizeof(tClipEvent), __func__);
    tClipPoint *edge_start;
    tClipPoint *edge_end;
    if (edge->start->x < edge->end->x) {
      edge->x_dir = 1;
      edge_start = edge->start;
      edge_end = edge->end;
    }
    else if (edge->start->x > edge->end->x) {
      edge->x_dir = 0;
      /* switch start and end to make edge point in x direction */
      edge_start = edge->end;
      edge_end = edge->start;
    }
    else {
      /* edge is vertical */
      if (edge->start->y < edge->end->y) {
        edge->x_dir = 1;
        edge_start = edge->start;
        edge_end = edge->end;
      }
      else if (edge->start->y > edge->end->y) {
        edge->x_dir = 0;
        /* swap start and end to make edge point in x direction */
        edge_start = edge->end;
        edge_end = edge->start;
      }
      else {
        /* edge is single point (should not happen) */
        edge_start = edge->start;
        edge_end = edge->end;
      }
    }
    edge->sweep_pt = edge_start;
    print_edge("insert: ", edge);
    start_event->pt = edge_start;
    start_event->edge = edge;
    start_event->type = CLIP_EVENT_START;

    end_event->pt = edge_end;
    end_event->edge = edge;
    end_event->type = CLIP_EVENT_END;

    BLI_heap_cmp_insert(event_queue, gp_compare_clip_events, start_event);
    BLI_heap_cmp_insert(event_queue, gp_compare_clip_events, end_event);
  }

  int num_intersections = 0;
  int ii = 0;
  while (!BLI_heap_cmp_is_empty(event_queue)) {
    tClipEvent *event = BLI_heap_cmp_pop_min(event_queue, gp_compare_clip_events);
    tClipEdge *event_edge = event->edge;
#ifdef DEBUG_BO
    printf("I: %d\n", ii);
    printf("Queue size: %u\n", BLI_heap_cmp_len(event_queue));
    printf("Event at (%.12f, %.12f) (%s)\n",
           event->pt->x,
           event->pt->y,
           (event->type == CLIP_EVENT_START) ?
               "START" :
               ((event->type == CLIP_EVENT_END) ? "END" : "INTERSECTION"));
    print_edge("Event edge", event_edge);
#endif
    tClipEdge *other_edge = NULL;
    if (event->type == CLIP_EVENT_INTERSECTION) {
      /* pop all duplicate events */
      for (tClipEvent *next_event = BLI_heap_cmp_top_ptr(event_queue);
           next_event->type == CLIP_EVENT_INTERSECTION &&
           gp_compare_clip_points(event->pt, next_event->pt) == 0;
           next_event = BLI_heap_cmp_top_ptr(event_queue)) {
        tClipEvent *dupli_event = BLI_heap_cmp_pop_min(event_queue, gp_compare_clip_events);
        MEM_freeN(dupli_event->pt);
        MEM_freeN(dupli_event);
      }

      /* create intersection point and insert points into point list */
      tClipPoint *cpt_isectA = event->pt;
      other_edge = event->isect_link_edge;

      float *isect_pt = &cpt_isectA->x;
      float *p0_a = &event_edge->start->x;
      float *p1_a = &event_edge->end->x;
      float *p0_b = &other_edge->start->x;
      float *p1_b = &other_edge->end->x;

      tClipPoint *cpt_isectB = MEM_dupallocN(cpt_isectA);

      cpt_isectA->z = event_edge->start->z;
      cpt_isectA->isect_dist = len_v2v2(p0_a, p1_a) / len_v2v2(p0_a, isect_pt);
      cpt_isectB->z = other_edge->start->z;
      cpt_isectB->isect_dist = len_v2v2(p0_b, p1_b) / len_v2v2(p0_b, isect_pt);

      cpt_isectA->isect_link = cpt_isectB;
      cpt_isectB->isect_link = cpt_isectA;

      /* insert intersection point at the right place in clip point list*/
      tClipPoint *cpt_insert = event_edge->start;
      while (cpt_insert->next != event_edge->end &&
             cpt_insert->next->isect_dist > cpt_isectA->isect_dist) {
        cpt_insert = cpt_insert->next;
      }
      BLI_insertlinkafter(clip_points, cpt_insert, cpt_isectA);

      cpt_insert = other_edge->start;
      while (cpt_insert->next != other_edge->end &&
             cpt_insert->next->isect_dist > cpt_isectB->isect_dist) {
        cpt_insert = cpt_insert->next;
      }
      BLI_insertlinkafter(clip_points, cpt_insert, cpt_isectB);

      /* swap the line segments in the sweep_line datastruct
       * and check neighbours for intersections */

      WAVL_Node *nodeA = BLI_wavlTree_search(sweep_line_tree, gp_y_compare_clip_edges, event_edge);
      WAVL_Node *nodeB = BLI_wavlTree_search(sweep_line_tree, gp_y_compare_clip_edges, other_edge);

#ifdef DEBUG_BO
      printf("nodeA: %p, nodeB: %p\n", nodeA, nodeB);
      printf("swapping: %p %p\n", event_edge, other_edge);
      WAVLTREE_REVERSE_INORDER(tClipEdge *, _clip_edge, sweep_line_tree)
      {
        printf("Data: %p (%.4f)\n", _clip_edge, _clip_edge->sweep_pt->y);
      }
      if (BLI_wavlTree_predecessor_ex(nodeA) == nodeB) {
        printf("pred nodeA %p = nodeB %p\n", BLI_wavlTree_predecessor_ex(nodeA), nodeB);
      }
      else if (BLI_wavlTree_successor_ex(nodeA) == nodeB) {
        printf("succ nodeA %p = nodeB %p\n", BLI_wavlTree_successor_ex(nodeA), nodeB);
      }
      else {
        printf("nodeA %p and nodeB %p are not neightbours\n", nodeA, nodeB);
      }
      print_edge("nodeA data", nodeA->data);
      print_edge("nodeB data", nodeB->data);
      short cmp1 = gp_y_compare_clip_edges(nodeA->data, nodeB->data);
      printf("cmp1: %d\n", cmp1);
#endif

      nodeA->data = other_edge;
      nodeB->data = event_edge;

#ifdef DEBUG_BO
      print_edge("nodeA data", nodeA->data);
      print_edge("nodeB data", nodeB->data);
      short cmp2 = gp_y_compare_clip_edges(nodeA->data, nodeB->data);
      printf("cmp2: %d\n", cmp2);

      if (cmp1 == cmp2 || cmp2 == 0 || cmp1 == 0)
        printf("compare values are wrong! before: %d after: %d\n", cmp1, cmp2);
#endif

      event_edge->sweep_pt = cpt_isectA;
      other_edge->sweep_pt = cpt_isectB;

      WAVL_Node *pair_nodeA;
      WAVL_Node *pair_nodeB;
      if (BLI_wavlTree_successor_ex(nodeA) == nodeB) {
        pair_nodeA = BLI_wavlTree_predecessor_ex(nodeA);
        pair_nodeB = BLI_wavlTree_successor_ex(nodeB);
      }
      else {
        pair_nodeA = BLI_wavlTree_successor_ex(nodeA);
        pair_nodeB = BLI_wavlTree_predecessor_ex(nodeB);
      }

      /* check new neighbours for intersections */
      tClipEvent *isect_event;
      if (pair_nodeA != NULL) {
        isect_event = isect_clip_event_edges(event, nodeA->data, pair_nodeA->data);
        if (isect_event != NULL) {
          BLI_heap_cmp_insert(event_queue, gp_compare_clip_events, isect_event);
        }
      }

      if (pair_nodeB != NULL) {
        isect_event = isect_clip_event_edges(event, nodeB->data, pair_nodeB->data);
        if (isect_event != NULL) {
          BLI_heap_cmp_insert(event_queue, gp_compare_clip_events, isect_event);
        }
      }

      num_intersections++;
    }
    else if (event->type == CLIP_EVENT_START) {
      /* insert edge into sweep_line datastruct and check neighbours for intersections */
      WAVL_Node *node = BLI_wavlTree_insert(sweep_line_tree, gp_y_compare_clip_edges, event_edge);
      if (node == NULL) {
        printf("Node could not be inserted! already exists!\n");
        printf("start: %p, end: %p\n", event_edge->start, event_edge->end);
        print_v2("start", &event_edge->start->x);
        print_v2("end", &event_edge->end->x);
        printf("len: %.12f\n", len_v2v2(&event_edge->start->x, &event_edge->end->x));
      }
      WAVL_Node *below_node = BLI_wavlTree_predecessor_ex(node);
      WAVL_Node *above_node = BLI_wavlTree_successor_ex(node);

      tClipEvent *isect_event;
      if (below_node != NULL) {
        isect_event = isect_clip_event_edges(event, node->data, below_node->data);
        if (isect_event != NULL) {
          BLI_heap_cmp_insert(event_queue, gp_compare_clip_events, isect_event);
        }
      }
      if (above_node != NULL) {
        isect_event = isect_clip_event_edges(event, node->data, above_node->data);
        if (isect_event != NULL) {
          BLI_heap_cmp_insert(event_queue, gp_compare_clip_events, isect_event);
        }
      }
    }
    else if (event->type == CLIP_EVENT_END) {
      /* remove edge from sweep_line datastruct; check the previous neighbours for intersection
       * note: this can return NULL if the edge inserted was identical with another. In that case,
       * just skip the intersection checking */
      event_edge->sweep_pt = event->pt;
      WAVL_Node *node = BLI_wavlTree_search(sweep_line_tree, gp_y_compare_clip_edges, event_edge);
#ifdef DEBUG_BO
      printf("Delete node: %p\n", node);
      if (node->data != event_edge) {
        printf("Did not find right edge!\n");
        print_edge("Found: ", node->data);
        print_edge("Actual: ", event_edge);
      }
#endif
      // if (node != NULL) {
      WAVL_Node *below_node = BLI_wavlTree_predecessor_ex(node);
      WAVL_Node *above_node = BLI_wavlTree_successor_ex(node);
      BLI_wavlTree_delete_node(sweep_line_tree, NULL, node);

      tClipEvent *isect_event;
      if (below_node != NULL && above_node != NULL) {
        isect_event = isect_clip_event_edges(event, below_node->data, above_node->data);
        if (isect_event != NULL) {
          BLI_heap_cmp_insert(event_queue, gp_compare_clip_events, isect_event);
        }
      }
      //}
    }
#ifdef DEBUG_BO
    printf("\nTree size: %u\n", BLI_wavlTree_size(sweep_line_tree));
    float prev_isept = FLT_MAX;
    bool order = true;
    WAVLTREE_REVERSE_INORDER(tClipEdge *, clip_edge, sweep_line_tree)
    {
      float y_isept = clip_edge != event->edge ? gp_y_intercept_edge(clip_edge, event->pt->x) :
                                                 event->pt->y;
      if (y_isept > prev_isept) {
        printf("%.12f > %.12f\n", y_isept, prev_isept);
        order = false;
      }

      printf("Data: %p (%.4f) %s\n",
             clip_edge,
             y_isept,
             (clip_edge == event_edge || clip_edge == other_edge) ? "*" : "");
      prev_isept = y_isept;
    }
    if (!order) {
      printf("ORDER IS NOT CORRECT!\n");
    }
    printf("\n");
    ii++;
#endif
    MEM_freeN(event);
  }

  BLI_heap_cmp_free(event_queue, NULL);
  BLI_wavlTree_free(sweep_line_tree, NULL);
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
  int num_intersections = 0;
  if (algorithm == BRUTE_FORCE) {
    num_intersections = gp_edge_intersection_algorithm_brute_force(&clip_path->edges,
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
    num_intersections = gp_edge_intersection_algorithm_brute_force_with_aabb(&clip_path->edges,
                                                                             &clip_path->points);
    /* we insert two points, one on each of two intersecting edges */
    clip_path->num_points += 2 * num_intersections;
    // gp_update_clip_edges_clip_path(clip_path);
    if (num_intersections == 0) {
      free_clip_path(clip_path);
      return gps;
    }
  }
  else if (algorithm == BENTLEY_OTTMANN) {
    num_intersections = gp_bentley_ottmann_algorithm(
        &clip_path->edges, clip_path->num_edges, &clip_path->points);
    /* we insert two points, one on each of two intersecting edges */
    clip_path->num_points += 2 * num_intersections;
    // gp_update_clip_edges_clip_path(clip_path);
    if (num_intersections == 0) {
      free_clip_path(clip_path);
      return gps;
    }
  }
  else {
    printf("Algorithm not found!\n");
    free_clip_path(clip_path);
    return gps;
  }

  printf("Num intersections: %d\n", num_intersections);

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
