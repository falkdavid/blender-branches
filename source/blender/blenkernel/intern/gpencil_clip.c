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
#include "BLI_math_vector.h"
#include "BLI_heap.h"
#include "BLI_wavl_tree.h"

#include "BLT_translation.h"

#include "DNA_gpencil_types.h"
#include "DNA_space_types.h"

#include "BKE_gpencil.h"
#include "BKE_gpencil_clip.h"
#include "BKE_gpencil_geom.h"
#include "BKE_main.h"

typedef struct tClipPoint {
  /* Listbase functonality */
  struct tClipPoint *next, *prev;
  /* Coordinates */
  float x, y, z;
  /* NULL if point is not an intersection,
   * otherwise link to other intersection point */
  struct tClipPoint *isect_link;
  /* value between 0 and 1 that represents the
   * normalized distance of intersection to 
   * the start of the edge */
  float isect_dist;
} tClipPoint;

typedef struct t2dStrokeEdge {
  /* Listbase functonality */
  struct tStrokeEdge *next, *prev;
  /* pointers to start and end */
  struct tClipPoint *start, *end;
  /* AABB */
  float min[2], max[2];
} t2dStrokeEdge;

static t2dStrokeEdge *gp_add_edge_from_clip_points(tClipPoint* A, tClipPoint *B)
{
  t2dStrokeEdge *new_edge = MEM_callocN(sizeof(t2dStrokeEdge), __func__);
  new_edge->start = A;
  new_edge->end = B;
  new_edge->min[0] = A->x < B->x ? A->x : B->x;
  new_edge->min[1] = A->y < B->y ? A->y : B->y;
  new_edge->max[0] = A->x > B->x ? A->x : B->x;
  new_edge->max[1] = A->y > B->y ? A->y : B->y;
  return new_edge;
}

/* Helper: get 3d point into proj space */
static void gpencil_point3d_to_proj_space(const float mat[4][4], const float p[3], float r[4])
{
  copy_v3_v3(r, p);
  r[3] = 1.0f;
  mul_m4_v4(mat, r);
}

static bool isect_aabb_aabb_v2(const float min1[2],
                               const float max1[2],
                               const float min2[2],
                               const float max2[2])
{
  return (min1[0] < max2[0] && min1[1] < max2[1] && min2[0] < max1[0] && min2[1] < max1[1]);
}

static void gp_stroke_to_points_and_edges(const bGPDstroke *gps, const float proj_mat[4][4],
                                          ListBase *edges, ListBase *clip_points)
{
  tClipPoint *cpt_first = MEM_callocN(sizeof(tClipPoint), __func__);
  bGPDspoint *pt_first = &gps->points[0];
  float vec_tmp[4];
  gpencil_point3d_to_proj_space(proj_mat, &pt_first->x, vec_tmp);
  copy_v3_v3(&cpt_first->x, vec_tmp);
  BLI_addhead(clip_points, cpt_first);

  tClipPoint *cpt_prev = cpt_first;
  for (int i = 1; i < gps->totpoints; i++) {
    tClipPoint *cpt_curr = MEM_callocN(sizeof(tClipPoint), __func__);
    bGPDspoint *pt = &gps->points[i];
    gpencil_point3d_to_proj_space(proj_mat, &pt->x, vec_tmp);
    copy_v3_v3(&cpt_curr->x, vec_tmp);
    BLI_addtail(clip_points, cpt_curr);

    t2dStrokeEdge *new_edge = gp_add_edge_from_clip_points(cpt_prev, cpt_curr);
    BLI_addtail(edges, new_edge);
    cpt_prev = cpt_curr;
  }
}

static int gp_outline_walk_algorithm(ListBase *points, ListBase *outline_points)
{
  int num_outline_points = 1;
  /* find point on the edge first */
  tClipPoint *cpt_min = NULL;
  float min_x = FLT_MAX;
  LISTBASE_FOREACH(tClipPoint *, curr, points) {
    if (curr->x < min_x) {
      min_x = curr->x;
      cpt_min = curr;
    }
  }

  /* link ends together */
  tClipPoint *cpt_first = points->first;
  tClipPoint *cpt_last = points->last;
  cpt_first->prev = cpt_last;
  cpt_last->next = cpt_first;

  /* now we know that the right side of the edge to the next pt is inside */
  tClipPoint *cpt_start = cpt_min;
  tClipPoint *out_start = MEM_dupallocN(cpt_start);
  BLI_addtail(outline_points, out_start);
  tClipPoint *cpt_curr = cpt_start->next;
  while (cpt_curr != cpt_start) {
    tClipPoint *out_curr = MEM_dupallocN(cpt_curr);
    BLI_addtail(outline_points, out_curr);
    if(cpt_curr->isect_link != NULL) {
      cpt_curr = cpt_curr->isect_link->next;
    }
    else {
      cpt_curr = cpt_curr->next;
    }
    num_outline_points++;
  }

  /* unlink ends */
  cpt_first->prev = NULL;
  cpt_last->next = NULL;

  return num_outline_points;
}

static int naive_intersection_algorithm(const RegionView3D *rv3d, const bGPDstroke *gps, ListBase *clip_points)
{
  ListBase edges = {NULL, NULL};
  gp_stroke_to_points_and_edges(gps, rv3d->viewmat, &edges, clip_points);

  int num_intersections = 0;
  /* check every pair for intersection */
  LISTBASE_FOREACH(t2dStrokeEdge *, edgeA, &edges) {
    t2dStrokeEdge *edgeB = NULL;
    for (edgeB = edgeA->next; edgeB != NULL; edgeB = edgeB->next) {
      /* check bounding boxes */
      if (isect_aabb_aabb_v2(edgeA->min, edgeA->max, edgeB->min, edgeB->max)) {
        /* check for intersection */
        float *p0_a = &edgeA->start->x;
        float *p1_a = &edgeA->end->x;
        float *p0_b = &edgeB->start->x;
        float *p1_b = &edgeB->end->x;

        float isect_pt[2];
        int status = isect_seg_seg_v2_point(p0_a, p1_a, p0_b, p1_b, isect_pt);
        if (status == 1) {
          tClipPoint *cpt_isectA = MEM_callocN(sizeof(tClipPoint), __func__);
          copy_v2_v2(&cpt_isectA->x, isect_pt);
          tClipPoint *cpt_isectB = MEM_dupallocN(cpt_isectA);

          cpt_isectA->z = edgeA->start->z;
          cpt_isectA->isect_dist = len_v2v2(p0_a, p1_a) / len_v2v2(p0_a, isect_pt);
          cpt_isectB->z = edgeB->start->z;
          cpt_isectB->isect_dist = len_v2v2(p0_b, p1_b) / len_v2v2(p0_b, isect_pt);

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

          cpt_isectA->isect_link = cpt_isectB;
          cpt_isectB->isect_link = cpt_isectA;
          num_intersections++;
        }
      }
    }
  }

  BLI_freelistN(&edges);
  return num_intersections;
}

// bool BKE_gpencil_stroke_find_intersections(const RegionView3D *rv3d, bGPDstroke *gps)
// {
//   if (gps == NULL) {
//     return false;
//   }
//   int num_points = gps->totpoints;
//   if (num_points < 3) {
//     return false;
//   }

//   return naive_intersection_algorithm(rv3d, gps);
// }

bGPDstroke *BKE_gpencil_stroke_clip_self(const RegionView3D *rv3d,
                                         const bGPDlayer *gpl,
                                         const bGPDstroke *gps)
{
  bGPDstroke *clip_stroke = NULL;
  ListBase clip_points = {NULL, NULL};
  int num_isect_points = naive_intersection_algorithm(rv3d, gps, &clip_points);
  if (num_isect_points == 0) {
    return clip_stroke;
  }

  /* create new stroke */
  int tot_points = gps->totpoints + 2*num_isect_points;
  clip_stroke = BKE_gpencil_stroke_new(gps->mat_nr, tot_points, 1);

  float vec_p[4];
  vec_p[3] = 1.0f;
  tClipPoint *cpt_curr = clip_points.first;
  for(int i = 0; i < tot_points; i++) {
    bGPDspoint *pt = &clip_stroke->points[i];
    copy_v3_v3(vec_p, &cpt_curr->x);
    mul_m4_v4(rv3d->viewinv, vec_p);
    copy_v3_v3(&pt->x, vec_p);

    /* Set pressure to zero and strength to one */
    pt->pressure = 0.0f;
    pt->strength = 1.0f;

    pt->flag |= GP_SPOINT_SELECT;

    cpt_curr = cpt_curr->next;
  }

  /* free temp data */
  BLI_freelistN(&clip_points);

  /* triangles cache needs to be recalculated */
  BKE_gpencil_stroke_geometry_update(clip_stroke);

  clip_stroke->flag |= GP_STROKE_SELECT;

  /* Delete the old stroke */
  BLI_remlink(&gpl->actframe->strokes, gps);
  BKE_gpencil_free_stroke(gps);

  return clip_stroke;
}

bGPDstroke *BKE_gpencil_fill_stroke_to_outline(const RegionView3D *rv3d,
                                               const bGPDlayer *gpl,
                                               const bGPDstroke *gps)
{
  bGPDstroke *outline_stroke = NULL;
  ListBase clip_points = {NULL, NULL};
  int num_isect_points = naive_intersection_algorithm(rv3d, gps, &clip_points);
  if (num_isect_points == 0) {
    return outline_stroke;
  }

  ListBase outline_points = {NULL, NULL};
  int num_outline_points = gp_outline_walk_algorithm(&clip_points, &outline_points);

  /* create new stroke */
  outline_stroke = BKE_gpencil_stroke_new(gps->mat_nr, num_outline_points, 1);

  float vec_p[4];
  vec_p[3] = 1.0f;
  tClipPoint *cpt_curr = outline_points.first;
  for(int i = 0; i < num_outline_points; i++) {
    bGPDspoint *pt = &outline_stroke->points[i];
    copy_v3_v3(vec_p, &cpt_curr->x);
    mul_m4_v4(rv3d->viewinv, vec_p);
    copy_v3_v3(&pt->x, vec_p);

    /* Set pressure to zero and strength to one */
    pt->pressure = 0.0f;
    pt->strength = 1.0f;

    pt->flag |= GP_SPOINT_SELECT;

    cpt_curr = cpt_curr->next;
  }

  /* free temp data */
  BLI_freelistN(&clip_points);
  BLI_freelistN(&outline_points);

  /* triangles cache needs to be recalculated */
  BKE_gpencil_stroke_geometry_update(outline_stroke);

  outline_stroke->flag |= GP_STROKE_SELECT;

  /* Delete the old stroke */
  BLI_remlink(&gpl->actframe->strokes, gps);
  BKE_gpencil_free_stroke(gps);

  return outline_stroke;
}
