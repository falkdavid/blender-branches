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

#define ISECT_LEFT 0
#define ISECT_RIGHT 1

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
  /* 0 = left; 1 = right */
  bool isect_type;
  /* temporary flag */
  int flag;
} tClipPoint;

/* tClipPoint->flag */
typedef enum tClipPointFlag {
  CP_UNVISITED = 0,
  CP_VISITED = 1,
  CP_CHECKED = 2,
  CP_OUTER_EDGE = 3,
} tClipPointFlag;

typedef struct t2dStrokeEdge {
  /* Listbase functonality */
  struct t2dStrokeEdge *next, *prev;
  /* pointers to start and end */
  struct tClipPoint *start, *end;
  /* AABB */
  float min[2], max[2];
} t2dStrokeEdge;

/* Helper: given that A and B intersect, returns the intersection type */
static bool gp_clip_point_isect_type(const t2dStrokeEdge *A, const t2dStrokeEdge *B)
{
  double v1[2];
  double v2[2];
  sub_v2_v2v2_db(v1, (double *)&A->end->x, (double *)&A->start->x);
  sub_v2_v2v2_db(v2, (double *)&B->end->x, (double *)&A->start->x);
  /* 0 = left; 1 = right */
  return (-v1[0]*v2[1] + v1[1]*v2[0]) < 0.0;
}

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

/* TODO: find a better position to insert hole */
static void gp_insert_hole_into_outer_edge(ListBase *outer_edge, ListBase *hole)
{
  tClipPoint *outer_edge_last = (tClipPoint *)(((LinkData *)outer_edge->last)->data);
  tClipPoint *hole_first = (tClipPoint *)(((LinkData *)hole->first)->data);

  tClipPoint *hole_exit_start = MEM_dupallocN(hole_first);
  tClipPoint *hole_exit_end = MEM_dupallocN(outer_edge_last);

  BLI_addtail(hole, BLI_genericNodeN(hole_exit_start));
  BLI_addtail(hole, BLI_genericNodeN(hole_exit_end));

  /* add hole to outer edge */
  BLI_movelisttolist(outer_edge, hole);  
}

static ListBase *gp_get_inner_edge(tClipPoint *cpt_start, int *r_num_inner_points)
{
  int num_points = 1;
  ListBase *inner_edge = MEM_callocN(sizeof(ListBase), __func__);
  BLI_addtail(inner_edge, BLI_genericNodeN(cpt_start));
  tClipPoint *cpt_curr = cpt_start->isect_link->next;
  while (cpt_curr != cpt_start) {
    BLI_addtail(inner_edge, BLI_genericNodeN(cpt_curr));
    if(cpt_curr->isect_link != NULL) {
      cpt_curr = cpt_curr->isect_link->next;
    }
    else {
      cpt_curr = cpt_curr->next;
    }
    num_points++;
  }

  *r_num_inner_points += num_points;
  
  return inner_edge;
}

static tClipPoint *gp_next_isect(const tClipPoint *cpt, bool flag_visited)
{
  tClipPoint *cpt_curr = cpt->next;
  while (cpt_curr->isect_link == NULL) {
    if (flag_visited == true) {
      cpt_curr->flag = CP_VISITED;
    }
    cpt_curr = cpt_curr->next;
  }
  return cpt_curr;
}

static ListBase *gp_clipPointList_to_outline_and_holes_list(ListBase *points, int tot_points, int *r_num_points)
{
  if (points == NULL || tot_points == 0) {
    return NULL;
  }
  printf("Num tot points: %d\n", tot_points);
  int point_count = 0;
  ListBase outer_edge = {NULL, NULL};
  ListBase outer_isect_points = {NULL, NULL};
  ListBase holes = {NULL, NULL};

  /* Find min x point on outer edge */
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

  /* Find outer edge */
  int num_outer_points = 1;
  tClipPoint *cpt_start = cpt_min;
  BLI_addtail(&outer_edge, BLI_genericNodeN(cpt_start));
  cpt_start->flag = CP_OUTER_EDGE;
  tClipPoint *cpt_curr = cpt_start->next;
  while (cpt_curr != cpt_start) {
    BLI_addtail(&outer_edge, BLI_genericNodeN(cpt_curr));
    cpt_curr->flag = CP_OUTER_EDGE;
    if(cpt_curr->isect_link != NULL) {
      cpt_curr->isect_link->flag = CP_OUTER_EDGE;
      BLI_addtail(&outer_isect_points, BLI_genericNodeN(cpt_curr));
      cpt_curr = cpt_curr->isect_link->next;
    }
    else {
      cpt_curr = cpt_curr->next;
    }
    num_outer_points++;
  }
  printf("Num outer points: %d\n", num_outer_points);

  int num_inner_points = 0;
  LISTBASE_FOREACH(LinkData *, _curr_outer, &outer_isect_points) {
    tClipPoint *curr_outer = (tClipPoint *)_curr_outer->data;
    tClipPoint *cpt_dfs_source = gp_next_isect(curr_outer, true);
    if (cpt_dfs_source->flag == CP_OUTER_EDGE) {
      continue;
    }
    ListBase cpt_stack = {NULL, NULL};
    BLI_addtail(&cpt_stack, BLI_genericNodeN(cpt_dfs_source));
    int stack_size = 1;
    while (stack_size > 0) {
      LinkData *curr_isect_link = (LinkData *)BLI_poptail(&cpt_stack);
      tClipPoint *curr_isect = (tClipPoint *)curr_isect_link->data;
      stack_size--;

      /* search ends if we hit the outer edge or checked node */
      if (curr_isect->flag == CP_OUTER_EDGE || curr_isect->flag == CP_CHECKED) {
        continue;
      }

      /* inner edge found */
      if (curr_isect->flag == CP_VISITED) {
        ListBase *inner_edge = gp_get_inner_edge(curr_isect, &num_inner_points);
        BLI_addtail(&holes, BLI_genericNodeN(inner_edge));
        continue;
      }

      /* check intersection type */
      if (curr_isect->isect_type == ISECT_LEFT) {
        if (curr_isect->isect_link->next->flag == CP_VISITED) {
          /* if next candidate has been visited, try alternative, else
           * mark point as checked */
          if (curr_isect->next->flag == CP_UNVISITED) {
            tClipPoint *next_isect = gp_next_isect(curr_isect, true);
            BLI_addtail(&cpt_stack, BLI_genericNodeN(next_isect));
            stack_size++;
          }
          else {
            curr_isect->flag = curr_isect->isect_link->flag = CP_CHECKED;
            continue;
          }
        }
        else {
          curr_isect = curr_isect->isect_link;
          tClipPoint *next_isect = gp_next_isect(curr_isect, true);
          BLI_addtail(&cpt_stack, BLI_genericNodeN(next_isect));
          stack_size++;
        }
      }
      else {
        if (curr_isect->next->flag == CP_VISITED) {
          curr_isect = curr_isect->isect_link;
          if (curr_isect->next->flag == CP_UNVISITED) {
            tClipPoint *next_isect = gp_next_isect(curr_isect, true);
            BLI_addtail(&cpt_stack, BLI_genericNodeN(next_isect));
            stack_size++;
          }
          else {
            curr_isect->flag = curr_isect->isect_link->flag = CP_CHECKED;
            continue;
          }
        }
        else {
          tClipPoint *next_isect = gp_next_isect(curr_isect, true);
          BLI_addtail(&cpt_stack, BLI_genericNodeN(next_isect));
          stack_size++;
        }
      }

      curr_isect->flag = curr_isect->isect_link->flag = CP_VISITED;
    }
    BLI_freelistN(&cpt_stack);
  }
  printf("Num inner points: %d\n", num_inner_points);

  point_count = num_outer_points + num_inner_points;
  /* form single outline by connecting holes to outer edge */
  LISTBASE_FOREACH(LinkData *, _curr_hole, &holes) {
    ListBase *curr_hole = (ListBase *)_curr_hole->data;
    gp_insert_hole_into_outer_edge(&outer_edge, curr_hole);
    point_count += 2;
  }
  printf("Num points: %d\n", point_count);

  ListBase *result_list = MEM_callocN(sizeof(ListBase), __func__);
  LISTBASE_FOREACH_MUTABLE(LinkData *, _curr, &outer_edge) {
    tClipPoint *curr = (tClipPoint *)_curr->data;
    BLI_addtail(result_list,  MEM_dupallocN(curr));
  }

  /* unlink ends */
  cpt_first->prev = NULL;
  cpt_last->next = NULL;

  /* free temp data */
  BLI_freelistN(&outer_isect_points);
  BLI_freelistN(&outer_edge);
  BLI_freelistN(&holes);

  *r_num_points = point_count;

  return result_list;
}

static int gp_outline_walk_algorithm(ListBase *points, ListBase *outline_points)
{
  int num_outline_points = 1;
  /* find point on the outer edge first */
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

static int naive_intersection_algorithm(ListBase *edges, ListBase *clip_points)
{
  int num_intersections = 0;
  /* check every pair for intersection */
  LISTBASE_FOREACH(t2dStrokeEdge *, edgeA, edges) {
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
          cpt_isectA->isect_type = gp_clip_point_isect_type(edgeA, edgeB);
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
  return num_intersections;
}

bGPDstroke *BKE_gpencil_stroke_clip_self(const RegionView3D *rv3d,
                                         const bGPDlayer *gpl,
                                         bGPDstroke *gps)
{
  bGPDstroke *clip_stroke = NULL;
  ListBase clip_points = {NULL, NULL};
  ListBase edges = {NULL, NULL};
  gp_stroke_to_points_and_edges(gps, rv3d->viewmat, &edges, &clip_points);

  int num_isect_points = naive_intersection_algorithm(&edges, &clip_points);
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
  BLI_freelistN(&edges);
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
                                               bGPDstroke *gps)
{
  bGPDstroke *outline_stroke = NULL;
  ListBase clip_points = {NULL, NULL};
  ListBase edges = {NULL, NULL};
  gp_stroke_to_points_and_edges(gps, rv3d->viewmat, &edges, &clip_points);

  int num_isect_points = naive_intersection_algorithm(&edges, &clip_points);
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
  BLI_freelistN(&edges);
  BLI_freelistN(&clip_points);
  BLI_freelistN(&outline_points);

  /* triangles cache needs to be recalculated */
  BKE_gpencil_stroke_geometry_update(outline_stroke);

  outline_stroke->flag |= GP_STROKE_SELECT | GP_STROKE_CYCLIC;

  /* Delete the old stroke */
  BLI_remlink(&gpl->actframe->strokes, gps);
  BKE_gpencil_free_stroke(gps);

  return outline_stroke;
}

bGPDstroke *BKE_gpencil_fill_stroke_to_outline_with_holes(const RegionView3D *rv3d,
                                                          const bGPDlayer *gpl,
                                                          bGPDstroke *gps)
{
  bGPDstroke *outline_stroke = NULL;
  ListBase clip_points = {NULL, NULL};
  ListBase edges = {NULL, NULL};
  gp_stroke_to_points_and_edges(gps, rv3d->viewmat, &edges, &clip_points);

  int num_isect_points = naive_intersection_algorithm(&edges, &clip_points);
  if (num_isect_points == 0) {
    return outline_stroke;
  }

  int num_outline_points = 0;
  int tot_points = gps->totpoints + 2*num_isect_points;
  ListBase *outline_points = gp_clipPointList_to_outline_and_holes_list(&clip_points, tot_points, &num_outline_points);
  printf("length: %d\n", BLI_listbase_count(outline_points));
  /* create new stroke */
  outline_stroke = BKE_gpencil_stroke_new(gps->mat_nr, num_outline_points, 1);

  float vec_p[4];
  vec_p[3] = 1.0f;
  tClipPoint *cpt_curr = outline_points->first;
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
  BLI_freelistN(&edges);
  BLI_freelistN(&clip_points);
  BLI_freelistN(outline_points);

  /* triangles cache needs to be recalculated */
  BKE_gpencil_stroke_geometry_update(outline_stroke);

  outline_stroke->flag |= GP_STROKE_SELECT | GP_STROKE_CYCLIC;

  /* Delete the old stroke */
  BLI_remlink(&gpl->actframe->strokes, gps);
  BKE_gpencil_free_stroke(gps);

  return outline_stroke;
}
