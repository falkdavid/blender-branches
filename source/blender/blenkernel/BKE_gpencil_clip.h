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

#ifndef __BKE_GPENCIL_CLIP_H__
#define __BKE_GPENCIL_CLIP_H__

/** \file
 * \ingroup bke
 */

#ifdef __cplusplus
extern "C" {
#endif

struct WAVL_Node;

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
  /* for sweep line algorithm: sweep crossing point */
  struct tClipPoint *sweep_pt;
  /* flag */
  char flag;
  /* edge points in the x direction */
  bool x_dir;
} tClipEdge;

typedef struct tClipEvent {
  tClipPoint *pt;
  tClipEdge *edge;
  tClipEdge *isect_link_edge;
  char type;
} tClipEvent;

enum CLIP_EVENT_TYPE {
  CLIP_EVENT_START = 1,
  CLIP_EVENT_END = 2,
  CLIP_EVENT_INTERSECTION = 3,
};

/* For testing only */
void gp_update_clip_edge_aabb(struct tClipEdge *edge);
bool gp_points_are_equal(tClipPoint *pointA, tClipPoint *pointB);
short gp_compare_points(const float A[2], const float B[2]);
bool gp_edge_is_vertical(struct tClipEdge *edge);
short gp_point_is_right(const float A[2], const float B[2], const float C[2]);
short gp_compare_clip_points(void *A, void *B);
double gp_y_intercept_edge(tClipEdge *edge, double x);
short gp_y_compare_clip_edges(void *A, void *B);
short gp_compare_clip_events(void *A, void *B);

bool BKE_gpencil_stroke_find_intersections(const struct RegionView3D *rv3d,
                                           struct bGPDstroke *gps);
struct bGPDstroke *BKE_gpencil_stroke_clip_self(const struct bContext *C,
                                                const struct bGPDlayer *gpl,
                                                struct bGPDstroke *gps,
                                                int algorithm);
struct bGPDstroke *BKE_gpencil_stroke_to_outline(const struct bContext *C,
                                                 const struct bGPDlayer *gpl,
                                                 struct bGPDstroke *gps);
struct bGPDstroke *BKE_gpencil_fill_stroke_to_outline_with_holes(const struct bContext *C,
                                                                 const struct bGPDlayer *gpl,
                                                                 struct bGPDstroke *gps);

#ifdef __cplusplus
}
#endif

#endif /*  __BKE_GPENCIL_CLIP_H__ */
