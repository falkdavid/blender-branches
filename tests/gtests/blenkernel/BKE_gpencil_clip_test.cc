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
 * The Original Code is Copyright (C) 2020 Blender Foundation
 * All rights reserved.
 */

#include "BKE_gpencil_clip.h"

#include "MEM_guardedalloc.h"

#include "BLI_math.h"

#include "testing/testing.h"

#define EPS 0.000001

static tClipPoint *create_point(float x, float y)
{
  tClipPoint *new_point = (tClipPoint *)MEM_callocN(sizeof(tClipPoint), "test_create_point");
  new_point->x = x;
  new_point->y = y;
  return new_point;
}

static tClipEdge *create_edge(tClipPoint *p1, tClipPoint *p2)
{
  tClipEdge *new_edge = (tClipEdge *)MEM_callocN(sizeof(tClipEdge), "test_create_edge");
  new_edge->start = p1;
  new_edge->end = p2;
  gp_update_clip_edge_aabb(new_edge);
  if (new_edge->start->x < new_edge->end->x) {
    new_edge->x_dir = 1;
  }
  if (new_edge->start->x == new_edge->end->x && new_edge->start->y < new_edge->end->y) {
    new_edge->x_dir = 1;
  }
  return new_edge;
}

static tClipEdge *create_edge_fl(float x1, float y1, float x2, float y2)
{
  tClipPoint *pt1 = create_point(x1, y1);
  tClipPoint *pt2 = create_point(x2, y2);
  tClipEdge *new_edge = create_edge(pt1, pt2);
  return new_edge;
}

static tClipEvent *create_event(tClipPoint *pt, tClipEdge *edge, short type)
{
  tClipEvent *new_event = (tClipEvent *)MEM_callocN(sizeof(tClipEvent), "test_create_event");
  new_event->pt = pt;
  new_event->edge = edge;
  new_event->type = type;

  return new_event;
}

static void free_point(tClipPoint *p)
{
  if (p == NULL)
    return;
  MEM_freeN(p);
}

static void free_edge(tClipEdge *e)
{
  if (e == NULL)
    return;
  free_point(e->start);
  free_point(e->sweep_pt);
  free_point(e->end);
  MEM_freeN(e);
}

TEST(bentley_ottmann_test, compare_points)
{
  float pt1[2];
  float pt2[2];
  copy_v2_fl2(pt1, 0.0f, 0.1f);
  copy_v2_fl2(pt2, 0.0f, 0.1f);
  ASSERT_EQ(0, gp_compare_points(pt1, pt2));

  copy_v2_fl2(pt1, 0.0f, 0.1f);
  copy_v2_fl2(pt2, 0.0f, 0.0f);
  ASSERT_EQ(1, gp_compare_points(pt1, pt2));

  copy_v2_fl2(pt1, 0.1f, 0.0f);
  copy_v2_fl2(pt2, 0.0f, 0.0f);
  ASSERT_EQ(1, gp_compare_points(pt1, pt2));

  copy_v2_fl2(pt1, 0.0f, -0.1f);
  copy_v2_fl2(pt2, 0.0f, 0.0f);
  ASSERT_EQ(-1, gp_compare_points(pt1, pt2));

  copy_v2_fl2(pt1, 1.0f, -0.1f);
  copy_v2_fl2(pt2, 1.0f, 0.1f);
  ASSERT_EQ(-1, gp_compare_points(pt1, pt2));

  copy_v2_fl2(pt1, -0.1f, 0.0f);
  copy_v2_fl2(pt2, 0.0f, 0.0f);
  ASSERT_EQ(-1, gp_compare_points(pt1, pt2));

  copy_v2_fl2(pt1, -0.5f, 2.0f);
  copy_v2_fl2(pt2, -0.1f, 2.0f);
  ASSERT_EQ(-1, gp_compare_points(pt1, pt2));
}

TEST(bentley_ottmann_test, point_is_right)
{
  float ptA[2];
  float ptB[2];
  float ptC[2];
  copy_v2_fl2(ptA, 0.0f, 0.0f);
  copy_v2_fl2(ptB, 0.1f, 0.0f);
  copy_v2_fl2(ptC, 0.0f, -0.1f);
  ASSERT_EQ(1, gp_point_is_right(ptA, ptB, ptC));
  copy_v2_fl2(ptA, 0.0f, 0.0f);
  copy_v2_fl2(ptB, 1.0f, 1.0f);
  copy_v2_fl2(ptC, 1.0f, -0.1f);
  ASSERT_EQ(1, gp_point_is_right(ptA, ptB, ptC));

  copy_v2_fl2(ptA, 0.0f, 0.0f);
  copy_v2_fl2(ptB, 0.1f, 0.0f);
  copy_v2_fl2(ptC, 0.0f, 0.1f);
  ASSERT_EQ(-1, gp_point_is_right(ptA, ptB, ptC));
  copy_v2_fl2(ptA, 0.0f, 0.0f);
  copy_v2_fl2(ptB, 0.1f, 0.0f);
  copy_v2_fl2(ptC, 0.0f, 0.1f);
  ASSERT_EQ(-1, gp_point_is_right(ptA, ptB, ptC));

  copy_v2_fl2(ptA, 0.0f, 0.0f);
  copy_v2_fl2(ptB, 0.0f, 1.0f);
  copy_v2_fl2(ptC, 0.0f, 0.0f);
  ASSERT_EQ(0, gp_point_is_right(ptA, ptB, ptC));
  copy_v2_fl2(ptA, 0.0f, 0.0f);
  copy_v2_fl2(ptB, 0.0f, 1.0f);
  copy_v2_fl2(ptC, 0.0f, 1.0f);
  ASSERT_EQ(0, gp_point_is_right(ptA, ptB, ptC));
  copy_v2_fl2(ptA, 0.0f, 0.0f);
  copy_v2_fl2(ptB, 0.0f, 1.0f);
  copy_v2_fl2(ptC, 0.0f, 0.5f);
  ASSERT_EQ(0, gp_point_is_right(ptA, ptB, ptC));
}

TEST(bentley_ottmann_test, y_intersept_edge)
{
  tClipEdge *e;
  e = create_edge_fl(0, 0, 1, 1);
  ASSERT_NEAR(0.0f, gp_y_intersept_edge(e, -1.0f), EPS);
  ASSERT_NEAR(0.0f, gp_y_intersept_edge(e, 0.0f), EPS);
  ASSERT_NEAR(1.0f, gp_y_intersept_edge(e, 1.0f), EPS);
  ASSERT_NEAR(1.0f, gp_y_intersept_edge(e, 2.0f), EPS);
  ASSERT_NEAR(0.5f, gp_y_intersept_edge(e, 0.5f), EPS);
  free_edge(e);

  e = create_edge_fl(0, 0, 1, 0);
  ASSERT_NEAR(0.0f, gp_y_intersept_edge(e, -1.0f), EPS);
  ASSERT_NEAR(0.0f, gp_y_intersept_edge(e, 0.0f), EPS);
  ASSERT_NEAR(0.0f, gp_y_intersept_edge(e, 1.0f), EPS);
  ASSERT_NEAR(0.0f, gp_y_intersept_edge(e, 2.0f), EPS);
  ASSERT_NEAR(0.0f, gp_y_intersept_edge(e, 0.5f), EPS);
  free_edge(e);

  e = create_edge_fl(0, -1, 0, 1);
  ASSERT_NEAR(-1.0f, gp_y_intersept_edge(e, -1.0f), EPS);
  ASSERT_NEAR(-1.0f, gp_y_intersept_edge(e, 0.0f), EPS);
  ASSERT_NEAR(1.0f, gp_y_intersept_edge(e, 0.00001f), EPS);
  ASSERT_NEAR(1.0f, gp_y_intersept_edge(e, 1.0f), EPS);
  free_edge(e);
}

TEST(bentley_ottmann_test, edge_is_vertical)
{
  tClipEdge *e0 = create_edge_fl(0, 0, 1, 1);
  tClipEdge *e1 = create_edge_fl(0, 0, 0, 2);
  tClipEdge *e2 = create_edge_fl(0, 0, 0.000001, 1);

  ASSERT_FALSE(gp_edge_is_vertical(e0));
  ASSERT_TRUE(gp_edge_is_vertical(e1));
  ASSERT_FALSE(gp_edge_is_vertical(e2));

  free_edge(e0);
  free_edge(e1);
  free_edge(e2);
}

TEST(bentley_ottmann_test, y_compare_edges_cross)
{
  tClipPoint *p0 = create_point(0, 0);
  tClipPoint *p1 = create_point(2, 1);
  tClipPoint *p2 = create_point(0, 1);
  tClipPoint *p3 = create_point(2, 0);
  tClipPoint *p4 = create_point(1, 0.5);

  tClipEdge *e0 = create_edge(p0, p1);
  tClipEdge *e1 = create_edge(p2, p3);

  e0->sweep_pt = p0;
  e1->sweep_pt = p2;
  ASSERT_EQ(0, gp_y_compare_clip_edges(e0, e0));
  ASSERT_EQ(0, gp_y_compare_clip_edges(e1, e1));
  ASSERT_EQ(-1, gp_y_compare_clip_edges(e0, e1));
  ASSERT_EQ(1, gp_y_compare_clip_edges(e1, e0));
  e0->sweep_pt = p4;
  e1->sweep_pt = p4;
  ASSERT_EQ(1, gp_y_compare_clip_edges(e0, e1));
  ASSERT_EQ(-1, gp_y_compare_clip_edges(e1, e0));
  e0->sweep_pt = p1;
  e1->sweep_pt = p3;
  ASSERT_EQ(1, gp_y_compare_clip_edges(e0, e1));
  ASSERT_EQ(-1, gp_y_compare_clip_edges(e1, e0));

  free_point(p0);
  free_point(p1);
  free_point(p2);
  free_point(p3);
  free_point(p4);
  MEM_freeN(e0);
  MEM_freeN(e1);
}

TEST(bentley_ottmann_test, y_compare_edges_vertical)
{
  tClipPoint *p0 = create_point(0, 0);
  tClipPoint *p1 = create_point(1, 1);
  tClipPoint *p2 = create_point(2, 0);
  tClipPoint *p3 = create_point(0, 2);
  tClipPoint *p4 = create_point(1, 3);
  tClipPoint *p5 = create_point(2, 2);
  tClipPoint *p6 = create_point(0, 4);
  tClipPoint *p7 = create_point(2, 4);

  tClipEdge *e0 = create_edge(p0, p1);
  tClipEdge *e1 = create_edge(p1, p2);
  tClipEdge *e2 = create_edge(p3, p1);
  tClipEdge *e3 = create_edge(p1, p4);
  tClipEdge *e4 = create_edge(p1, p5);
  tClipEdge *e5 = create_edge(p3, p4);
  tClipEdge *e6 = create_edge(p4, p5);
  tClipEdge *e7 = create_edge(p6, p4);
  tClipEdge *e8 = create_edge(p4, p7);

  e0->sweep_pt = p1;
  e2->sweep_pt = p1;
  ASSERT_EQ(-1, gp_y_compare_clip_edges(e0, e2));
  ASSERT_EQ(1, gp_y_compare_clip_edges(e2, e0));
  e1->sweep_pt = p1;
  e4->sweep_pt = p1;
  ASSERT_EQ(-1, gp_y_compare_clip_edges(e1, e4));
  ASSERT_EQ(1, gp_y_compare_clip_edges(e4, e1));

  free_point(p0);
  free_point(p1);
  free_point(p2);
  free_point(p3);
  free_point(p4);
  free_point(p5);
  free_point(p6);
  free_point(p7);
  MEM_freeN(e0);
  MEM_freeN(e1);
  MEM_freeN(e2);
  MEM_freeN(e3);
  MEM_freeN(e4);
  MEM_freeN(e5);
  MEM_freeN(e6);
  MEM_freeN(e7);
  MEM_freeN(e8);
}