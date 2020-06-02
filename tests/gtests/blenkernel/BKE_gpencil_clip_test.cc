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
  if (new_edge->start->x < new_edge->end->x || new_edge->start->y < new_edge->end->y) {
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

static tClipEvent *create_event(tClipPoint *p1, tClipEdge *edge, short type)
{
  
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

TEST(bentley_ottmann_test, )
