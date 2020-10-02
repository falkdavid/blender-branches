/* Apache License, Version 2.0 */
#include <iostream>

#include "testing/testing.h"

#include "BLI_double2.hh"
#include "BLI_polyclip_2d.hh"

namespace blender::polyclip::tests {

TEST(polyclip2d, clip_path_insert)
{
  ClipPath list;
  double2 A = {1.0, 2.0};
  double2 B = {2.0, 3.0};
  double2 C = {3.0, 4.0};

  auto *node_A = list.insert_back(A);
  auto *node_B = list.insert_back(B);
  auto *node_C = list.insert_back(C);
  // std::cout << list;

  EXPECT_EQ(list.size(), 3);
  EXPECT_EQ(node_A->data, A);
  EXPECT_EQ(node_B->data, B);
  EXPECT_EQ(node_C->data, C);

  list.remove(node_A);
  // std::cout << list;

  EXPECT_EQ(list.size(), 2);
  EXPECT_EQ(list.front(), node_B);
}

TEST(polyclip2d, clip_path_from_list)
{
  PointList plist = {{1.0, 2.0}, {2.0, 3.0}, {3.0, 4.0}};
  ClipPath list = ClipPath(plist);
  // std::cout << list;

  EXPECT_EQ(list.size(), 3);
}

TEST(polyclip2d, clip_path_search)
{
  double2 A = {1.0, 2.0};
  double2 B = {2.0, 3.0};
  double2 C = {3.0, 4.0};
  PointList plist = {A, B, C};
  ClipPath list = ClipPath(plist);
  // std::cout << list;

  auto node_A = list.search(A);
  EXPECT_EQ(node_A->data, A);

  auto node_B = list.search(B);
  EXPECT_EQ(node_B->data, B);

  auto node_C = list.search(C);
  EXPECT_EQ(node_C->data, C);
}

TEST(polyclip2d, clip_path_pair_iterate)
{
  PointList plist = {{1.0, 2.0}, {2.0, 3.0}, {3.0, 4.0}};
  ClipPath list = ClipPath(plist);

  auto list_it = plist.begin();
  for (auto it = list.begin_pair(); it != list.end_pair(); ++it) {
    auto edge = *it;
    // std::cout << edge.first->data << edge.second->data << "\n";
    EXPECT_EQ(edge.first->data, *list_it);
    ++list_it;
  }
}

static bool compare_vertlists(const VertList &a, const VertList &b, double limit)
{
  bool same = true;
  auto it_b = b.begin();
  for (auto it_a : a) {
    if (double2::compare(it_a.co, it_b->co, limit) == false) {
      std::cout << it_a.co << " != " << (*it_b).co << "\n";
      same = false;
      break;
    }
    ++it_b;
  }
  return same;
}

TEST(polyclip2d, offset_polyline_simple01)
{
  VertList verts = {{0, 0, 1.0}, {1, 0, 1.0}};
  VertList expected_verts = {{1, 1}, {2, 0}, {1, -1}, {0, -1}, {-1, 0}, {0, 1}, {0.00999999, 1}};
  Polyline pline = Polyline(verts);

  Polyline out = polyline_offset(pline, 1, 1.0, CapType::CAP_ROUND, CapType::CAP_ROUND);
  // std::cout << out << "\n";

  EXPECT_EQ(out.verts.size(), expected_verts.size());
  EXPECT_TRUE(compare_vertlists(out.verts, expected_verts, FLT_EPSILON));
}

TEST(polyclip2d, offset_polyline_simple02)
{
  VertList verts = {{0, 0, 1.0}, {1, 0, 1.0}};
  VertList expected_verts = {{1, 1}, {1, -1}, {0, -1}, {0, 1}, {0.00999999, 1}};
  Polyline pline = Polyline(verts);

  Polyline out = polyline_offset(pline, 0, 1.0, CapType::CAP_FLAT, CapType::CAP_FLAT);
  // std::cout << out << "\n";

  EXPECT_EQ(out.verts.size(), expected_verts.size());
  EXPECT_TRUE(compare_vertlists(out.verts, expected_verts, FLT_EPSILON));
}

TEST(polyclip2d, offset_polyline_simple03)
{
  VertList verts = {{0, 0, 1.0}, {1, 0, 1.0}, {2, 0, 1.0}};
  VertList expected_verts = {
      {1, 1}, {2, 1}, {3, 0}, {2, -1}, {1, -1}, {0, -1}, {-1, 0}, {0, 1}, {0.00999999, 1}};
  Polyline pline = Polyline(verts);

  Polyline out = polyline_offset(pline, 1, 1.0, CapType::CAP_ROUND, CapType::CAP_ROUND);
  std::cout << out << "\n";

  EXPECT_EQ(out.verts.size(), expected_verts.size());
  EXPECT_TRUE(compare_vertlists(out.verts, expected_verts, FLT_EPSILON));
}

TEST(polyclip2d, offset_polyline_simple04)
{
  VertList verts = {{0, 0, 1.0}, {2, 0, 1.0}, {2, 2, 1.0}};
  Polyline pline = Polyline(verts);

  Polyline out = polyline_offset(pline, 1, 1.0, CapType::CAP_ROUND, CapType::CAP_ROUND);
  std::cout << out << "\n";

  EXPECT_EQ(out.verts.size(), 11);
}

}  // namespace blender::polyclip::tests