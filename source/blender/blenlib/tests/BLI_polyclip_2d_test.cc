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
}

TEST(polyclip2d, clip_path_insert2)
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
  EXPECT_EQ(node_A->next, node_B);
  EXPECT_EQ(node_B->next, node_C);
  EXPECT_EQ(node_C->next, nullptr);
  EXPECT_EQ(node_A->prev, nullptr);
  EXPECT_EQ(node_B->prev, node_A);
  EXPECT_EQ(node_C->prev, node_B);
}

TEST(polyclip2d, clip_path_copy)
{
  PointList plist = {{1.0, 2.0}, {2.0, 3.0}, {3.0, 4.0}};
  ClipPath list = ClipPath(plist);
  ClipPath list2 = ClipPath(list);

  EXPECT_EQ(list2.size(), 3);

  auto it1 = list.begin();
  auto it2 = list2.begin();
  for (auto elem : plist) {
    EXPECT_NE(*it1, *it2);
    EXPECT_EQ(elem, (*it2)->data);
    ++it1;
    ++it2;
  }
}

TEST(polyclip2d, clip_path_assign)
{
  PointList plist = {{1.0, 2.0}, {2.0, 3.0}, {3.0, 4.0}};
  ClipPath list = ClipPath(plist);
  ClipPath list2 = ClipPath(plist);

  list.insert_back({4.0, 5.0});
  list2 = list;

  auto it = list2.begin();
  for (auto elem : list) {
    EXPECT_EQ(elem->data, (*it)->data);
    ++it;
  }
}

TEST(polyclip2d, clip_path_from_list)
{
  PointList plist = {{1.0, 2.0}, {2.0, 3.0}, {3.0, 4.0}};
  ClipPath list = ClipPath(plist);
  // std::cout << list;

  EXPECT_EQ(list.size(), 3);
  auto it = list.begin();
  for (auto elem : list) {
    EXPECT_EQ(elem->data, (*it)->data);
    ++it;
  }
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

TEST(polyclip2d, clip_path_remove)
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
  list.remove(node_A);

  EXPECT_EQ(list.size(), 2);
  EXPECT_EQ(list.front(), node_B);
  list.remove(node_B);

  EXPECT_EQ(list.size(), 1);
  EXPECT_EQ(list.front(), node_C);
  list.remove(node_C);

  EXPECT_TRUE(list.empty());
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

TEST(polyclip2d, clip_path_get_outer_boundary)
{
  ClipPath path;
  path.insert_back({0.0, 0.0});
  ClipPath::Node *link_a = path.insert_back({0.5, 0.5});
  path.insert_back({1.0, 1.0});
  path.insert_back({0.0, 1.0});
  ClipPath::Node *link_b = path.insert_back({0.5, 0.5});
  path.insert_back({1.0, 0.0});
  path.link(link_a, link_b);

  PointList plist_expected = {
      {0.0, 0.0}, {0.5, 0.5}, {0.0, 1.0}, {1.0, 1.0}, {0.5, 0.5}, {1.0, 0.0}};

  PointList outer = clip_path_get_outer_boundary(path);
  auto it_exp = plist_expected.begin();
  for (auto pt : outer) {
    EXPECT_EQ(pt, *it_exp);
    it_exp++;
  }
}

TEST(polyclip2d, clip_path_get_outer_boundary2)
{
  ClipPath path;
  path.insert_back({0.0, 0.0});
  path.insert_back({1.0, 0.0});
  ClipPath::Node *link_a = path.insert_back({0.5, 0.5});
  path.insert_back({0.0, 1.0});
  path.insert_back({1.0, 1.0});
  ClipPath::Node *link_b = path.insert_back({0.5, 0.5});
  path.link(link_a, link_b);

  PointList plist_expected = {
      {0.0, 0.0}, {0.5, 0.5}, {0.0, 1.0}, {1.0, 1.0}, {0.5, 0.5}, {1.0, 0.0}};

  PointList outer = clip_path_get_outer_boundary(path);
  auto it_exp = plist_expected.begin();
  for (auto pt : outer) {
    EXPECT_EQ(pt, *it_exp);
    it_exp++;
  }
}

TEST(polyclip2d, clip_path_get_outer_boundary3)
{
  PointList plist = {{0, 0}, {1, 2}, {0, 2}, {1, 1}, {0, 1}};
  ClipPath path = point_list_find_intersections_brute_force(plist);
  // std::cout << path << "\n";
  PointList plist_expected = {{0, 0},
                              {0, 1},
                              {0.5, 1},
                              {0.666666666666667, 1.33333333333333},
                              {0, 2},
                              {1, 2},
                              {0.666666666666667, 1.33333333333333},
                              {1, 1},
                              {0.5, 1}};

  PointList outer = clip_path_get_outer_boundary(path);
  auto it_exp = plist_expected.begin();
  for (auto pt : outer) {
    // std::cout << pt << "?=" << *it_exp << std::endl;
    EXPECT_TRUE(double2::compare(pt, *it_exp, FLT_EPSILON));
    it_exp++;
  }
}

TEST(polyclip2d, clip_path_itersect_brute_force)
{
  PointList plist = {{0.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {1.0, 0.0}};
  // PointList plist_expected = {{0.0, 1.0}, {1.0, 2.0}, {2.0, 1.0}, {1.0, 0.0}};

  ClipPath result = point_list_find_intersections_brute_force(plist);
  std::cout << result << "\n";
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
  // std::cout << out << "\n";

  EXPECT_EQ(out.verts.size(), expected_verts.size());
  EXPECT_TRUE(compare_vertlists(out.verts, expected_verts, FLT_EPSILON));
}

TEST(polyclip2d, offset_polyline_simple04)
{
  VertList verts = {{0, 0, 1.0}, {2, 0, 1.0}, {2, 2, 1.0}};
  Polyline pline = Polyline(verts);

  Polyline out = polyline_offset(pline, 1, 1.0, CapType::CAP_ROUND, CapType::CAP_ROUND);
  // std::cout << out << "\n";

  EXPECT_EQ(out.verts.size(), 11);
}

}  // namespace blender::polyclip::tests