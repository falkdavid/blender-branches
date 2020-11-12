/* Apache License, Version 2.0 */
#include <iostream>

#include "testing/testing.h"

#include "BLI_double2.hh"
#include "BLI_polyclip_2d.hh"

namespace blender::polyclip::tests {

static bool compare_vertlists(const VertList &a, const VertList &b, double limit)
{
  bool same = true;
  auto it_b = b.begin();
  for (auto it_a : a) {
    if (double2::compare_limit(it_a.co, it_b->co, limit) == false) {
      std::cout << it_a.co << " != " << (*it_b).co << "\n";
      same = false;
      break;
    }
    ++it_b;
  }
  return same;
}

static bool compare_clip_paths(const ClipPath &a, const ClipPath &b, double limit)
{
  if (a.size() != b.size()) {
    std::cout << "ClipPath sizes are not equal!" << std::endl;
    return false;
  }

  bool same = true;
  auto it_b = b.begin();
  for (auto it_a = a.begin(); it_a != a.end(); ++it_a) {
    auto node_a = *it_a;
    auto node_b = *it_b;
    bool cmp = double2::compare_limit(node_a->data, node_b->data, limit);
    EXPECT_TRUE(cmp);
    if (!cmp) {
      std::cout << node_a->data << " is not equal to " << node_b->data << " (limit=" << limit
                << ")" << std::endl;
    }

    ++it_b;
  }
  return same;
}

static std::ostream &operator<<(std::ostream &stream, const PointList &plist)
{
  stream << "[";
  for (auto pt : plist) {
    stream << pt << ", ";
  }
  stream << "]";
  return stream;
}

TEST(polyclip2d, linked_chain_set_head)
{
  PointList expected = {{5, 6}, {7, 8}, {9, 0}, {1, 2}, {3, 4}};
  ClipPath cp = ClipPath({{1, 2}, {3, 4}, {5, 6}, {7, 8}, {9, 0}});
  auto it = ClipPath::Iterator::next(cp.begin(), 2);
  cp.shift_head(it);
  EXPECT_TRUE(compare_clip_paths(cp, expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << expected << std::endl;
    std::cout << "Result: " << cp << std::endl;
  }
}

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
  EXPECT_TRUE(compare_clip_paths(outer, plist_expected, FLT_EPSILON));
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
  EXPECT_TRUE(compare_clip_paths(outer, plist_expected, FLT_EPSILON));
}

TEST(polyclip2d, clip_path_get_outer_boundary3)
{
  PointList plist = {{0, 0}, {1, 2}, {0, 2}, {1, 1}, {0, 1}};
  PolyclipBruteForce bf;
  ClipPath path = bf.find_intersections(plist);
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
  EXPECT_TRUE(compare_clip_paths(outer, plist_expected, FLT_EPSILON));
}

TEST(polyclip2d, clip_path_intersect_brute_force)
{
  PointList plist = {{0.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {1.0, 0.0}};
  PointList plist_expected = {
      {0.0, 0.0}, {0.5, 0.5}, {1.0, 1.0}, {0.0, 1.0}, {0.5, 0.5}, {1.0, 0.0}};
  PolyclipBruteForce bf;
  ClipPath result = bf.find_intersections(plist);
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_bentley_ottman01)
{
  PolyclipBentleyOttmann bo;
  PointList plist = {{0, 0}, {1, 1}, {0, 1}, {1, 0}};
  PointList plist_expected = {{0, 0}, {0.5, 0.5}, {1, 1}, {0, 1}, {0.5, 0.5}, {1, 0}};

  ClipPath result = bo.find_intersections(plist);
  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_bentley_ottman02)
{
  PolyclipBentleyOttmann bo;
  PointList plist = {{0, 0}, {1, 2}, {0, 2}, {1, 1}, {0, 1}};
  PointList plist_expected = {{0, 0},
                              {0.5, 1},
                              {(2.0 / 3.0), (4.0 / 3.0)},
                              {1, 2},
                              {0, 2},
                              {(2.0 / 3.0), (4.0 / 3.0)},
                              {1, 1},
                              {0.5, 1},
                              {0, 1}};
  ClipPath result = bo.find_intersections(plist);
  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_bentley_ottman03)
{
  PolyclipBentleyOttmann bo;
  PointList plist = {{0, 0}, {3, 2}, {1, 2}, {3, 1}, {1, 0}, {3, 0}, {0, 2}};
  PointList plist_expected = {{0, 0},
                              {1.5, 1},
                              {(15.0 / 7.0), (10.0 / 7.0)},
                              {3, 2},
                              {1, 2},
                              {(15.0 / 7.0), (10.0 / 7.0)},
                              {3, 1},
                              {(15.0 / 7.0), (4.0 / 7.0)},
                              {1, 0},
                              {3, 0},
                              {(15.0 / 7.0), (4.0 / 7.0)},
                              {1.5, 1},
                              {0, 2}};
  ClipPath result = bo.find_intersections(plist);
  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_bentley_ottman04)
{
  PolyclipBentleyOttmann bo;
  PointList plist = {{0, 0}, {3, 2}, {2, 2}, {3, 0}, {1, 2}, {1, 0}};
  PointList plist_expected = {{0, 0},
                              {1, 2.0 / 3.0},
                              {1.8, 1.2},
                              {2.25, 1.5},
                              {3, 2},
                              {2, 2},
                              {2.25, 1.5},
                              {3, 0},
                              {1.8, 1.2},
                              {1, 2},
                              {1, 2.0 / 3.0},
                              {1, 0}};
  ClipPath result = bo.find_intersections(plist);
  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_bentley_ottman05)
{
  PolyclipBentleyOttmann bo;
  PointList plist = {{4, 3}, {1, 0}, {3, 0}, {0, 3}, {0, 1}, {3, 4}, {1, 4}, {4, 1}};
  PointList plist_expected = {{4, 3},
                              {3, 2},
                              {2, 1},
                              {1, 0},
                              {3, 0},
                              {2, 1},
                              {1, 2},
                              {0, 3},
                              {0, 1},
                              {1, 2},
                              {2, 3},
                              {3, 4},
                              {1, 4},
                              {2, 3},
                              {3, 2},
                              {4, 1}};
  ClipPath result = bo.find_intersections(plist);
  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_bentley_ottman06)
{
  PolyclipBentleyOttmann bo;
  PointList plist = {{0, 0}, {2, 0}, {2, 2}, {4, 2}, {0, 0}};
  PointList plist_expected = {{0, 0}, {2, 0}, {2, 1}, {2, 2}, {4, 2}, {2, 1}, {0, 0}};
  ClipPath result = bo.find_intersections(plist);
  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_bentley_ottman07)
{
  PolyclipBentleyOttmann bo;
  PointList plist = {{0, 0}, {1, 3}, {2, 4}, {3, 3}, {2, 2}, {0, 1}, {1, 4}, {3, 2}, {1, 0}};
  PointList plist_expected = {{0, 0},
                              {0.4, 1.2},
                              {1, 3},
                              {1.5, 3.5},
                              {2, 4},
                              {3, 3},
                              {2.5, 2.5},
                              {2, 2},
                              {0.4, 1.2},
                              {0, 1},
                              {1, 4},
                              {1.5, 3.5},
                              {2.5, 2.5},
                              {3, 2},
                              {1, 0}};
  ClipPath result = bo.find_intersections(plist);
  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_bentley_ottman08)
{
  PolyclipBentleyOttmann bo;
  PointList plist = {{0, 0}, {2, 1}, {0, 1}, {2, 2}, {0, 2}, {2, 3}, {1, 3}, {1, 0}};
  PointList plist_expected = {{0, 0},
                              {1, 0.5},
                              {2, 1},
                              {1, 1},
                              {0, 1},
                              {1, 1.5},
                              {2, 2},
                              {1, 2},
                              {0, 2},
                              {1, 2.5},
                              {2, 3},
                              {1, 3},
                              {1, 2.5},
                              {1, 2},
                              {1, 1.5},
                              {1, 1},
                              {1, 0.5},
                              {1, 0}};
  ClipPath result = bo.find_intersections(plist);
  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

// TEST(polyclip2d, mono_chain_compare_active_chain_greater)
// {
//   PolyclipParkShin ps;
//   ClipPath m1_list = PointList({{0, 0}, {1, 1}});
//   ClipPath m2_list = PointList({{0, 1}, {1, 1}});
//   ClipPath m3_list = PointList({{0, 1}, {1, 0}});
//   PolyclipParkShin::MonotoneChain m1{m1_list.front(), m1_list.back(), nullptr, true};
//   PolyclipParkShin::MonotoneChain m2{m2_list.front(), m2_list.back(), nullptr, true};
//   PolyclipParkShin::MonotoneChain m3{m3_list.front(), m3_list.back(), nullptr, true};

//   EXPECT_TRUE(m1 > m2);
//   EXPECT_FALSE(m2 > m1);

//   m1.advance_front();
//   m2.advance_front();

//   EXPECT_TRUE(m1 > m2);
//   EXPECT_FALSE(m2 > m1);
// }

TEST(polyclip2d, clip_path_intersect_park_shin01)
{
  PointList plist = {{0, 0}, {1, 0}, {2, 0}, {3, 0}, {3, 1}, {3, 2}, {2, 2}};

  PolyclipParkShin ps;
  ps.add_monotone_chains_from_point_list(plist);
  ps.print_mono_chains();
}

TEST(polyclip2d, clip_path_intersect_park_shin02)
{
  PolyclipParkShin ps;
  PointList plist = {{0, 0}, {1, 1}, {0, 1}, {1, 0}};
  PointList plist_expected = {{0, 0}, {0.5, 0.5}, {1, 1}, {0, 1}, {0.5, 0.5}, {1, 0}};
  ps.add_monotone_chains_from_point_list(plist);

  ClipPath result = ps.find_intersections();
  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_park_shin03)
{
  PolyclipParkShin ps;
  PointList plist = {{0, 3}, {3, 0}, {6, 3}, {5, 1}, {1, 1}};
  PointList plist_expected = {
      {0, 3}, {2, 1}, {3, 0}, {4, 1}, {6, 3}, {5, 1}, {4, 1}, {2, 1}, {1, 1}};
  ps.add_monotone_chains_from_point_list(plist);

  ClipPath result = ps.find_intersections();

  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_park_shin04)
{
  PolyclipParkShin ps;
  PointList plist = {{0, 0}, {3, 2}, {2, 2}, {3, 0}, {1, 2}, {1, 0}};
  PointList plist_expected = {{0, 0},
                              {1, 2.0 / 3.0},
                              {1.8, 1.2},
                              {2.25, 1.5},
                              {3, 2},
                              {2, 2},
                              {2.25, 1.5},
                              {3, 0},
                              {1.8, 1.2},
                              {1, 2},
                              {1, 2.0 / 3.0},
                              {1, 0}};
  ps.add_monotone_chains_from_point_list(plist);

  ClipPath result = ps.find_intersections();

  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_park_shin05)
{
  PolyclipParkShin ps;
  PointList plist = {{0, 0}, {1, 3}, {2, 4}, {3, 3}, {2, 2}, {0, 1}, {1, 4}, {3, 2}, {1, 0}};
  PointList plist_expected = {{0, 0},
                              {0.4, 1.2},
                              {1, 3},
                              {1.5, 3.5},
                              {2, 4},
                              {3, 3},
                              {2.5, 2.5},
                              {2, 2},
                              {0.4, 1.2},
                              {0, 1},
                              {1, 4},
                              {1.5, 3.5},
                              {2.5, 2.5},
                              {3, 2},
                              {1, 0}};
  ps.add_monotone_chains_from_point_list(plist);

  ClipPath result = ps.find_intersections();

  EXPECT_TRUE(compare_clip_paths(result, plist_expected, FLT_EPSILON));
  if (HasFailure()) {
    std::cout << "Expext: " << plist_expected << std::endl;
    std::cout << "Result: " << result << std::endl;
  }
}

TEST(polyclip2d, clip_path_intersect_park_shin06)
{
  PolyclipParkShin ps;
  PointList plist = {{4, 4}, {5, 3}, {3, 0}, {5.1, 3}, {6, 1}, {0, 0}, {-1, 0}};

  ps.add_monotone_chains_from_point_list(plist);
  ClipPath result = ps.find_intersections();
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