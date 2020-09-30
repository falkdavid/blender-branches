/* Apache License, Version 2.0 */
#include <iostream>

#include "testing/testing.h"

#include "BLI_double2.hh"
#include "BLI_polyclip_2d.hh"

namespace blender::polyclip::tests {

TEST(polyclip2d, triple_linked_list)
{
  TripleLinkedList<double2> list;
  // list.insert_back({1.0, 1.0});
  // list.insert_back({2.0, 2.0});
  // list.insert_back({3.0, 3.0});

  std::cout << list;
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

  Polyline out = polyline_offset(pline, 0, 1.0, CapType::CAP_ROUND, CapType::CAP_ROUND);
  std::cout << out << "\n";

  EXPECT_EQ(out.verts.size(), expected_verts.size());
  EXPECT_TRUE(compare_vertlists(out.verts, expected_verts, FLT_EPSILON));
}

TEST(polyclip2d, offset_polyline_simple02)
{
  VertList verts = {{0, 0, 1.0}, {1, 0, 1.0}};
  VertList expected_verts = {{1, 1}, {1, -1}, {0, -1}, {0, 1}, {0.00999999, 1}};
  Polyline pline = Polyline(verts);

  Polyline out = polyline_offset(pline, 0, 1.0, CapType::CAP_FLAT, CapType::CAP_FLAT);
  std::cout << out << "\n";

  EXPECT_EQ(out.verts.size(), expected_verts.size());
  EXPECT_TRUE(compare_vertlists(out.verts, expected_verts, FLT_EPSILON));
}

TEST(polyclip2d, offset_polyline_simple03)
{
  VertList verts = {{0, 0, 1.0}, {1, 0, 1.0}, {2, 0, 1.0}};
  VertList expected_verts = {
      {1, 1}, {2, 1}, {3, 0}, {2, -1}, {1, -1}, {0, -1}, {-1, 0}, {0, 1}, {0.00999999, 1}};
  Polyline pline = Polyline(verts);

  Polyline out = polyline_offset(pline, 0, 1.0, CapType::CAP_ROUND, CapType::CAP_ROUND);
  std::cout << out << "\n";

  EXPECT_EQ(out.verts.size(), expected_verts.size());
  EXPECT_TRUE(compare_vertlists(out.verts, expected_verts, FLT_EPSILON));
}

TEST(polyclip2d, offset_polyline_simple04)
{
  VertList verts = {{0, 0, 1.0}, {2, 0, 1.0}, {2, 2, 1.0}};
  Polyline pline = Polyline(verts);

  Polyline out = polyline_offset(pline, 0, 1.0, CapType::CAP_ROUND, CapType::CAP_ROUND);
  std::cout << out << "\n";

  // EXPECT_EQ(out.verts.size(), 11);
}

}  // namespace blender::polyclip::tests