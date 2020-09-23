/* Apache License, Version 2.0 */

#include "testing/testing.h"

#include "BLI_double2.hh"
#include "BLI_polyclip_2d.hh"

namespace blender::polyclip::tests {

TEST(polyclip2d, offset_polyline)
{
  VertList verts = {{0, 0, 0}, {1, 0, 0}};
  Polyline pline = Polyline(verts);

  Polyline out = polyline_offset(pline, 0, 1.0, CapType::CAP_ROUND, CapType::CAP_ROUND);
}

}  // namespace blender::polyclip::tests