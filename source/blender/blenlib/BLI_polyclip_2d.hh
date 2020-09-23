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
 */

#pragma once

/** \file
 * \ingroup bli
 */

#include <list>

#include "BLI_double2.hh"
#include "BLI_double3.hh"

namespace blender::polyclip {

enum CapType {
  CAP_ROUND = 0,
  CAP_FLAT = 1,
  CAP_MAX,
};

struct Vert {
  double3 co;
  double radius;

  Vert() = default;

  Vert(double3 co, double radius = 0) : co(co), radius(radius)
  {
  }

  Vert(double2 co, double radius = 0) : co(co.x, co.y, 0), radius(radius)
  {
  }

  Vert(double x, double y, double z, double radius = 0) : co(x, y, z), radius(radius)
  {
  }
};

typedef std::list<Vert> VertList;

struct Polyline {
  VertList verts;
  uint64_t num_verts;

  Polyline() = default;

  Polyline(VertList &verts) : verts(verts)
  {
    num_verts = verts.size();
  }
};

typedef std::list<Polyline> Polylines;

struct Path : Polyline {
  bool isClosed;
};

typedef std::list<Path> Paths;

struct Polygon {
  Polyline contour;
  Polylines holes;
};

Polyline polyline_offset(Polyline &pline,
                         const uint subdivisions,
                         const double pline_radius,
                         CapType start_cap_t,
                         CapType end_cap_t);

} /* namespace blender::polyclip */