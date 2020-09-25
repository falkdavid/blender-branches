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

/** \file
 * \ingroup bli
 */

/**
 * Generates the offset of a polyline by taking the radius of each vertex into account.
 * \param verts: The input vertices. Expected to be a flat 3d array with x,y and the radius.
 * \param num_verts: The number of vertices.
 * \param radius: The radius of the polyline. Every vertex radius is a factor of this radius.
 * \param subdivisions: The number of subdivisions along the end caps and corners (n / 180deg).
 * \param r_offset_verts: Returning array of offset points. This will be a flat 2d array with x and y.
 * \param r_num_offset_verts: Number of returning vertices.
 */
void BLI_polyline_offset(const double *verts,
                         uint num_verts,
                         const double radius,
                         const uint subdivisions,
                         uint start_cap_t,
                         uint end_cap_t,
                         double **r_offset_verts,
                         uint *r_num_offset_verts);