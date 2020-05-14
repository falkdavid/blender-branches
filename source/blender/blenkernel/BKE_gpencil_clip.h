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
