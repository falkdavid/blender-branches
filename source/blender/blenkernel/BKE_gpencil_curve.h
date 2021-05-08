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

#pragma once

/** \file
 * \ingroup bke
 */

#ifdef __cplusplus
extern "C" {
#endif

struct Main;
struct Object;
struct Scene;
struct bGPDcurve;
struct bGPDlayer;
struct bGPDstroke;
struct bGPDcurve;
struct bGPDcurve_point;
struct bGPdata;
enum eGPStrokeGeoUpdateFlag;

void BKE_gpencil_convert_curve(struct Main *bmain,
                               struct Scene *scene,
                               struct Object *ob_gp,
                               struct Object *ob_cu,
                               const bool use_collections,
                               const float scale_thickness,
                               const float sample);

struct bGPDcurve *BKE_gpencil_stroke_editcurve_generate(struct bGPDstroke *gps,
                                                        const float error_threshold,
                                                        const float corner_angle);
struct bGPDcurve *BKE_gpencil_stroke_editcurve_tagged_segments_update(struct bGPDstroke *gps,
                                                                      const float error_threshold,
                                                                      const float corner_angle);
void BKE_gpencil_stroke_editcurve_regenerate_single(struct bGPDstroke *gps,
                                                    uint32_t start_idx,
                                                    uint32_t end_idx,
                                                    const float error_threshold);
void BKE_gpencil_stroke_refit_curve(struct bGPDstroke *gps,
                                    const float threshold,
                                    const float corner_angle,
                                    const enum eGPStrokeGeoUpdateFlag flag);
void BKE_gpencil_editcurve_stroke_sync_selection(struct bGPdata *gpd,
                                                 struct bGPDstroke *gps,
                                                 struct bGPDcurve *gpc);
void BKE_gpencil_stroke_editcurve_sync_selection(struct bGPdata *gpd,
                                                 struct bGPDstroke *gps,
                                                 struct bGPDcurve *gpc);
void BKE_gpencil_stroke_update_geometry_from_editcurve(struct bGPDstroke *gps,
                                                       const uint resolution,
                                                       const bool is_adaptive,
                                                       const enum eGPStrokeGeoUpdateFlag flag);
bool BKE_gpencil_editcurve_recalculate_handles(struct bGPDstroke *gps);
void BKE_gpencil_editcurve_subdivide(struct bGPDstroke *gps, const int cuts);
int BKE_gpencil_editcurve_dissolve(struct bGPDstroke *gps,
                                   const uint flag,
                                   const bool refit_segments,
                                   const float error_threshold);
void BKE_gpencil_editcurve_simplify_adaptive(struct bGPDstroke *gps, const float threshold);
void BKE_gpencil_editcurve_simplify_fixed(struct bGPDstroke *gps, const int count);
void BKE_gpencil_editcurve_smooth_ex(struct bGPDstroke *gps,
                                     const float factor,
                                     const uint step_size,
                                     const uint repeat,
                                     const bool only_selected,
                                     const bool affect_endpoints,
                                     const bool use_vertex_groups,
                                     const bool invert_weights,
                                     const int deform_group,
                                     const CurveMapping *curve_mapping,
                                     const bool do_positions,
                                     const bool do_pressure,
                                     const bool do_strength);
void BKE_gpencil_editcurve_smooth(struct bGPDstroke *gps,
                                  const float factor,
                                  const uint step_size,
                                  const uint repeat,
                                  const bool only_selected,
                                  const bool affect_endpoints,
                                  const bool do_positions,
                                  const bool do_pressure,
                                  const bool do_strength);
bool BKE_gpencil_editcurve_merge_distance(struct bGPDstroke *gps,
                                          const float threshold,
                                          const bool use_unselected,
                                          const bool refit_segments,
                                          const float error_threshold);
void BKE_gpencil_editcurve_sample(struct bGPDstroke *gps,
                                  const float length,
                                  const bool refit_segments,
                                  const float error_threshold);

#ifdef __cplusplus
}
#endif
