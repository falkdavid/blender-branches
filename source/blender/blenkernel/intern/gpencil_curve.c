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

/** \file
 * \ingroup bke
 */

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CLG_log.h"

#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_math_vector.h"

#include "BLT_translation.h"

#include "DNA_collection_types.h"
#include "DNA_color_types.h"
#include "DNA_gpencil_types.h"
#include "DNA_material_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_scene_types.h"

#include "BKE_collection.h"
#include "BKE_colortools.h"
#include "BKE_context.h"
#include "BKE_curve.h"
#include "BKE_deform.h"
#include "BKE_gpencil.h"
#include "BKE_gpencil_curve.h"
#include "BKE_gpencil_geom.h"
#include "BKE_main.h"
#include "BKE_material.h"
#include "BKE_object.h"

#include "curve_fit_nd.h"

#include "DEG_depsgraph_query.h"

/* -------------------------------------------------------------------- */
/** \name General helper functions
 * \{ */

static void *array_insert_index(void *arr, uint len, uint elem_size, uint idx)
{
  BLI_assert(elem_size > 0);
  BLI_assert(arr != NULL);
  arr = MEM_recallocN(arr, (len + 1) * elem_size);
  if (idx == len - 1) {
    return arr;
  }
  memcpy(arr + idx + 1, arr + idx, (len - idx) * elem_size);
  memset(arr + idx, 0, elem_size);
  return arr;
}

static void gpencil_editstroke_deselect_all(bGPDcurve *gpc)
{
  for (int i = 0; i < gpc->tot_curve_points; i++) {
    bGPDcurve_point *gpc_pt = &gpc->curve_points[i];
    BezTriple *bezt = &gpc_pt->bezt;
    gpc_pt->flag &= ~GP_CURVE_POINT_SELECT;
    BEZT_DESEL_ALL(bezt);
  }
  gpc->flag &= ~GP_CURVE_SELECT;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Convert to curve object
 * \{ */

/* Helper: Check materials with same color. */
static int gpencil_check_same_material_color(Object *ob_gp,
                                             const float color_stroke[4],
                                             const float color_fill[4],
                                             const bool do_fill,
                                             const bool do_stroke,
                                             Material **r_mat)
{
  int index = -1;
  Material *ma = NULL;
  *r_mat = NULL;
  float color_cu[4];
  float hsv_stroke[4], hsv_fill[4];

  copy_v4_v4(color_cu, color_stroke);
  zero_v3(hsv_stroke);
  rgb_to_hsv_v(color_cu, hsv_stroke);
  hsv_stroke[3] = color_stroke[3];

  copy_v4_v4(color_cu, color_fill);
  zero_v3(hsv_fill);
  rgb_to_hsv_v(color_cu, hsv_fill);
  hsv_fill[3] = color_fill[3];

  bool match_stroke = false;
  bool match_fill = false;

  for (int i = 1; i <= ob_gp->totcol; i++) {
    ma = BKE_object_material_get(ob_gp, i);
    MaterialGPencilStyle *gp_style = ma->gp_style;
    const bool fill = (gp_style->fill_style == GP_MATERIAL_FILL_STYLE_SOLID);
    const bool stroke = (gp_style->fill_style == GP_MATERIAL_STROKE_STYLE_SOLID);

    if (do_fill && !fill) {
      continue;
    }

    if (do_stroke && !stroke) {
      continue;
    }

    /* Check color with small tolerance (better result in HSV). */
    float hsv2[4];
    if (do_fill) {
      zero_v3(hsv2);
      rgb_to_hsv_v(gp_style->fill_rgba, hsv2);
      hsv2[3] = gp_style->fill_rgba[3];
      if (compare_v4v4(hsv_fill, hsv2, 0.01f)) {
        *r_mat = ma;
        index = i - 1;
        match_fill = true;
      }
    }
    else {
      match_fill = true;
    }

    if (do_stroke) {
      zero_v3(hsv2);
      rgb_to_hsv_v(gp_style->stroke_rgba, hsv2);
      hsv2[3] = gp_style->stroke_rgba[3];
      if (compare_v4v4(hsv_stroke, hsv2, 0.01f)) {
        *r_mat = ma;
        index = i - 1;
        match_stroke = true;
      }
    }
    else {
      match_stroke = true;
    }

    /* If match, don't look for more. */
    if (match_stroke || match_fill) {
      break;
    }
  }

  if (!match_stroke || !match_fill) {
    *r_mat = NULL;
    index = -1;
  }

  return index;
}

/* Helper: Add gpencil material using curve material as base. */
static Material *gpencil_add_from_curve_material(Main *bmain,
                                                 Object *ob_gp,
                                                 const float stroke_color[4],
                                                 const float fill_color[4],
                                                 const bool stroke,
                                                 const bool fill,
                                                 int *r_idx)
{
  Material *mat_gp = BKE_gpencil_object_material_new(bmain, ob_gp, "Material", r_idx);
  MaterialGPencilStyle *gp_style = mat_gp->gp_style;

  /* Stroke color. */
  if (stroke) {
    copy_v4_v4(mat_gp->gp_style->stroke_rgba, stroke_color);
    gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
  }

  /* Fill color. */
  if (fill) {
    copy_v4_v4(mat_gp->gp_style->fill_rgba, fill_color);
    gp_style->flag |= GP_MATERIAL_FILL_SHOW;
  }

  /* Check at least one is enabled. */
  if (((gp_style->flag & GP_MATERIAL_STROKE_SHOW) == 0) &&
      ((gp_style->flag & GP_MATERIAL_FILL_SHOW) == 0)) {
    gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
  }

  return mat_gp;
}

/* Helper: Create new stroke section. */
static void gpencil_add_new_points(bGPDstroke *gps,
                                   const float *coord_array,
                                   const float pressure_start,
                                   const float pressure_end,
                                   const int init,
                                   const int totpoints,
                                   const float init_co[3],
                                   const bool last)
{
  BLI_assert(totpoints > 0);

  const float step = 1.0f / ((float)totpoints - 1.0f);
  float factor = 0.0f;
  for (int i = 0; i < totpoints; i++) {
    bGPDspoint *pt = &gps->points[i + init];
    copy_v3_v3(&pt->x, &coord_array[3 * i]);
    /* Be sure the last point is not on top of the first point of the curve or
     * the close of the stroke will produce glitches. */
    if ((last) && (i > 0) && (i == totpoints - 1)) {
      float dist = len_v3v3(init_co, &pt->x);
      if (dist < 0.1f) {
        /* Interpolate between previous point and current to back slightly. */
        bGPDspoint *pt_prev = &gps->points[i + init - 1];
        interp_v3_v3v3(&pt->x, &pt_prev->x, &pt->x, 0.95f);
      }
    }

    pt->strength = 1.0f;
    pt->pressure = interpf(pressure_end, pressure_start, factor);
    factor += step;
  }
}

/* Helper: Get the first collection that includes the object. */
static Collection *gpencil_get_parent_collection(Scene *scene, Object *ob)
{
  Collection *mycol = NULL;
  FOREACH_SCENE_COLLECTION_BEGIN (scene, collection) {
    LISTBASE_FOREACH (CollectionObject *, cob, &collection->gobject) {
      if ((mycol == NULL) && (cob->ob == ob)) {
        mycol = collection;
      }
    }
  }
  FOREACH_SCENE_COLLECTION_END;

  return mycol;
}
static int gpencil_get_stroke_material_fromcurve(
    Main *bmain, Object *ob_gp, Object *ob_cu, bool *do_stroke, bool *do_fill)
{
  Curve *cu = (Curve *)ob_cu->data;

  Material *mat_gp = NULL;
  Material *mat_curve_stroke = NULL;
  Material *mat_curve_fill = NULL;

  float color_stroke[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float color_fill[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  /* If the curve has 2 materials, the first is considered as Fill and the second as Stroke.
   * If the has only one material, if the name contains _stroke, the is used
   * as stroke, else as fill.*/
  if (ob_cu->totcol >= 2) {
    *do_stroke = true;
    *do_fill = true;
    mat_curve_fill = BKE_object_material_get(ob_cu, 1);
    mat_curve_stroke = BKE_object_material_get(ob_cu, 2);
  }
  else if (ob_cu->totcol == 1) {
    mat_curve_stroke = BKE_object_material_get(ob_cu, 1);
    if ((mat_curve_stroke) && (strstr(mat_curve_stroke->id.name, "_stroke") != NULL)) {
      *do_stroke = true;
      *do_fill = false;
      mat_curve_fill = NULL;
    }
    else {
      *do_stroke = false;
      *do_fill = true;
      /* Invert materials. */
      mat_curve_fill = mat_curve_stroke;
      mat_curve_stroke = NULL;
    }
  }
  else {
    /* No materials in the curve. */
    *do_fill = false;
    return -1;
  }

  if (mat_curve_stroke) {
    copy_v4_v4(color_stroke, &mat_curve_stroke->r);
  }
  if (mat_curve_fill) {
    copy_v4_v4(color_fill, &mat_curve_fill->r);
  }

  int r_idx = gpencil_check_same_material_color(
      ob_gp, color_stroke, color_fill, *do_stroke, *do_fill, &mat_gp);

  if ((ob_gp->totcol < r_idx) || (r_idx < 0)) {
    mat_gp = gpencil_add_from_curve_material(
        bmain, ob_gp, color_stroke, color_fill, *do_stroke, *do_fill, &r_idx);
  }

  /* Set fill and stroke depending of curve type (3D or 2D). */
  if ((cu->flag & CU_3D) || ((cu->flag & (CU_FRONT | CU_BACK)) == 0)) {
    mat_gp->gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
    mat_gp->gp_style->flag &= ~GP_MATERIAL_FILL_SHOW;
  }
  else {
    mat_gp->gp_style->flag &= ~GP_MATERIAL_STROKE_SHOW;
    mat_gp->gp_style->flag |= GP_MATERIAL_FILL_SHOW;
  }

  return r_idx;
}

/* Helper: Convert one spline to grease pencil stroke. */
static void gpencil_convert_spline(Main *bmain,
                                   Object *ob_gp,
                                   Object *ob_cu,
                                   const float scale_thickness,
                                   const float sample,
                                   bGPDframe *gpf,
                                   Nurb *nu)
{
  bGPdata *gpd = (bGPdata *)ob_gp->data;
  bool cyclic = true;

  /* Create Stroke. */
  bGPDstroke *gps = MEM_callocN(sizeof(bGPDstroke), "bGPDstroke");
  gps->thickness = 1.0f;
  gps->fill_opacity_fac = 1.0f;
  gps->hardeness = 1.0f;
  gps->uv_scale = 1.0f;

  ARRAY_SET_ITEMS(gps->aspect_ratio, 1.0f, 1.0f);
  ARRAY_SET_ITEMS(gps->caps, GP_STROKE_CAP_ROUND, GP_STROKE_CAP_ROUND);
  gps->inittime = 0.0f;

  gps->flag &= ~GP_STROKE_SELECT;
  gps->flag |= GP_STROKE_3DSPACE;

  gps->mat_nr = 0;
  /* Count total points
   * The total of points must consider that last point of each segment is equal to the first
   * point of next segment.
   */
  int totpoints = 0;
  int segments = 0;
  int resolu = nu->resolu + 1;
  segments = nu->pntsu;
  if ((nu->flagu & CU_NURB_CYCLIC) == 0) {
    segments--;
    cyclic = false;
  }
  totpoints = (resolu * segments) - (segments - 1);

  /* Materials
   * Notice: The color of the material is the color of viewport and not the final shader color.
   */
  bool do_stroke, do_fill;
  int r_idx = gpencil_get_stroke_material_fromcurve(bmain, ob_gp, ob_cu, &do_stroke, &do_fill);
  CLAMP_MIN(r_idx, 0);

  /* Assign material index to stroke. */
  gps->mat_nr = r_idx;

  /* Add stroke to frame.*/
  BLI_addtail(&gpf->strokes, gps);

  float *coord_array = NULL;
  float init_co[3];

  switch (nu->type) {
    case CU_POLY: {
      /* Allocate memory for storage points. */
      gps->totpoints = nu->pntsu;
      gps->points = MEM_callocN(sizeof(bGPDspoint) * gps->totpoints, "gp_stroke_points");
      /* Increase thickness for this type. */
      gps->thickness = 10.0f;

      /* Get all curve points */
      for (int s = 0; s < gps->totpoints; s++) {
        BPoint *bp = &nu->bp[s];
        bGPDspoint *pt = &gps->points[s];
        copy_v3_v3(&pt->x, bp->vec);
        pt->pressure = bp->radius;
        pt->strength = 1.0f;
      }
      break;
    }
    case CU_BEZIER: {
      /* Allocate memory for storage points. */
      gps->totpoints = totpoints;
      gps->points = MEM_callocN(sizeof(bGPDspoint) * gps->totpoints, "gp_stroke_points");

      int init = 0;
      resolu = nu->resolu + 1;
      segments = nu->pntsu;
      if ((nu->flagu & CU_NURB_CYCLIC) == 0) {
        segments--;
      }
      /* Get all interpolated curve points of Beziert */
      for (int s = 0; s < segments; s++) {
        int inext = (s + 1) % nu->pntsu;
        BezTriple *prevbezt = &nu->bezt[s];
        BezTriple *bezt = &nu->bezt[inext];
        bool last = (bool)(s == segments - 1);

        coord_array = MEM_callocN((size_t)3 * resolu * sizeof(float), __func__);

        for (int j = 0; j < 3; j++) {
          BKE_curve_forward_diff_bezier(prevbezt->vec[1][j],
                                        prevbezt->vec[2][j],
                                        bezt->vec[0][j],
                                        bezt->vec[1][j],
                                        coord_array + j,
                                        resolu - 1,
                                        sizeof(float[3]));
        }
        /* Save first point coordinates. */
        if (s == 0) {
          copy_v3_v3(init_co, &coord_array[0]);
        }
        /* Add points to the stroke */
        float radius_start = prevbezt->radius * scale_thickness;
        float radius_end = bezt->radius * scale_thickness;

        gpencil_add_new_points(
            gps, coord_array, radius_start, radius_end, init, resolu, init_co, last);
        /* Free memory. */
        MEM_SAFE_FREE(coord_array);

        /* As the last point of segment is the first point of next segment, back one array
         * element to avoid duplicated points on the same location.
         */
        init += resolu - 1;
      }
      break;
    }
    case CU_NURBS: {
      if (nu->pntsv == 1) {

        int nurb_points;
        if (nu->flagu & CU_NURB_CYCLIC) {
          resolu++;
          nurb_points = nu->pntsu * resolu;
        }
        else {
          nurb_points = (nu->pntsu - 1) * resolu;
        }
        /* Get all curve points. */
        coord_array = MEM_callocN(sizeof(float[3]) * nurb_points, __func__);
        BKE_nurb_makeCurve(nu, coord_array, NULL, NULL, NULL, resolu, sizeof(float[3]));

        /* Allocate memory for storage points. */
        gps->totpoints = nurb_points;
        gps->points = MEM_callocN(sizeof(bGPDspoint) * gps->totpoints, "gp_stroke_points");

        /* Add points. */
        gpencil_add_new_points(gps, coord_array, 1.0f, 1.0f, 0, gps->totpoints, init_co, false);

        MEM_SAFE_FREE(coord_array);
      }
      break;
    }
    default: {
      break;
    }
  }
  /* Cyclic curve, close stroke. */
  if (cyclic) {
    BKE_gpencil_stroke_close(gps);
  }

  if (sample > 0.0f) {
    BKE_gpencil_stroke_sample(gpd, gps, sample, false);
  }

  /* Recalc fill geometry. */
  BKE_gpencil_stroke_geometry_update(gpd, gps, GP_GEO_UPDATE_DEFAULT);
}

/**
 * Convert a curve object to grease pencil stroke.
 *
 * \param bmain: Main thread pointer
 * \param scene: Original scene.
 * \param ob_gp: Grease pencil object to add strokes.
 * \param ob_cu: Curve to convert.
 * \param use_collections: Create layers using collection names.
 * \param scale_thickness: Scale thickness factor.
 * \param sample: Sample distance, zero to disable.
 */
void BKE_gpencil_convert_curve(Main *bmain,
                               Scene *scene,
                               Object *ob_gp,
                               Object *ob_cu,
                               const bool use_collections,
                               const float scale_thickness,
                               const float sample)
{
  if (ELEM(NULL, ob_gp, ob_cu) || (ob_gp->type != OB_GPENCIL) || (ob_gp->data == NULL)) {
    return;
  }

  Curve *cu = (Curve *)ob_cu->data;
  bGPdata *gpd = (bGPdata *)ob_gp->data;
  bGPDlayer *gpl = NULL;

  /* If the curve is empty, cancel. */
  if (cu->nurb.first == NULL) {
    return;
  }

  /* Check if there is an active layer. */
  if (use_collections) {
    Collection *collection = gpencil_get_parent_collection(scene, ob_cu);
    if (collection != NULL) {
      gpl = BKE_gpencil_layer_named_get(gpd, collection->id.name + 2);
      if (gpl == NULL) {
        gpl = BKE_gpencil_layer_addnew(gpd, collection->id.name + 2, true);
      }
    }
  }

  if (gpl == NULL) {
    gpl = BKE_gpencil_layer_active_get(gpd);
    if (gpl == NULL) {
      gpl = BKE_gpencil_layer_addnew(gpd, DATA_("GP_Layer"), true);
    }
  }

  /* Check if there is an active frame and add if needed. */
  bGPDframe *gpf = BKE_gpencil_layer_frame_get(gpl, CFRA, GP_GETFRAME_ADD_COPY);

  /* Read all splines of the curve and create a stroke for each. */
  LISTBASE_FOREACH (Nurb *, nu, &cu->nurb) {
    gpencil_convert_spline(bmain, ob_gp, ob_cu, scale_thickness, sample, gpf, nu);
  }

  /* Merge any similar material. */
  int removed = 0;
  BKE_gpencil_merge_materials(ob_gp, 0.001f, 0.001f, 0.001f, &removed);

  /* Remove any unused slot. */
  int actcol = ob_gp->actcol;

  for (int slot = 1; slot <= ob_gp->totcol; slot++) {
    while (slot <= ob_gp->totcol && !BKE_object_material_slot_used(ob_gp->data, slot)) {
      ob_gp->actcol = slot;
      BKE_object_material_slot_remove(bmain, ob_gp);

      if (actcol >= slot) {
        actcol--;
      }
    }
  }

  ob_gp->actcol = actcol;

  /* Tag for recalculation */
  DEG_id_tag_update(&gpd->id, ID_RECALC_GEOMETRY | ID_RECALC_COPY_ON_WRITE);
  DEG_id_tag_update(&ob_gp->id, ID_RECALC_GEOMETRY);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Edit-Curve Kernel Functions
 * \{ */

typedef struct tCurveFitPoint {
  float x, y, z;
  float pressure;
  float strength;
  float color[4];
} tCurveFitPoint;

#define CURVE_FIT_POINT_DIM 9

/* We treat pressure, strength and vertex colors as dimensions in the curve fitting like the
 * position. But this means that e.g. a high pressure will change the shape of the
 * higher-dimensional fitted curve which will in turn change the shape of the projected
 * 3-dimensional curve. But we don't really want the pressure or something else than the position
 * to influence the shape. So this COORD_FITTING_INFLUENCE will "dampen" the effect of the other
 * attributes affecting the shape. Ideally, we fit the other attributes separate from the position.
 */
#define COORD_FITTING_INFLUENCE 20.0f

typedef struct tCurveAttribute {
  /* Value array of shape values[values_len][dim]. */
  float *values;
  uint32_t values_len;
  uint32_t dim;

  /* Output of cubic curve fitting. */
  float *cubic_array;
  uint32_t cubic_array_len;
  uint32_t *cubic_orig_index;
  uint32_t *corners_index_array;

  /* Array of t-values of size cubic_array_len * 2 */
  float *t_values;

} tCurveAttribute;

typedef struct tCurveSegment {
  struct tCurveSegment *next, *prev;

  /* Input of the fitting process. Flat array of stroke point values (x, y, z, pressure, etc.). */
  float *point_array;
  uint32_t point_array_len;
  uint32_t dim;

  /* Array of tCurveAttribute with length num_attributes. */
  tCurveAttribute *attributes;
  uint32_t num_attributes;

  /* Output of the fitting process. */
  bGPDcurve_point *curve_points;

  bool is_cyclic;
} tCurveSegment;

/**
 * Find the stationary points of a cubic bezier curve.
 * Calcualtes the t-values (factor along curve) of the stationary points of a cubic bezier curve by
 * finding the roots of the first derivative. If no roots where found the function returns false.
 * Otherwise if one of the roots was found, one of r_t1 or r_t2 will be NaN and the other will
 * contain the t-value of the root found. If both roots were found, both r_t1 and r_t2 will contain
 * a value.
 * \param p1, p2, p3, p4: Points of the cubic bezier curve.
 * \param r_t1, r_t2: Return t-values (factor along curve).
 */
static bool find_cubic_bezier_stationary_points(
    const float p1, const float p2, const float p3, const float p4, float *r_t1, float *r_t2)
{
  float a = 6.0f * (-p1 + 3.0f * p2 - 3.0f * p3 + p4);
  if (IS_EQF(a, 0.0f)) {
    /* Special edge-case that we have to handle seperately.*/
    if ((p1 == p4) && (p2 == p3)) {
      if (p1 == p2) {
        *r_t1 = NAN;
        *r_t2 = NAN;
        return false;
      }
      *r_t1 = 0.5f;
      *r_t2 = NAN;
      return true;
    }
    /* Denominator is zero. No roots. */
    return false;
  }

  float x = p4 * (p1 - p2) - p3 * (p1 + p2) + p2 * p2 + p3 * p3;
  if (x < 0.0f) {
    /* Negative number under square root. No real roots. */
    return false;
  }

  float s = 6.0f * sqrtf(x);
  float b = 6.0f * (p1 - 2.0f * p2 + p3);

  float t1 = (s - b) / a;
  float t2 = -(s + b) / a;

  /* Discard root outside of limits. */
  *r_t1 = IN_RANGE_INCL(t1, 0.0f, 1.0f) ? t1 : NAN;
  *r_t2 = IN_RANGE_INCL(t2, 0.0f, 1.0f) ? t2 : NAN;

  return !(*r_t1 == NAN) || !(*r_t2 == NAN);
}

static void gpencil_free_curve_segment(tCurveSegment *tcs)
{
  if (tcs == NULL) {
    return;
  }

  if (tcs->point_array != NULL && tcs->point_array_len > 0) {
    MEM_freeN(tcs->point_array);
  }
  if (tcs->curve_points != NULL && tcs->cubic_array_len > 0) {
    MEM_freeN(tcs->curve_points);
  }
  if (tcs->cubic_array != NULL && tcs->cubic_array_len > 0) {
    free(tcs->cubic_array);
  }
  if (tcs->cubic_orig_index != NULL && tcs->cubic_array_len > 0) {
    free(tcs->cubic_orig_index);
  }
  if (tcs->corners_index_array != NULL && tcs->corners_index_len > 0) {
    free(tcs->corners_index_array);
  }

  MEM_freeN(tcs);
}

static bool gpencil_is_segment_tagged(bGPDstroke *gps, const uint32_t cpt_start)
{
  bGPDcurve *gpc = gps->editcurve;
  const bool is_cyclic = (gps->flag & GP_STROKE_CYCLIC);
  const uint32_t cpt_end = (cpt_start + 1) % gpc->tot_curve_points;

  const uint32_t start_pt_idx = gpc->curve_points[cpt_start].point_index;
  const uint32_t end_pt_idx = gpc->curve_points[cpt_end].point_index;

  if (!is_cyclic && (cpt_start == gpc->tot_curve_points - 1)) {
    return false;
  }

  /* Check endpoints of segment frist. */
  if ((gps->points[start_pt_idx].flag & GP_SPOINT_TAG) ||
      (gps->points[end_pt_idx].flag & GP_SPOINT_TAG)) {
    return true;
  }

  /* Iterate over all stroke points in between and check if any point is tagged. */
  if (is_cyclic && (cpt_start == gpc->tot_curve_points - 1)) {
    for (int i = start_pt_idx + 1; i < gps->totpoints; i++) {
      bGPDspoint *pt = &gps->points[i];
      if ((pt->flag & GP_SPOINT_TAG)) {
        return true;
      }
    }
  }
  else {
    for (int i = start_pt_idx + 1; i < end_pt_idx - 1; i++) {
      bGPDspoint *pt = &gps->points[i];
      if ((pt->flag & GP_SPOINT_TAG)) {
        return true;
      }
    }
  }

  return false;
}

static int gpencil_count_tagged_curve_segments(bGPDstroke *gps)
{
  bGPDcurve *gpc = gps->editcurve;
  /* Handle single point edgecase. */
  if (gpc->tot_curve_points == 1) {
    return (gps->points[0].flag & GP_SPOINT_TAG) ? 1 : 0;
  }

  /* Iterate over points and check if segment is tagged. */
  int count = 0;
  for (int i = 0; i < gpc->tot_curve_points; i++) {
    if (gpencil_is_segment_tagged(gps, i)) {
      count++;
    }
  }

  return count;
}

static void gpencil_clear_point_tag(bGPDstroke *gps)
{
  for (int i = 0; i < gps->totpoints; i++) {
    bGPDspoint *pt = &gps->points[i];
    pt->flag &= ~GP_SPOINT_TAG;
  }
}

static bGPDcurve *gpencil_stroke_editcurve_generate_edgecases(bGPDstroke *gps)
{
  BLI_assert(gps->totpoints < 3);

  if (gps->totpoints == 1) {
    bGPDcurve *editcurve = BKE_gpencil_stroke_editcurve_new(1);
    bGPDspoint *pt = &gps->points[0];
    bGPDcurve_point *cpt = &editcurve->curve_points[0];
    BezTriple *bezt = &cpt->bezt;

    /* This is not the actual radius, but it is good enough for what we need. */
    float stroke_radius = gps->thickness / 1000.0f;
    /* Handles are twice as long as the radius of the point. */
    float offset = (pt->pressure * stroke_radius) * 2.0f;

    float tmp_vec[3];
    for (int j = 0; j < 3; j++) {
      copy_v3_v3(tmp_vec, &pt->x);
      /* Move handles along the x-axis away from the control point */
      tmp_vec[0] += (float)(j - 1) * offset;
      copy_v3_v3(bezt->vec[j], tmp_vec);
    }

    cpt->pressure = pt->pressure;
    cpt->strength = pt->strength;
    copy_v4_v4(cpt->vert_color, pt->vert_color);

    /* default handle type */
    bezt->h1 = HD_FREE;
    bezt->h2 = HD_FREE;

    cpt->point_index = 0;

    return editcurve;
  }
  if (gps->totpoints == 2) {
    bGPDcurve *editcurve = BKE_gpencil_stroke_editcurve_new(2);
    bGPDspoint *first_pt = &gps->points[0];
    bGPDspoint *last_pt = &gps->points[1];

    float length = len_v3v3(&first_pt->x, &last_pt->x);
    float offset = length / 3;
    float dir[3];
    sub_v3_v3v3(dir, &last_pt->x, &first_pt->x);

    for (int i = 0; i < 2; i++) {
      bGPDspoint *pt = &gps->points[i];
      bGPDcurve_point *cpt = &editcurve->curve_points[i];
      BezTriple *bezt = &cpt->bezt;

      float tmp_vec[3];
      for (int j = 0; j < 3; j++) {
        copy_v3_v3(tmp_vec, dir);
        normalize_v3_length(tmp_vec, (float)(j - 1) * offset);
        add_v3_v3v3(bezt->vec[j], &pt->x, tmp_vec);
      }

      cpt->pressure = pt->pressure;
      cpt->strength = pt->strength;
      copy_v4_v4(cpt->vert_color, pt->vert_color);

      /* default handle type */
      bezt->h1 = HD_VECT;
      bezt->h2 = HD_VECT;

      cpt->point_index = 0;
    }

    return editcurve;
  }

  return NULL;
}

/**
 *
 *
 */
static int fit_curve_to_nd_attribute_ex(const float *values,
                                        const uint values_len,
                                        const uint dims,
                                        const float error_threshold)
{
  float *cubic_array;
  uint cubic_array_len;
  uint *cubic_orig_index_array;

  int calc_flag = CURVE_FIT_CALC_HIGH_QUALIY;
  int r = curve_fit_cubic_to_points_refit_fl(values,
                                             values_len,
                                             dims,
                                             error_threshold,
                                             calc_flag,
                                             NULL,
                                             0,
                                             M_PI,
                                             &cubic_array,
                                             &cubic_array_len,
                                             &cubic_orig_index_array,
                                             NULL,
                                             NULL);
  if (r != 0 || cubic_array_len < 1) {
    MEM_SAFE_FREE(cubic_array);
    MEM_SAFE_FREE(cubic_orig_index_array);
    return -1;
  }

  uint num_curve_segments = cubic_array_len - 1;
  for (int i = 0; i < num_curve_segments; i++) {
    float(*curr_elem)[3][dims] = &cubic_array[i];
    float(*next_elem)[3][dims] = &cubic_array[i + 1];

    /* Get the four control points of one segment. */
    float(*p1)[dims] = &curr_elem[1];
    float(*p2)[dims] = &curr_elem[2];
    float(*p3)[dims] = &next_elem[0];
    float(*p4)[dims] = &next_elem[1];

    /* Skip the first */
    for (int d = 1; d < dims; d++) {
      float t1 = NAN, t2 = NAN;
      find_cubic_bezier_stationary_points(*p1[d], *p2[d], *p3[d], *p4[d], &t1, &t2);
      if (t1 != NAN) {
      }
      if (t2 != NAN) {
      }
    }
  }
}

static tCurveSegment *gpencil_fit_curve_to_stroke_points_ex(bGPDstroke *gps,
                                                            const uint32_t start_idx,
                                                            const uint32_t end_idx,
                                                            const float error_threshold,
                                                            const float corner_angle,
                                                            const bool is_cyclic,
                                                            const eGPStrokeGeoUpdateFlag flag)
{
  const uint32_t length = end_idx - start_idx + 1;

  tCurveSegment *tcs = MEM_callocN(sizeof(tCurveSegment), __func__);
  tcs->is_cyclic = is_cyclic;

  for (int i = 0; i < length; i++) {
    bGPDspoint *pt = &gps->points[i + start_idx];
  }
}

/* Helper: Generates a tCurveSegment using the stroke points in gps from the start index up to
 * the end index. */
static tCurveSegment *gpencil_fit_curve_to_points_ex(bGPDstroke *gps,
                                                     const float diag_length,
                                                     const float error_threshold,
                                                     const float corner_angle,
                                                     const uint32_t start_idx,
                                                     const uint32_t end_idx,
                                                     const bool is_cyclic)
{
  const uint32_t length = end_idx - start_idx + 1;

  tCurveSegment *tcs = MEM_callocN(sizeof(tCurveSegment), __func__);
  tcs->point_array_len = length * CURVE_FIT_POINT_DIM;
  tcs->point_array = MEM_callocN(sizeof(float) * tcs->point_array_len, __func__);

  float tmp_vec[3];
  for (int i = 0; i < length; i++) {
    bGPDspoint *pt = &gps->points[i + start_idx];
    int row = i * CURVE_FIT_POINT_DIM;
    tCurveFitPoint *cfpt = (tCurveFitPoint *)&tcs->point_array[row];
    /* normalize coordinate to 0..1 */
    sub_v3_v3v3(tmp_vec, &pt->x, gps->boundbox_min);
    mul_v3_v3fl(&cfpt->x, tmp_vec, COORD_FITTING_INFLUENCE / diag_length);
    cfpt->pressure = pt->pressure / diag_length;

    /* strength and color are already normalized */
    cfpt->strength = pt->strength / diag_length;
    mul_v4_v4fl(cfpt->color, pt->vert_color, 1.0f / diag_length);
  }

  int calc_flag = CURVE_FIT_CALC_HIGH_QUALIY;
  if (is_cyclic) {
    calc_flag |= CURVE_FIT_CALC_CYCLIC;
  }
  int r = curve_fit_cubic_to_points_refit_fl(tcs->point_array,
                                             length,
                                             CURVE_FIT_POINT_DIM,
                                             error_threshold,
                                             calc_flag,
                                             NULL,
                                             0,
                                             corner_angle,
                                             &tcs->cubic_array,
                                             &tcs->cubic_array_len,
                                             &tcs->cubic_orig_index,
                                             &tcs->corners_index_array,
                                             &tcs->corners_index_len);

  if (r != 0 || tcs->cubic_array_len < 1) {
    gpencil_free_curve_segment(tcs);
    return NULL;
  }

  tcs->curve_points = MEM_callocN(sizeof(bGPDcurve_point) * tcs->cubic_array_len, __func__);

  for (int i = 0; i < tcs->cubic_array_len; i++) {
    bGPDcurve_point *cpt = &tcs->curve_points[i];
    BezTriple *bezt = &cpt->bezt;
    tCurveFitPoint *curve_point = (tCurveFitPoint *)&tcs->cubic_array[i * 3 * CURVE_FIT_POINT_DIM];

    /* Loop over bezier triple and fill coordinates. */
    for (int j = 0; j < 3; j++) {
      float *bez = &curve_point[j].x;
      madd_v3_v3v3fl(bezt->vec[j], gps->boundbox_min, bez, diag_length / COORD_FITTING_INFLUENCE);
    }

    tCurveFitPoint *ctrl_point = &curve_point[1];
    cpt->pressure = ctrl_point->pressure * diag_length;
    cpt->strength = ctrl_point->strength * diag_length;
    mul_v4_v4fl(cpt->vert_color, ctrl_point->color, diag_length);

    /* Default handle type */
    bezt->h1 = HD_ALIGN;
    bezt->h2 = HD_ALIGN;

    /* Make sure to add the start index. */
    cpt->point_index = tcs->cubic_orig_index[i] + start_idx;

    /* Select the curve point if the original stroke point was selected. */
    if (gps->points[cpt->point_index].flag & GP_SPOINT_SELECT) {
      cpt->flag |= GP_CURVE_POINT_SELECT;
      BEZT_SEL_ALL(bezt);
    }
  }

  /* Set handle type to HD_FREE for corner handles. Ignore first and last. */
  if (tcs->corners_index_array != NULL) {
    uint i_start = 0, i_end = tcs->corners_index_len;

    if ((tcs->corners_index_len >= 2) && (calc_flag & CURVE_FIT_CALC_CYCLIC) == 0) {
      i_start += 1;
      i_end -= 1;
    }

    for (uint i = i_start; i < i_end; i++) {
      bGPDcurve_point *cpt = &tcs->curve_points[tcs->corners_index_array[i]];
      BezTriple *bezt = &cpt->bezt;
      bezt->h1 = HD_FREE;
      bezt->h2 = HD_FREE;
    }
  }

  return tcs;
}

static float get_point_weight(MDeformVert *dvert, bool inverse, int def_nr)
{
  float weight = 1.0f;

  if ((dvert != NULL) && (def_nr != -1)) {
    MDeformWeight *dw = BKE_defvert_find_index(dvert, def_nr);
    weight = dw ? dw->weight : -1.0f;
    if ((weight >= 0.0f) && (inverse == 1)) {
      return -1.0f;
    }

    if ((weight < 0.0f) && (inverse == 0)) {
      return -1.0f;
    }

    /* if inverse, weight is always 1 */
    if ((weight < 0.0f) && (inverse == 1)) {
      return 1.0f;
    }
  }

  /* handle special empty groups */
  if ((dvert == NULL) && (def_nr != -1)) {
    if (inverse == 1) {
      return 1.0f;
    }

    return -1.0f;
  }

  return weight;
}

/**
 * Creates a bGPDcurve by doing a cubic curve fitting on the grease pencil stroke points.
 */
bGPDcurve *BKE_gpencil_stroke_editcurve_generate(bGPDstroke *gps,
                                                 const float error_threshold,
                                                 const float corner_angle)
{
  if (gps->totpoints < 3) {
    return gpencil_stroke_editcurve_generate_edgecases(gps);
  }
  const bool is_cyclic = (gps->flag & GP_STROKE_CYCLIC);
  const float diag_length = len_v3v3(gps->boundbox_min, gps->boundbox_max);
  tCurveSegment *tcs = gpencil_fit_curve_to_points_ex(
      gps, diag_length, error_threshold, corner_angle, 0, gps->totpoints - 1, is_cyclic);

  bGPDcurve *gpc = BKE_gpencil_stroke_editcurve_new(tcs->cubic_array_len);
  memcpy(gpc->curve_points, tcs->curve_points, sizeof(bGPDcurve_point) * tcs->cubic_array_len);
  gpencil_free_curve_segment(tcs);

  return gpc;
}

/**
 * Creates a bGPDcurve by doing a cubic curve fitting on the tagged segments (parts of the curve
 * with at least one bGPDspoint with GP_SPOINT_TAG).
 */
bGPDcurve *BKE_gpencil_stroke_editcurve_tagged_segments_update(bGPDstroke *gps,
                                                               const float error_threshold,
                                                               const float corner_angle)
{
  if (gps->totpoints < 3) {
    BKE_gpencil_free_stroke_editcurve(gps);
    return gpencil_stroke_editcurve_generate_edgecases(gps);
  }

  /* The reason why we cant just take all the bad segments and regenerate them is because that
   * will leave the control points at their original spot (if a control point moved, we should no
   * longer treat it as a control point). Therefor the only option is to find */

  bGPDcurve *gpc = gps->editcurve;
  const int num_segments = gpc->tot_curve_points - 1;
  float diag_length = len_v3v3(gps->boundbox_min, gps->boundbox_max);

  ListBase curve_segments = {NULL, NULL};

  /* TODO: If the stroke is cyclic we need to move the start of the curve to the last curve point
   * that is not tagged. */

  tCurveSegment *tcs;
  int j = 0, new_num_segments = 0;
  for (int i = 0; i < num_segments; i = j) {
    int island_length = 1;
    int start_idx = gpc->curve_points[i].point_index;
    bool is_tagged = gpencil_is_segment_tagged(gps, i);

    /* Find the end of this island (tagged or un-tagged). */
    int end_idx;
    bGPDspoint *end_pt;
    for (j = i + 1; j < gpc->tot_curve_points; j++, island_length++) {
      end_idx = gpc->curve_points[j].point_index;
      end_pt = &gps->points[end_idx];
      if (!(is_tagged && (end_pt->flag & GP_SPOINT_TAG)) &&
          (gpencil_is_segment_tagged(gps, j) != is_tagged)) {
        break;
      }
    }

    if (is_tagged) {
      /* Regenerate this segment. */
      tcs = gpencil_fit_curve_to_points_ex(
          gps, diag_length, error_threshold, corner_angle, start_idx, end_idx, false);
    }
    else {
      /* Save this segment. */
      tcs = MEM_callocN(sizeof(tCurveSegment), __func__);
      tcs->cubic_array_len = island_length;
      tcs->point_array_len = end_idx - start_idx + 1;
      tcs->curve_points = MEM_callocN(sizeof(bGPDcurve_point) * tcs->cubic_array_len, __func__);
      memcpy(tcs->curve_points,
             &gpc->curve_points[i],
             sizeof(bGPDcurve_point) * tcs->cubic_array_len);
    }

    new_num_segments += (tcs->cubic_array_len - 1);
    BLI_addtail(&curve_segments, tcs);
  }

  /* TODO: If the stroke is cyclic we need to check if the last segment needs to be regenerated or
   * saved. */
  bGPDcurve *editcurve = BKE_gpencil_stroke_editcurve_new(new_num_segments + 1);

  /* Copy first point. */
  tCurveSegment *frist_cs = (tCurveSegment *)&curve_segments.first;
  memcpy(&editcurve->curve_points[0], &frist_cs->curve_points[0], sizeof(bGPDcurve_point));

  /* Combine listbase curve segments to gpencil curve. */
  int offset = 0;
  LISTBASE_FOREACH_MUTABLE (tCurveSegment *, cs, &curve_segments) {
    /* Copy second handle of first point */
    bGPDcurve_point *first_dst = &editcurve->curve_points[offset];
    bGPDcurve_point *first_src = &cs->curve_points[0];
    copy_v3_v3(first_dst->bezt.vec[2], first_src->bezt.vec[2]);

    /* Copy the rest of the points. */
    memcpy(&editcurve->curve_points[offset + 1],
           &cs->curve_points[1],
           sizeof(bGPDcurve_point) * (cs->cubic_array_len - 1));

    offset += (cs->cubic_array_len - 1);
    gpencil_free_curve_segment(cs);
  }

  /* Free the old curve. */
  BKE_gpencil_free_stroke_editcurve(gps);
  return editcurve;
}

static void gpencil_stroke_editcurve_regenerate_single_ex(bGPDcurve_point *start,
                                                          bGPDcurve_point *end,
                                                          bGPDspoint *points,
                                                          const int totpoints,
                                                          const float boundbox_min[3],
                                                          const float diag_length,
                                                          const bool is_cyclic,
                                                          const float error_threshold,
                                                          float *r_error_sq)
{
#define POINT_DIM 3
  BezTriple *bezt_prev = &start->bezt;
  BezTriple *bezt_next = &end->bezt;

  const uint32_t start_pt_idx = start->point_index;
  const uint32_t end_pt_idx = end->point_index;

  uint32_t length = 0;
  if (start_pt_idx > end_pt_idx) {
    if (!is_cyclic) {
      return;
    }
    length = (totpoints - start_pt_idx) + end_pt_idx + 1;
  }
  else {
    length = (end_pt_idx - start_pt_idx) + 1;
  }

  uint32_t point_array_len = length * POINT_DIM;
  float *point_array = MEM_callocN(sizeof(float) * point_array_len, __func__);

  float tmp_vec[3];
  for (int i = 0; i < length; i++) {
    int idx = (i + start_pt_idx) % totpoints;
    bGPDspoint *pt = &points[idx];
    float *point = &point_array[i * POINT_DIM];
    /* normalize coordinate to 0..1 */
    sub_v3_v3v3(tmp_vec, &pt->x, boundbox_min);
    mul_v3_v3fl(point, tmp_vec, 1.0f / diag_length);
  }

  float tan_l[3], tan_r[3];

  sub_v3_v3v3(tan_l, bezt_prev->vec[1], bezt_prev->vec[2]);
  normalize_v3(tan_l);
  sub_v3_v3v3(tan_r, bezt_next->vec[0], bezt_next->vec[1]);
  normalize_v3(tan_r);

  float error_sq;
  uint error_index;
  int r = curve_fit_cubic_to_points_single_fl(point_array,
                                              length,
                                              NULL,
                                              POINT_DIM,
                                              error_threshold,
                                              tan_l,
                                              tan_r,
                                              bezt_prev->vec[2],
                                              bezt_next->vec[0],
                                              &error_sq,
                                              &error_index);
  if (r != 0) {
    MEM_freeN(point_array);
    return;
  }

  madd_v3_v3v3fl(bezt_prev->vec[2], boundbox_min, bezt_prev->vec[2], diag_length);
  madd_v3_v3v3fl(bezt_next->vec[0], boundbox_min, bezt_next->vec[0], diag_length);

  if (!ELEM(bezt_prev->h2, HD_FREE, HD_ALIGN)) {
    bezt_prev->h2 = (bezt_prev->h2 == HD_VECT) ? HD_FREE : HD_ALIGN;
  }
  if (!ELEM(bezt_next->h1, HD_FREE, HD_ALIGN)) {
    bezt_next->h1 = (bezt_next->h1 == HD_VECT) ? HD_FREE : HD_ALIGN;
  }

  MEM_freeN(point_array);

  if (r_error_sq != NULL) {
    *r_error_sq = error_sq;
  }
#undef POINT_DIM
}

/**
 * Regenerate the handles for the segment from `start_idx` to `end_idx` by refitting the curve to
 * the points. As an example, this is used by the dissolve operator to fit the curve to a segment
 * where the curve poitns have been deleted.
 * Note: `start_idx` can be greater than `end_idx`. If this is the case and the stroke is cyclic,
 * then the fitting will use the points from the end and beginning of the stroke.
 */
void BKE_gpencil_stroke_editcurve_regenerate_single(bGPDstroke *gps,
                                                    uint32_t start_idx,
                                                    uint32_t end_idx,
                                                    const float error_threshold)
{
  if (!GPENCIL_STROKE_TYPE_BEZIER(gps) || (start_idx == end_idx)) {
    return;
  }
  bGPDcurve *gpc = gps->editcurve;
  const float diag_length = len_v3v3(gps->boundbox_min, gps->boundbox_max);

  BLI_assert(start_idx < gpc->tot_curve_points && end_idx < gpc->tot_curve_points);

  bGPDcurve_point *start = &gpc->curve_points[start_idx];
  bGPDcurve_point *end = &gpc->curve_points[end_idx];

  gpencil_stroke_editcurve_regenerate_single_ex(start,
                                                end,
                                                gps->points,
                                                gps->totpoints,
                                                gps->boundbox_min,
                                                diag_length,
                                                gps->flag & GP_STROKE_CYCLIC,
                                                error_threshold,
                                                NULL);
}

/**
 * Updates the editcurve for a stroke.
 * \param gps: The stroke.
 * \param threshold: Fitting threshold. The gernerated curve should not deviate more than this
 * amount from the stroke.
 * \param corner_angle: If angles greater than this amount are detected during fitting, they will
 * be sharp (non-aligned handles).
 * \param flag: Flag for refitting options (see eGPStrokeGeoUpdateFlag).
 */
void BKE_gpencil_stroke_refit_curve(bGPDstroke *gps,
                                    const float threshold,
                                    const float corner_angle,
                                    const eGPStrokeGeoUpdateFlag flag)
{
  if (gps == NULL || gps->totpoints < 0) {
    return;
  }

  bGPDcurve *editcurve = NULL;
  short prev_flag = 0;

  /* If editcurve exists save the selection to the stroke points (only for syncing later). */
  if (gps->editcurve != NULL) {
    BKE_gpencil_stroke_editcurve_sync_selection(NULL, gps, gps->editcurve);
  }

  /* Do a partial update by only updating the curve segments that contain tagged points. */
  if ((flag & GP_GEO_UPDATE_CURVE_PARTIAL_REFIT) && gps->editcurve != NULL) {
    prev_flag = gps->editcurve->flag;
    /* Find the segments that need an update, then update them. */
    const int tot_num_segments = (gps->flag & GP_STROKE_CYCLIC) ?
                                     gps->editcurve->tot_curve_points :
                                     gps->editcurve->tot_curve_points - 1;
    const int num_sel_segments = gpencil_count_tagged_curve_segments(gps);
    if (num_sel_segments == 0) {
      /* No update needed. */
      return;
    }
    else if (num_sel_segments == tot_num_segments) {
      /* All segments need to be updated. Do a full update. */
      BKE_gpencil_free_stroke_editcurve(gps);
      editcurve = BKE_gpencil_stroke_editcurve_generate(gps, threshold, corner_angle);
    }
    else {
      /* Some segments are unchanged. Do a partial update. */
      editcurve = BKE_gpencil_stroke_editcurve_tagged_segments_update(
          gps, threshold, corner_angle);
      gpencil_clear_point_tag(gps);
    }
  }
  else {
    /* Do a full update. Delete the old curve and generate a new one. */
    if (gps->editcurve != NULL) {
      prev_flag = gps->editcurve->flag;
      BKE_gpencil_free_stroke_editcurve(gps);
    }

    editcurve = BKE_gpencil_stroke_editcurve_generate(gps, threshold, corner_angle);
  }

  if (editcurve == NULL) {
    /* This should not happen. Maybe add an assert here? */
    return;
  }

  /* Assign pointer. This makes the stroke a bezier stroke now. */
  gps->editcurve = editcurve;
  if (prev_flag) {
    gps->editcurve->flag = prev_flag;
  }

  if ((flag & GP_GEO_UPDATE_CURVE_PARTIAL_REFIT)) {
    BKE_gpencil_editcurve_recalculate_handles(gps);
  }
}

/**
 * Sync the selection from stroke to editcurve
 */
void BKE_gpencil_editcurve_stroke_sync_selection(bGPdata *gpd, bGPDstroke *gps, bGPDcurve *gpc)
{
  if (gps->flag & GP_STROKE_SELECT) {
    gpc->flag |= GP_CURVE_SELECT;
    BKE_gpencil_stroke_select_index_set(gpd, gps);

    for (int i = 0; i < gpc->tot_curve_points; i++) {
      bGPDcurve_point *gpc_pt = &gpc->curve_points[i];
      bGPDspoint *pt = &gps->points[gpc_pt->point_index];
      if (pt->flag & GP_SPOINT_SELECT) {
        gpc_pt->flag |= GP_CURVE_POINT_SELECT;
        BEZT_SEL_ALL(&gpc_pt->bezt);
      }
      else {
        gpc_pt->flag &= ~GP_CURVE_POINT_SELECT;
        BEZT_DESEL_ALL(&gpc_pt->bezt);
      }
    }
  }
  else {
    gpc->flag &= ~GP_CURVE_SELECT;
    gpencil_editstroke_deselect_all(gpc);
    BKE_gpencil_stroke_select_index_reset(gps);
  }
}

/**
 * Sync the selection from editcurve to stroke
 */
void BKE_gpencil_stroke_editcurve_sync_selection(bGPdata *gpd, bGPDstroke *gps, bGPDcurve *gpc)
{
  if (gpc->flag & GP_CURVE_SELECT) {
    gps->flag |= GP_STROKE_SELECT;
    if (gpd != NULL) {
      BKE_gpencil_stroke_select_index_set(gpd, gps);
    }

    for (int i = 0; i < gpc->tot_curve_points - 1; i++) {
      bGPDcurve_point *gpc_pt = &gpc->curve_points[i];
      bGPDspoint *pt = &gps->points[gpc_pt->point_index];
      bGPDcurve_point *gpc_pt_next = &gpc->curve_points[i + 1];

      if (gpc_pt->flag & GP_CURVE_POINT_SELECT) {
        pt->flag |= GP_SPOINT_SELECT;
        if (gpc_pt_next->flag & GP_CURVE_POINT_SELECT) {
          /* select all the points after */
          for (int j = gpc_pt->point_index + 1; j < gpc_pt_next->point_index; j++) {
            bGPDspoint *pt_next = &gps->points[j];
            pt_next->flag |= GP_SPOINT_SELECT;
          }
        }
      }
      else {
        pt->flag &= ~GP_SPOINT_SELECT;
        /* deselect all points after */
        for (int j = gpc_pt->point_index + 1; j < gpc_pt_next->point_index; j++) {
          bGPDspoint *pt_next = &gps->points[j];
          pt_next->flag &= ~GP_SPOINT_SELECT;
        }
      }
    }

    bGPDcurve_point *gpc_first = &gpc->curve_points[0];
    bGPDcurve_point *gpc_last = &gpc->curve_points[gpc->tot_curve_points - 1];
    bGPDspoint *last_pt = &gps->points[gpc_last->point_index];
    if (gpc_last->flag & GP_CURVE_POINT_SELECT) {
      last_pt->flag |= GP_SPOINT_SELECT;
    }
    else {
      last_pt->flag &= ~GP_SPOINT_SELECT;
    }

    if (gps->flag & GP_STROKE_CYCLIC) {
      if (gpc_first->flag & GP_CURVE_POINT_SELECT && gpc_last->flag & GP_CURVE_POINT_SELECT) {
        for (int i = gpc_last->point_index + 1; i < gps->totpoints; i++) {
          bGPDspoint *pt_next = &gps->points[i];
          pt_next->flag |= GP_SPOINT_SELECT;
        }
      }
      else {
        for (int i = gpc_last->point_index + 1; i < gps->totpoints; i++) {
          bGPDspoint *pt_next = &gps->points[i];
          pt_next->flag &= ~GP_SPOINT_SELECT;
        }
      }
    }
  }
  else {
    gps->flag &= ~GP_STROKE_SELECT;
    if (gpd != NULL) {
      BKE_gpencil_stroke_select_index_reset(gps);
    }
    for (int i = 0; i < gps->totpoints; i++) {
      bGPDspoint *pt = &gps->points[i];
      pt->flag &= ~GP_SPOINT_SELECT;
    }
  }
}

static float smooth_interpf(const float target, const float origin, const float t)
{
  float fac = 3.0f * t * t - 2.0f * t * t * t;  // smooth
  return interpf(target, origin, fac);
}

static void smooth_interp_v4_v4v4(float *r, const float *a, const float *b, const float t)
{
  float fac = 3.0f * t * t - 2.0f * t * t * t;  // smooth
  interp_v4_v4v4(r, a, b, fac);
}

static void gpencil_interp_stride_fl_from_to(
    float from, float to, float *point_offset, int it, int stride)
{
  /* smooth interpolation */
  float *r = point_offset;
  for (int i = 0; i <= it; i++) {
    *r = smooth_interpf(to, from, (float)i / (float)it);
    r = POINTER_OFFSET(r, stride);
  }
}

static void gpencil_interp_stride_v4_from_to(
    float from[4], float to[4], float *point_offset, int it, int stride)
{
  /* smooth interpolation */
  float *r = point_offset;
  for (int i = 0; i <= it; i++) {
    smooth_interp_v4_v4v4(r, from, to, (float)i / (float)it);
    r = POINTER_OFFSET(r, stride);
  }
}

static float gpencil_approximate_curve_segment_arclength(bGPDcurve_point *cpt_start,
                                                         bGPDcurve_point *cpt_end)
{
  BezTriple *bezt_start = &cpt_start->bezt;
  BezTriple *bezt_end = &cpt_end->bezt;

  float chord_len = len_v3v3(bezt_start->vec[1], bezt_end->vec[1]);
  float net_len = len_v3v3(bezt_start->vec[1], bezt_start->vec[2]);
  net_len += len_v3v3(bezt_start->vec[2], bezt_end->vec[0]);
  net_len += len_v3v3(bezt_end->vec[0], bezt_end->vec[1]);

  return (chord_len + net_len) / 2.0f;
}

/* Helper: Interpolate curve point attributes from curve point pair into point array. */
static void gpencil_calculate_stroke_points_curve_segment(bGPDcurve_point *cpt,
                                                          bGPDcurve_point *cpt_next,
                                                          float *points_offset,
                                                          int resolu,
                                                          int stride,
                                                          const eGPStrokeGeoUpdateFlag flag)
{
  const bool update_all_attributes = (flag == GP_GEO_UPDATE_DEFAULT);

  if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_POSITION)) {
    /* sample points on all 3 axis between two curve points */
    for (uint axis = 0; axis < 3; axis++) {
      BKE_curve_forward_diff_bezier(cpt->bezt.vec[1][axis],
                                    cpt->bezt.vec[2][axis],
                                    cpt_next->bezt.vec[0][axis],
                                    cpt_next->bezt.vec[1][axis],
                                    POINTER_OFFSET(points_offset, sizeof(float) * axis),
                                    (int)resolu,
                                    stride);
    }
  }

  /* interpolate other attributes */
  if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_PRESSURE)) {
    gpencil_interp_stride_fl_from_to(cpt->pressure,
                                     cpt_next->pressure,
                                     POINTER_OFFSET(points_offset, sizeof(float) * 3),
                                     resolu,
                                     stride);
  }
  if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_STRENGTH)) {
    gpencil_interp_stride_fl_from_to(cpt->strength,
                                     cpt_next->strength,
                                     POINTER_OFFSET(points_offset, sizeof(float) * 4),
                                     resolu,
                                     stride);
  }
  if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_COLOR)) {
    gpencil_interp_stride_v4_from_to(cpt->vert_color,
                                     cpt_next->vert_color,
                                     POINTER_OFFSET(points_offset, sizeof(float) * 5),
                                     resolu,
                                     stride);
  }
}

static float *gpencil_stroke_points_from_editcurve_adaptive_resolu(
    bGPDcurve_point *curve_point_array,
    int curve_point_array_len,
    int resolution,
    const uint num_segments,
    bool is_cyclic,
    const eGPStrokeGeoUpdateFlag flag,
    int *r_points_len,
    int **r_segment_lengths)
{
  /* One stride contains: x, y, z, pressure, strength, Vr, Vg, Vb, Vmix_factor */
  const uint stride = sizeof(float[9]);
  const uint cpt_last = curve_point_array_len - 1;
  int *segment_point_lengths = MEM_callocN(sizeof(int) * num_segments, __func__);

  uint points_len = 1;
  for (int i = 0; i < cpt_last; i++) {
    bGPDcurve_point *cpt = &curve_point_array[i];
    bGPDcurve_point *cpt_next = &curve_point_array[i + 1];
    float arclen = gpencil_approximate_curve_segment_arclength(cpt, cpt_next);
    int segment_resolu = (int)floorf(arclen * resolution);
    CLAMP_MIN(segment_resolu, 1);

    segment_point_lengths[i] = segment_resolu;
    points_len += segment_resolu;
  }

  if (is_cyclic) {
    bGPDcurve_point *cpt = &curve_point_array[cpt_last];
    bGPDcurve_point *cpt_next = &curve_point_array[0];
    float arclen = gpencil_approximate_curve_segment_arclength(cpt, cpt_next);
    int segment_resolu = (int)floorf(arclen * resolution);
    CLAMP_MIN(segment_resolu, 1);

    segment_point_lengths[cpt_last] = segment_resolu;
    points_len += segment_resolu;
  }

  float(*r_points)[9] = MEM_callocN((stride * points_len * (is_cyclic ? 2 : 1)), __func__);
  float *points_offset = &r_points[0][0];
  int point_index = 0;
  for (int i = 0; i < cpt_last; i++) {
    bGPDcurve_point *cpt_curr = &curve_point_array[i];
    bGPDcurve_point *cpt_next = &curve_point_array[i + 1];
    int segment_resolu = segment_point_lengths[i];
    gpencil_calculate_stroke_points_curve_segment(
        cpt_curr, cpt_next, points_offset, segment_resolu, stride, flag);
    /* update the index */
    cpt_curr->point_index = point_index;
    point_index += segment_resolu;
    points_offset = POINTER_OFFSET(points_offset, segment_resolu * stride);
  }

  bGPDcurve_point *cpt_curr = &curve_point_array[cpt_last];
  cpt_curr->point_index = point_index;
  if (is_cyclic) {
    bGPDcurve_point *cpt_next = &curve_point_array[0];
    int segment_resolu = segment_point_lengths[cpt_last];
    gpencil_calculate_stroke_points_curve_segment(
        cpt_curr, cpt_next, points_offset, segment_resolu, stride, flag);
  }

  *r_points_len = points_len;
  *r_segment_lengths = segment_point_lengths;
  return (float(*))r_points;
}

/**
 * Helper: calculate the points on a curve with a fixed resolution.
 */
static float *gpencil_stroke_points_from_editcurve_fixed_resolu(bGPDcurve_point *curve_point_array,
                                                                int curve_point_array_len,
                                                                int resolution,
                                                                bool is_cyclic,
                                                                const eGPStrokeGeoUpdateFlag flag,
                                                                int *r_points_len)
{
  /* One stride contains: x, y, z, pressure, strength, Vr, Vg, Vb, Vmix_factor */
  const uint stride = sizeof(float[9]);
  const uint array_last = curve_point_array_len - 1;
  const uint resolu_stride = resolution * stride;
  const uint points_len = BKE_curve_calc_coords_axis_len(
      curve_point_array_len, resolution, is_cyclic, false);

  float(*r_points)[9] = MEM_callocN((stride * points_len * (is_cyclic ? 2 : 1)), __func__);
  float *points_offset = &r_points[0][0];
  for (unsigned int i = 0; i < array_last; i++) {
    bGPDcurve_point *cpt_curr = &curve_point_array[i];
    bGPDcurve_point *cpt_next = &curve_point_array[i + 1];

    gpencil_calculate_stroke_points_curve_segment(
        cpt_curr, cpt_next, points_offset, resolution, stride, flag);
    /* update the index */
    cpt_curr->point_index = i * resolution;
    points_offset = POINTER_OFFSET(points_offset, resolu_stride);
  }

  bGPDcurve_point *cpt_curr = &curve_point_array[array_last];
  cpt_curr->point_index = array_last * resolution;
  if (is_cyclic) {
    bGPDcurve_point *cpt_next = &curve_point_array[0];
    gpencil_calculate_stroke_points_curve_segment(
        cpt_curr, cpt_next, points_offset, resolution, stride, flag);
  }

  *r_points_len = points_len;
  return (float(*))r_points;
}

/**
 * Recalculate stroke points with the editcurve of the stroke.
 */
void BKE_gpencil_stroke_update_geometry_from_editcurve(bGPDstroke *gps,
                                                       const uint resolution,
                                                       const bool adaptive,
                                                       const eGPStrokeGeoUpdateFlag flag)
{
  if (gps == NULL || gps->editcurve == NULL) {
    return;
  }
  const bool update_all_attributes = (flag == GP_GEO_UPDATE_DEFAULT);

  bGPDcurve *gpc = gps->editcurve;
  bGPDcurve_point *curve_point_array = gpc->curve_points;
  int curve_point_array_len = gpc->tot_curve_points;
  if (curve_point_array_len == 0) {
    return;
  }
  /* Handle case for single curve point. */
  if (curve_point_array_len == 1) {
    bGPDcurve_point *cpt = &curve_point_array[0];
    /* resize stroke point array */
    gps->totpoints = 1;
    gps->points = MEM_recallocN(gps->points, sizeof(bGPDspoint) * gps->totpoints);

    bGPDspoint *pt = &gps->points[0];
    if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_POSITION)) {
      copy_v3_v3(&pt->x, cpt->bezt.vec[1]);
    }
    if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_PRESSURE)) {
      pt->pressure = cpt->pressure;
    }
    if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_STRENGTH)) {
      pt->strength = cpt->strength;
    }
    if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_COLOR)) {
      copy_v4_v4(pt->vert_color, cpt->vert_color);
    }

    if (gpc->dvert != NULL && (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_WEIGHT))) {
      gps->dvert = MEM_recallocN(gps->dvert, sizeof(MDeformVert) * gps->totpoints);

      /* Copy the weights of the curve point. */
      MDeformVert *dv_first = &gpc->dvert[0];
      MDeformVert *dvert = &gps->dvert[0];
      BKE_defvert_copy(dvert, dv_first);
      BKE_defvert_sync(dvert, dv_first, true);
    }

    /* deselect */
    pt->flag &= ~GP_SPOINT_SELECT;
    gps->flag &= ~GP_STROKE_SELECT;
    BKE_gpencil_stroke_select_index_reset(gps);

    return;
  }

  bool is_cyclic = gps->flag & GP_STROKE_CYCLIC;

  int points_len = 0;
  float(*points)[9] = NULL;
  int *segment_length_cache = NULL;
  const uint num_segments = (is_cyclic) ? curve_point_array_len : curve_point_array_len - 1;
  if (adaptive) {
    points = (float(*)[9])gpencil_stroke_points_from_editcurve_adaptive_resolu(
        curve_point_array,
        curve_point_array_len,
        resolution,
        num_segments,
        is_cyclic,
        flag,
        &points_len,
        &segment_length_cache);
  }
  else {
    points = (float(*)[9])gpencil_stroke_points_from_editcurve_fixed_resolu(
        curve_point_array, curve_point_array_len, resolution, is_cyclic, flag, &points_len);
  }

  if (points == NULL || points_len == 0) {
    return;
  }

  /* We have to free all of the old weight data and replace it completely. */
  BKE_gpencil_free_stroke_weights(gps);

  /* resize stroke point array */
  gps->totpoints = points_len;
  gps->points = MEM_recallocN(gps->points, sizeof(bGPDspoint) * gps->totpoints);

  int idx = 0;
  /* write new data to stroke point array */
  for (int i = 0; i < points_len; i++) {
    bGPDcurve_point *gpc_pt = &curve_point_array[idx % curve_point_array_len];
    bGPDspoint *pt = &gps->points[i];
    if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_POSITION)) {
      copy_v3_v3(&pt->x, &points[i][0]);
    }
    if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_PRESSURE)) {
      pt->pressure = points[i][3];
    }
    if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_STRENGTH)) {
      pt->strength = points[i][4];
    }
    if (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_COLOR)) {
      copy_v4_v4(pt->vert_color, &points[i][5]);
    }

    /* deselect points */
    pt->flag &= ~GP_SPOINT_SELECT;

    if (gpc_pt->point_index == i) {
      pt->flag |= GP_SPOINT_IS_BEZT_CONTROL;
      idx++;
    }
  }
  gps->flag &= ~GP_STROKE_SELECT;
  BKE_gpencil_stroke_select_index_reset(gps);

  /* Interpolate weights. */
  if (gpc->dvert != NULL && (update_all_attributes || (flag & GP_GEO_UPDATE_POLYLINE_WEIGHT))) {
    gps->dvert = MEM_callocN(sizeof(MDeformVert) * gps->totpoints, __func__);

    idx = 0;
    for (int i = 0; i < gpc->tot_curve_points - 1; i++) {
      MDeformVert *dv_curr = &gpc->dvert[i];
      MDeformVert *dv_next = &gpc->dvert[i + 1];
      int segment_length = (adaptive) ? segment_length_cache[i] : resolution;
      int totweight = max_ii(dv_curr->totweight, dv_next->totweight);

      if (totweight == 0) {
        idx += segment_length;
        continue;
      }

      for (int j = 0; j < segment_length; j++) {
        MDeformVert *dvert = &gps->dvert[idx + j];
        BKE_defvert_copy(dvert, dv_curr);
        float t = (float)j / (float)segment_length;
        for (int d = 0; d < totweight; d++) {
          MDeformWeight *dw_a = BKE_defvert_ensure_index(dv_curr, d);
          MDeformWeight *dw_b = BKE_defvert_ensure_index(dv_next, d);
          MDeformWeight *dw_final = BKE_defvert_ensure_index(dvert, d);

          if (dw_a->weight == dw_b->weight) {
            dw_final->weight = dw_a->weight;
          }
          else {
            dw_final->weight = smooth_interpf(dw_b->weight, dw_a->weight, t);
          }
        }
      }

      idx += segment_length;
    }

    if (is_cyclic) {
      /* Interpolate weights between last and first curve point. */
      MDeformVert *dv_curr = &gpc->dvert[gpc->tot_curve_points - 1];
      MDeformVert *dv_next = &gpc->dvert[0];
      int segment_length = (adaptive) ? segment_length_cache[gpc->tot_curve_points - 1] :
                                        resolution;
      int totweight = max_ii(dv_curr->totweight, dv_next->totweight);

      if (totweight) {
        for (int j = 0; j < segment_length; j++) {
          MDeformVert *dvert = &gps->dvert[idx + j];
          BKE_defvert_copy(dvert, dv_curr);
          float t = (float)j / (float)segment_length;
          for (int d = 0; d < totweight; d++) {
            MDeformWeight *dw_a = BKE_defvert_ensure_index(dv_curr, d);
            MDeformWeight *dw_b = BKE_defvert_ensure_index(dv_next, d);
            MDeformWeight *dw_final = BKE_defvert_ensure_index(dvert, d);

            if (dw_a->weight == dw_b->weight) {
              dw_final->weight = dw_a->weight;
            }
            else {
              dw_final->weight = smooth_interpf(dw_b->weight, dw_a->weight, t);
            }
          }
        }
      }
    }
    else {
      /* Copy the weights of the last curve point. */
      MDeformVert *dv_last = &gpc->dvert[gpc->tot_curve_points - 1];
      MDeformVert *dvert = &gps->dvert[gps->totpoints - 1];
      BKE_defvert_copy(dvert, dv_last);
      BKE_defvert_sync(dvert, dv_last, true);
    }
  }

  /* free temp data */
  MEM_freeN(points);
  MEM_SAFE_FREE(segment_length_cache);
}

/**
 * Recalculate the handles of the edit curve of a grease pencil stroke
 */
bool BKE_gpencil_editcurve_recalculate_handles(bGPDstroke *gps)
{
  if (gps == NULL || gps->editcurve == NULL) {
    return false;
  }

  bool changed = false;
  bGPDcurve *gpc = gps->editcurve;
  if (gpc->tot_curve_points < 2) {
    return false;
  }

  if (gpc->tot_curve_points == 1) {
    BKE_nurb_handle_calc(
        &(gpc->curve_points[0].bezt), NULL, &(gpc->curve_points[0].bezt), false, 0);
  }

  for (int i = 1; i < gpc->tot_curve_points - 1; i++) {
    bGPDcurve_point *gpc_pt = &gpc->curve_points[i];
    bGPDcurve_point *gpc_pt_prev = &gpc->curve_points[i - 1];
    bGPDcurve_point *gpc_pt_next = &gpc->curve_points[i + 1];
    /* update handle if point or neighbour is selected */
    if (gpc_pt->flag & GP_CURVE_POINT_SELECT || gpc_pt_prev->flag & GP_CURVE_POINT_SELECT ||
        gpc_pt_next->flag & GP_CURVE_POINT_SELECT) {
      BezTriple *bezt = &gpc_pt->bezt;
      BezTriple *bezt_prev = &gpc_pt_prev->bezt;
      BezTriple *bezt_next = &gpc_pt_next->bezt;

      BKE_nurb_handle_calc(bezt, bezt_prev, bezt_next, false, 0);
      changed = true;
    }
  }

  bGPDcurve_point *gpc_first = &gpc->curve_points[0];
  bGPDcurve_point *gpc_last = &gpc->curve_points[gpc->tot_curve_points - 1];
  bGPDcurve_point *gpc_first_next = &gpc->curve_points[1];
  bGPDcurve_point *gpc_last_prev = &gpc->curve_points[gpc->tot_curve_points - 2];

  if (gpc_first->flag & GP_CURVE_POINT_SELECT || gpc_last->flag & GP_CURVE_POINT_SELECT) {
    BezTriple *bezt_first = &gpc_first->bezt;
    BezTriple *bezt_last = &gpc_last->bezt;
    BezTriple *bezt_first_next = &gpc_first_next->bezt;
    BezTriple *bezt_last_prev = &gpc_last_prev->bezt;

    if (gps->flag & GP_STROKE_CYCLIC) {
      BKE_nurb_handle_calc(bezt_first, bezt_last, bezt_first_next, false, 0);
      BKE_nurb_handle_calc(bezt_last, bezt_last_prev, bezt_first, false, 0);
    }
    else {
      BKE_nurb_handle_calc(bezt_first, NULL, bezt_first_next, false, 0);
      BKE_nurb_handle_calc(bezt_last, bezt_last_prev, NULL, false, 0);
    }

    changed = true;
  }

  return changed;
}

/* Helper: count how many new curve points must be generated. */
static int gpencil_editcurve_subdivide_count(bGPDcurve *gpc, bool is_cyclic)
{
  int count = 0;
  for (int i = 0; i < gpc->tot_curve_points - 1; i++) {
    bGPDcurve_point *cpt = &gpc->curve_points[i];
    bGPDcurve_point *cpt_next = &gpc->curve_points[i + 1];

    if (cpt->flag & GP_CURVE_POINT_SELECT && cpt_next->flag & GP_CURVE_POINT_SELECT) {
      count++;
    }
  }

  if (is_cyclic) {
    bGPDcurve_point *cpt = &gpc->curve_points[0];
    bGPDcurve_point *cpt_next = &gpc->curve_points[gpc->tot_curve_points - 1];

    if (cpt->flag & GP_CURVE_POINT_SELECT && cpt_next->flag & GP_CURVE_POINT_SELECT) {
      count++;
    }
  }

  return count;
}

/* Helper: Divide the curve segment at t, where t is a value between 0 and 1. */
static void gpencil_editcurve_subdivide_curve_segment_factor(bGPDcurve_point *cpt_start,
                                                             bGPDcurve_point *cpt_end,
                                                             bGPDcurve_point *cpt_new,
                                                             const float t)
{
  BLI_assert(t >= 0.0f && t <= 1.0f);

  BezTriple *bezt_start = &cpt_start->bezt;
  BezTriple *bezt_end = &cpt_end->bezt;
  BezTriple *bezt_new = &cpt_new->bezt;
  const float z = 1 - t, tt = t * t, zz = z * z, ttt = t * t * t, zzz = z * z * z;

  for (int axis = 0; axis < 3; axis++) {
    float p0, p1, p2, p3, m0, m1, q0, q1, b;
    p0 = bezt_start->vec[1][axis];
    p1 = bezt_start->vec[2][axis];
    p2 = bezt_end->vec[0][axis];
    p3 = bezt_end->vec[1][axis];

    m0 = (t * p1) + (z * p0);
    q0 = (tt * p2) + 2 * (t * z * p1) + (zz * p0);
    b = (ttt * p3) + 3 * (tt * z * p2) + 3 * (t * zz * p1) + (zzz * p0);
    q1 = (tt * p3) + 2 * (t * z * p2) + (zz * p1);
    m1 = (t * p3) + (z * p2);

    bezt_new->vec[0][axis] = q0;
    bezt_new->vec[1][axis] = b;
    bezt_new->vec[2][axis] = q1;

    bezt_start->vec[2][axis] = m0;
    bezt_end->vec[0][axis] = m1;
  }

  cpt_new->pressure = interpf(cpt_end->pressure, cpt_start->pressure, t);
  cpt_new->strength = interpf(cpt_end->strength, cpt_start->strength, t);
  interp_v4_v4v4(cpt_new->vert_color, cpt_start->vert_color, cpt_end->vert_color, t);
  /* TODO: interpolate weights */
}

static void gpencil_editcurve_subdivide_curve_segment(bGPDcurve_point *cpt_start,
                                                      bGPDcurve_point *cpt_end,
                                                      bGPDcurve_point *cpt_new)
{
  BezTriple *bezt_start = &cpt_start->bezt;
  BezTriple *bezt_end = &cpt_end->bezt;
  BezTriple *bezt_new = &cpt_new->bezt;
  for (int axis = 0; axis < 3; axis++) {
    float p0, p1, p2, p3, m0, m1, q0, q1, b;
    p0 = bezt_start->vec[1][axis];
    p1 = bezt_start->vec[2][axis];
    p2 = bezt_end->vec[0][axis];
    p3 = bezt_end->vec[1][axis];

    m0 = (p0 + p1) / 2;
    q0 = (p0 + 2 * p1 + p2) / 4;
    b = (p0 + 3 * p1 + 3 * p2 + p3) / 8;
    q1 = (p1 + 2 * p2 + p3) / 4;
    m1 = (p2 + p3) / 2;

    bezt_new->vec[0][axis] = q0;
    bezt_new->vec[2][axis] = q1;
    bezt_new->vec[1][axis] = b;

    bezt_start->vec[2][axis] = m0;
    bezt_end->vec[0][axis] = m1;
  }

  cpt_new->pressure = interpf(cpt_end->pressure, cpt_start->pressure, 0.5f);
  cpt_new->strength = interpf(cpt_end->strength, cpt_start->strength, 0.5f);
  interp_v4_v4v4(cpt_new->vert_color, cpt_start->vert_color, cpt_end->vert_color, 0.5f);
}

void BKE_gpencil_editcurve_subdivide(bGPDstroke *gps, const int cuts)
{
  bGPDcurve *gpc = gps->editcurve;
  if (gpc == NULL || gpc->tot_curve_points < 2) {
    return;
  }
  bool is_cyclic = gps->flag & GP_STROKE_CYCLIC;

  /* repeat for number of cuts */
  /* TODO: Ideally the subdivisions should match how it works in the rest of Blender. Here we get
   * 1->1, 2->3, 3->7 etc. because we keep supdividing already subdivided segments. But it would be
   * better to get the exact number of cuts per segment.*/
  for (int s = 0; s < cuts; s++) {
    int old_tot_curve_points = gpc->tot_curve_points;
    int new_num_curve_points = gpencil_editcurve_subdivide_count(gpc, is_cyclic);
    if (new_num_curve_points == 0) {
      break;
    }
    int new_tot_curve_points = old_tot_curve_points + new_num_curve_points;

    bGPDcurve_point *temp_curve_points = (bGPDcurve_point *)MEM_callocN(
        sizeof(bGPDcurve_point) * new_tot_curve_points, __func__);

    bool prev_subdivided = false;
    int j = 0;
    for (int i = 0; i < old_tot_curve_points - 1; i++, j++) {
      bGPDcurve_point *cpt = &gpc->curve_points[i];
      bGPDcurve_point *cpt_next = &gpc->curve_points[i + 1];

      if (cpt->flag & GP_CURVE_POINT_SELECT && cpt_next->flag & GP_CURVE_POINT_SELECT) {
        bGPDcurve_point *cpt_new = &temp_curve_points[j + 1];
        gpencil_editcurve_subdivide_curve_segment(cpt, cpt_next, cpt_new);

        memcpy(&temp_curve_points[j], cpt, sizeof(bGPDcurve_point));
        memcpy(&temp_curve_points[j + 2], cpt_next, sizeof(bGPDcurve_point));

        cpt_new->flag |= GP_CURVE_POINT_SELECT;
        cpt_new->bezt.h1 = HD_ALIGN;
        cpt_new->bezt.h2 = HD_ALIGN;
        BEZT_SEL_ALL(&cpt_new->bezt);

        prev_subdivided = true;
        j++;
      }
      else if (!prev_subdivided) {
        memcpy(&temp_curve_points[j], cpt, sizeof(bGPDcurve_point));
        prev_subdivided = false;
      }
      else {
        prev_subdivided = false;
      }
    }

    if (is_cyclic) {
      bGPDcurve_point *cpt = &gpc->curve_points[old_tot_curve_points - 1];
      bGPDcurve_point *cpt_next = &gpc->curve_points[0];

      if (cpt->flag & GP_CURVE_POINT_SELECT && cpt_next->flag & GP_CURVE_POINT_SELECT) {
        bGPDcurve_point *cpt_new = &temp_curve_points[j + 1];
        gpencil_editcurve_subdivide_curve_segment(cpt, cpt_next, cpt_new);

        memcpy(&temp_curve_points[j], cpt, sizeof(bGPDcurve_point));
        memcpy(&temp_curve_points[0], cpt_next, sizeof(bGPDcurve_point));

        cpt_new->flag |= GP_CURVE_POINT_SELECT;
        cpt_new->bezt.h1 = HD_ALIGN;
        cpt_new->bezt.h2 = HD_ALIGN;
        BEZT_SEL_ALL(&cpt_new->bezt);
      }
      else if (!prev_subdivided) {
        memcpy(&temp_curve_points[j], cpt, sizeof(bGPDcurve_point));
      }
    }
    else {
      bGPDcurve_point *cpt = &gpc->curve_points[old_tot_curve_points - 1];
      memcpy(&temp_curve_points[j], cpt, sizeof(bGPDcurve_point));
    }

    MEM_freeN(gpc->curve_points);
    gpc->curve_points = temp_curve_points;
    gpc->tot_curve_points = new_tot_curve_points;
  }
}

static void gpencil_refit_single_from_to(bGPDstroke *gps,
                                         bGPDcurve *gpc,
                                         bGPDcurve_point *new_points,
                                         int from_start,
                                         int from_end,
                                         int to_start,
                                         int to_end,
                                         float error_threshold)
{
  bGPDcurve_point *cpt_start = &gpc->curve_points[from_start];
  bGPDcurve_point *cpt_end = &gpc->curve_points[from_end];
  bGPDcurve_point *new_cpt_start = &new_points[to_start];
  bGPDcurve_point *new_cpt_end = &new_points[to_end];

  BKE_gpencil_stroke_editcurve_regenerate_single(gps, from_start, from_end, error_threshold);

  memcpy(new_cpt_start, cpt_start, sizeof(bGPDcurve_point));
  memcpy(new_cpt_end, cpt_end, sizeof(bGPDcurve_point));
}

/**
 * Dissolves the curve points tagged with `flag`.
 * \param gps: The stroke.
 * \param flag: flag (see eGPDcurve_point_Flag).
 * \param refit_segments: Do a refit of the segments where points have been dissolved.
 * \param error_threshold: Refit threshold when refitting.
 *
 * \returns The number of points that remain after dissolving. Can be 0 in which case the caller
 * should delete the stroke.
 */
int BKE_gpencil_editcurve_dissolve(bGPDstroke *gps,
                                   const uint flag,
                                   const bool refit_segments,
                                   const float error_threshold)
{
  bGPDcurve *gpc = gps->editcurve;
  if (gpc == NULL || gpc->tot_curve_points < 2) {
    return gpc->tot_curve_points;
  }
  const bool is_cyclic = (gps->flag & GP_STROKE_CYCLIC);

  int first = -1, last = 0;
  int num_points_remaining = gpc->tot_curve_points;
  for (int i = 0; i < gpc->tot_curve_points; i++) {
    bGPDcurve_point *cpt = &gpc->curve_points[i];
    if ((cpt->flag & flag)) {
      num_points_remaining--;
    }
    else {
      if (first < 0) {
        first = i;
      }
      last = i;
    }
  }

  /* All points will be deleted. */
  if (num_points_remaining < 1) {
    return 0;
  }
  /* All points remain. */
  else if (num_points_remaining == gpc->tot_curve_points) {
    return gpc->tot_curve_points;
  }

  bGPDcurve_point *new_points = MEM_callocN(sizeof(bGPDcurve_point) * num_points_remaining,
                                            __func__);

  int new_idx = 0, start = first, end = first;
  for (int i = first; i < gpc->tot_curve_points; i++) {
    bGPDcurve_point *cpt = &gpc->curve_points[i];
    bGPDcurve_point *new_cpt = &new_points[new_idx];
    if ((cpt->flag & flag) == 0) {
      memcpy(new_cpt, cpt, sizeof(bGPDcurve_point));
      /* Check that the indices are in bounds. */
      if (refit_segments && (start > 0) && IN_RANGE(end, start, gpc->tot_curve_points)) {
        /* Refit this segment. */
        gpencil_refit_single_from_to(
            gps, gpc, new_points, start - 1, end, new_idx - 1, new_idx, error_threshold);
        start = end = i;
      }
      new_idx++;
      start++;
    }
    end++;
  }

  if (is_cyclic && refit_segments) {
    /* Make sure that there is at least one segment that needs updating. */
    if (last != first && first >= 0 && last >= 0 &&
        !(first == 0 && last == (gpc->tot_curve_points - 1))) {

      /* Refit this segment. */
      gpencil_refit_single_from_to(
          gps, gpc, new_points, last, first, num_points_remaining - 1, 0, error_threshold);
    }
  }

  /* Recreate array. */
  if (gpc->curve_points != NULL) {
    MEM_freeN(gpc->curve_points);
  }

  gpc->curve_points = new_points;
  gpc->tot_curve_points = num_points_remaining;
  BKE_gpencil_editcurve_recalculate_handles(gps);

  return num_points_remaining;
}

/* Helper: Dissolve the curve point with the lowest re-fit error. Dissolve only if error is under
 * `threshold`. Returns true if a point was dissolved. */
static bool gpencil_editcurve_dissolve_smallest_error_curve_point(bGPDstroke *gps,
                                                                  bGPDcurve *gpc,
                                                                  const float diag_length,
                                                                  const bool is_cyclic,
                                                                  const float threshold)
{
  bool changed = false;
  /* Duplicate the curve point array. */
  bGPDcurve_point *curve_point_array = MEM_dupallocN(gpc->curve_points);

  float lowest_error = FLT_MAX;
  int lowest_error_idx = 0;
  /* Loop over control point pairs with one control point in between.
   * Find the control point that produces the lowest error when removed. */
  for (int i = 0; i < gpc->tot_curve_points - 2; i++) {
    bGPDcurve_point *cpt_prev = &curve_point_array[i];
    bGPDcurve_point *cpt_next = &curve_point_array[i + 2];

    float error_sq;
    /* Regenerate the handles between cpt_prev and cpt_next as if the point in the middle didn't
     * exist. Get the re-fit error. */
    gpencil_stroke_editcurve_regenerate_single_ex(cpt_prev,
                                                  cpt_next,
                                                  gps->points,
                                                  gps->totpoints,
                                                  gps->boundbox_min,
                                                  diag_length,
                                                  is_cyclic,
                                                  0.0f,
                                                  &error_sq);

    if (error_sq < lowest_error) {
      lowest_error = error_sq;
      lowest_error_idx = i + 1;
    }
  }

  /* Dissolve the control point with the lowest error found. */
  if (sqrtf(lowest_error) < threshold) {
    int new_tot_curve_points = gpc->tot_curve_points - 1;
    bGPDcurve_point *new_points = MEM_callocN(sizeof(bGPDcurve_point) * new_tot_curve_points,
                                              __func__);
    /* Copy all other points. Skip the point that will be dissolved. */
    memcpy(new_points, gpc->curve_points, lowest_error_idx * sizeof(bGPDcurve_point));
    memcpy(new_points + lowest_error_idx,
           gpc->curve_points + lowest_error_idx + 1,
           (new_tot_curve_points - lowest_error_idx) * sizeof(bGPDcurve_point));

    /* Get the start and end points of the segment. */
    bGPDcurve_point *cpt_start = &curve_point_array[lowest_error_idx - 1];
    bGPDcurve_point *cpt_end = &curve_point_array[lowest_error_idx + 1];
    bGPDcurve_point *new_cpt_start = &new_points[lowest_error_idx - 1];
    bGPDcurve_point *new_cpt_end = &new_points[lowest_error_idx];

    /* Write the new handles. */
    copy_v3_v3(new_cpt_start->bezt.vec[2], cpt_start->bezt.vec[2]);
    copy_v3_v3(new_cpt_end->bezt.vec[0], cpt_end->bezt.vec[0]);

    /* Overwrite curve points. */
    MEM_freeN(gpc->curve_points);
    gpc->curve_points = new_points;
    gpc->tot_curve_points = new_tot_curve_points;

    changed = true;
  }

  MEM_freeN(curve_point_array);

  return changed;
}

/**
 * Simplify Adaptive
 * Dissolves all the curve points with a refit-error that is less than threshold.
 */
void BKE_gpencil_editcurve_simplify_adaptive(bGPDstroke *gps, const float threshold)
{
  bGPDcurve *gpc = gps->editcurve;
  if (gpc == NULL || gpc->tot_curve_points < 3) {
    return;
  }
  const bool is_cyclic = gps->flag & GP_STROKE_CYCLIC;
  const float diag_length = len_v3v3(gps->boundbox_min, gps->boundbox_max);

  bool changed = true;
  /* Loop until we have removed all points that causes an error less than `threshold`. */
  while (gpc->tot_curve_points > 2 && changed) {
    changed = gpencil_editcurve_dissolve_smallest_error_curve_point(
        gps, gpc, diag_length, is_cyclic, threshold);
  }
}

/**
 * Simplify Fixed
 * Dissolves `count` curve points with the lowest refit-error.
 */
void BKE_gpencil_editcurve_simplify_fixed(bGPDstroke *gps, const int count)
{
  bGPDcurve *gpc = gps->editcurve;
  if (gpc == NULL || gpc->tot_curve_points < 3 || count < 1) {
    return;
  }
  const bool is_cyclic = gps->flag & GP_STROKE_CYCLIC;
  const float diag_length = len_v3v3(gps->boundbox_min, gps->boundbox_max);

  /* Loop until we have removed all points that causes an error less than `threshold`. */
  for (int i = count; i >= 0 && gpc->tot_curve_points > 2; i--) {
    gpencil_editcurve_dissolve_smallest_error_curve_point(
        gps, gpc, diag_length, is_cyclic, FLT_MAX);
  }
}

/**
 * Smooth curve extra
 * \param gps: The grease pencil stroke.
 * \param factor: Smoothing factor.
 * \param step_size: Number of adjacent points to consider when smoothing.
 * \param repeat: Number of times to repeat the smoothing process.
 * \param only_selected: Only smooth selected points.
 * \param affect_endpoints: Include end-points when smoothing (only used when stroke is
 * non-cyclic).
 * \param use_vertex_groups: Use vertex group to mask the smoothing effect.
 * \param invert_weights: Invert the effect when using vertex groups.
 * \param deform_group: The vertex group.
 * \param curve_mapping: If not NULL use the curve mapping along the stroke to weigh the influence
 * of the smoothing effect.
 * \param do_positions: Smooth positions.
 * \param do_pressure: Smooth pressure.
 * \param do_strength: Smooth strength.
 */
void BKE_gpencil_editcurve_smooth_ex(bGPDstroke *gps,
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
                                     const bool do_strength)
{
  bGPDcurve *gpc = gps->editcurve;
  if (gpc == NULL || gpc->tot_curve_points < 2 || factor == 0.0f || repeat < 1) {
    return;
  }

  if (!(do_positions || do_pressure || do_strength)) {
    return;
  }

  const bool is_cyclic = gps->flag & GP_STROKE_CYCLIC;
  const bool use_curve_mapping = (curve_mapping != NULL);

  /* Total number of cubic points. */
  const uint tot_points = gpc->tot_curve_points * 3;
  for (uint r = 0; r < repeat; r++) {
    uint start_idx = (affect_endpoints || is_cyclic) ? 0 : 2;
    uint end_idx = (affect_endpoints || is_cyclic) ? tot_points : tot_points - 2;

    for (uint i = start_idx; i < end_idx; i++) {
      /* bGPDcurve_point index */
      uint pt_idx = i / 3;
      /* BezTriple handle index */
      uint hd_idx = i % 3;

      bGPDcurve_point *gpc_pt = &gpc->curve_points[pt_idx];
      MDeformVert *dvert = &gpc->dvert[pt_idx];
      if ((gpc_pt->flag & GP_CURVE_POINT_SELECT) == 0 && only_selected) {
        continue;
      }

      float weight = 1.0f;
      if (use_vertex_groups) {
        weight = get_point_weight(dvert, invert_weights, deform_group);
        if (weight < 0.0f) {
          continue;
        }
      }

      if (curve_mapping != NULL) {
        float value = (float)i / (float)(tot_points - 1);
        weight *= BKE_curvemapping_evaluateF(curve_mapping, 0, value);
        if (weight < 0.0f) {
          continue;
        }
      }

      BezTriple *bezt = &gpc_pt->bezt;
      float sco[3] = {0.0f};
      float smoothed_pressure = 0.0f;
      float smoothed_strength = 0.0f;

      float average_fac = 1.0f / (float)(step_size * 2 + 1);
      float weighted_factor = factor;
      if (use_vertex_groups || use_curve_mapping) {
        weighted_factor *= weight;
      }

      if (do_positions) {
        /* Include the current point. */
        madd_v3_v3fl(sco, bezt->vec[hd_idx], average_fac);

        for (uint step = 1; step <= step_size; step++) {
          int prev_idx = i - step;
          int next_idx = i + step;

          if (is_cyclic) {
            prev_idx = mod_i(prev_idx, tot_points);
            next_idx = next_idx % tot_points;
          }
          else {
            CLAMP_MIN(prev_idx, 0);
            CLAMP_MAX(next_idx, tot_points - 1);
          }

          uint prev_pt_idx = prev_idx / 3;
          uint prev_hd_idx = prev_idx % 3;
          uint next_pt_idx = next_idx / 3;
          uint next_hd_idx = next_idx % 3;

          bGPDcurve_point *gpc_prev_pt = &gpc->curve_points[prev_pt_idx];
          bGPDcurve_point *gpc_next_pt = &gpc->curve_points[next_pt_idx];
          BezTriple *prev_bezt = &gpc_prev_pt->bezt;
          BezTriple *next_bezt = &gpc_next_pt->bezt;

          madd_v3_v3fl(sco, prev_bezt->vec[prev_hd_idx], average_fac);
          madd_v3_v3fl(sco, next_bezt->vec[next_hd_idx], average_fac);
        }

        interp_v3_v3v3(bezt->vec[hd_idx], bezt->vec[hd_idx], sco, weighted_factor);
      }

      if (do_pressure && hd_idx == 1) {
        /* Include the current point. */
        smoothed_pressure += gpc_pt->pressure * average_fac;

        for (uint step = 1; step <= step_size; step++) {
          int prev_idx = pt_idx - step;
          int next_idx = pt_idx + step;

          if (is_cyclic) {
            prev_idx = mod_i(prev_idx, gpc->tot_curve_points);
            next_idx = next_idx % gpc->tot_curve_points;
          }
          else {
            CLAMP_MIN(prev_idx, 0);
            CLAMP_MAX(next_idx, gpc->tot_curve_points - 1);
          }

          bGPDcurve_point *gpc_prev_pt = &gpc->curve_points[prev_idx];
          bGPDcurve_point *gpc_next_pt = &gpc->curve_points[next_idx];
          smoothed_pressure += gpc_prev_pt->pressure * average_fac;
          smoothed_pressure += gpc_next_pt->pressure * average_fac;
        }

        gpc_pt->pressure = interpf(smoothed_pressure, gpc_pt->pressure, weighted_factor);
      }

      if (do_strength && hd_idx == 1) {
        /* Include the current point. */
        smoothed_strength += gpc_pt->strength * average_fac;

        for (uint step = 1; step <= step_size; step++) {
          int prev_idx = pt_idx - step;
          int next_idx = pt_idx + step;

          if (is_cyclic) {
            prev_idx = mod_i(prev_idx, gpc->tot_curve_points);
            next_idx = next_idx % gpc->tot_curve_points;
          }
          else {
            CLAMP_MIN(prev_idx, 0);
            CLAMP_MAX(next_idx, gpc->tot_curve_points - 1);
          }

          bGPDcurve_point *gpc_prev_pt = &gpc->curve_points[prev_idx];
          bGPDcurve_point *gpc_next_pt = &gpc->curve_points[next_idx];
          smoothed_strength += gpc_prev_pt->strength * average_fac;
          smoothed_strength += gpc_next_pt->strength * average_fac;
        }

        gpc_pt->strength = interpf(smoothed_strength, gpc_pt->strength, weighted_factor);
      }
    }
  }
}

/**
 * Smooth curve
 * \param gps: The grease pencil stroke.
 * \param factor: Smoothing factor.
 * \param step_size: Number of adjacent points to consider when smoothing.
 * \param repeat: Number of times to repeat the smoothing process.
 * \param only_selected: Only smooth selected points.
 * \param affect_endpoints: Include end-points when smoothing (only used when stroke is
 * non-cyclic).
 * \param do_positions: Smooth positions.
 * \param do_pressure: Smooth pressure.
 * \param do_strength: Smooth strength.
 */
void BKE_gpencil_editcurve_smooth(bGPDstroke *gps,
                                  const float factor,
                                  const uint step_size,
                                  const uint repeat,
                                  const bool only_selected,
                                  const bool affect_endpoints,
                                  const bool do_positions,
                                  const bool do_pressure,
                                  const bool do_strength)
{
  BKE_gpencil_editcurve_smooth_ex(gps,
                                  factor,
                                  step_size,
                                  repeat,
                                  only_selected,
                                  affect_endpoints,
                                  false,
                                  false,
                                  0,
                                  NULL,
                                  do_positions,
                                  do_pressure,
                                  do_strength);
}

/**
 * Curve Merge by Distance
 * Merge the control points by distance. Merging will always occur on the first point.
 * The first and last point are never merged. Note: The caller is resposible for the geometry
 * update of the stroke.
 * \param gps: Grease Pencil stroke.
 * \param threshold: Distance between points.
 * \param use_unselected: Set to true to analyze all stroke and not only selected points.
 * \param refit_segments: Set to refit segments where points were dissolved.
 * \param error_threshold: Error threshold for refitting (only used when refit_segments = true).
 * \returns True if any point was merged and the geometry changed.
 */
bool BKE_gpencil_editcurve_merge_distance(bGPDstroke *gps,
                                          const float threshold,
                                          const bool use_unselected,
                                          const bool refit_segments,
                                          const float error_threshold)
{
  bGPDcurve *gpc = gps->editcurve;
  if (gpc == NULL || gpc->tot_curve_points < 2 || threshold == 0.0f) {
    return false;
  }
  const float th_square = threshold * threshold;

  bool tagged = false;
  int i = 0, step = 1;
  while ((i < gpc->tot_curve_points - 1) && (i + step < gpc->tot_curve_points)) {
    bGPDcurve_point *gpc_pt = &gpc->curve_points[i];
    bGPDcurve_point *gpc_pt_next = &gpc->curve_points[i + step];
    if ((!use_unselected && ((gpc_pt->flag & GP_CURVE_POINT_SELECT) == 0 ||
                             (gpc_pt_next->flag & GP_CURVE_POINT_SELECT) == 0)) ||
        (gpc_pt->flag & GP_CURVE_POINT_TAG)) {
      i++;
      step = 1;
      continue;
    }
    if (gpc_pt_next->flag & GP_CURVE_POINT_TAG) {
      step++;
      continue;
    }
    BezTriple *bezt = &gpc_pt->bezt;
    BezTriple *bezt_next = &gpc_pt_next->bezt;

    float len_square = len_squared_v3v3(bezt->vec[1], bezt_next->vec[1]);
    if (len_square <= th_square) {
      gpc_pt_next->flag |= GP_CURVE_POINT_TAG;
      tagged = true;
      step++;
    }
    else {
      i++;
      step = 1;
    }
  }

  gpc->curve_points[0].flag &= ~GP_CURVE_POINT_TAG;
  gpc->curve_points[gpc->tot_curve_points - 1].flag &= ~GP_CURVE_POINT_TAG;

  int old_num_points = gpc->tot_curve_points;
  if (tagged) {
    return BKE_gpencil_editcurve_dissolve(
               gps, GP_CURVE_POINT_TAG, refit_segments, error_threshold) != old_num_points;
  }
  return false;
}

/** \} */

