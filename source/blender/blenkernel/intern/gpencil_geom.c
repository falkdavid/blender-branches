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
#include "BLI_polyfill_2d.h"

#include "BLT_translation.h"

#include "DNA_gpencil_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_screen_types.h"

#include "BKE_collection.h"
#include "BKE_context.h"
#include "BKE_curve.h"
#include "BKE_deform.h"
#include "BKE_gpencil.h"
#include "BKE_gpencil_geom.h"
#include "BKE_main.h"
#include "BKE_material.h"
#include "BKE_object.h"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_query.h"

/* GP Object - Boundbox Support */
/**
 * Get min/max coordinate bounds for single stroke
 * \return Returns whether we found any selected points
 */
bool BKE_gpencil_stroke_minmax(const bGPDstroke *gps,
                               const bool use_select,
                               float r_min[3],
                               float r_max[3])
{
  const bGPDspoint *pt;
  int i;
  bool changed = false;

  if (ELEM(NULL, gps, r_min, r_max)) {
    return false;
  }

  for (i = 0, pt = gps->points; i < gps->totpoints; i++, pt++) {
    if ((use_select == false) || (pt->flag & GP_SPOINT_SELECT)) {
      minmax_v3v3_v3(r_min, r_max, &pt->x);
      changed = true;
    }
  }
  return changed;
}

/* get min/max bounds of all strokes in GP datablock */
bool BKE_gpencil_data_minmax(const bGPdata *gpd, float r_min[3], float r_max[3])
{
  bool changed = false;

  INIT_MINMAX(r_min, r_max);

  if (gpd == NULL) {
    return changed;
  }

  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    bGPDframe *gpf = gpl->actframe;

    if (gpf != NULL) {
      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
        changed = BKE_gpencil_stroke_minmax(gps, false, r_min, r_max);
      }
    }
  }

  return changed;
}

/* compute center of bounding box */
void BKE_gpencil_centroid_3d(bGPdata *gpd, float r_centroid[3])
{
  float min[3], max[3], tot[3];

  BKE_gpencil_data_minmax(gpd, min, max);

  add_v3_v3v3(tot, min, max);
  mul_v3_v3fl(r_centroid, tot, 0.5f);
}

/* Compute stroke bounding box. */
void BKE_gpencil_stroke_boundingbox_calc(bGPDstroke *gps)
{
  INIT_MINMAX(gps->boundbox_min, gps->boundbox_max);
  BKE_gpencil_stroke_minmax(gps, false, gps->boundbox_min, gps->boundbox_max);
}

/* create bounding box values */
static void boundbox_gpencil(Object *ob)
{
  BoundBox *bb;
  bGPdata *gpd;
  float min[3], max[3];

  if (ob->runtime.bb == NULL) {
    ob->runtime.bb = MEM_callocN(sizeof(BoundBox), "GPencil boundbox");
  }

  bb = ob->runtime.bb;
  gpd = ob->data;

  if (!BKE_gpencil_data_minmax(gpd, min, max)) {
    min[0] = min[1] = min[2] = -1.0f;
    max[0] = max[1] = max[2] = 1.0f;
  }

  BKE_boundbox_init_from_minmax(bb, min, max);

  bb->flag &= ~BOUNDBOX_DIRTY;
}

/* get bounding box */
BoundBox *BKE_gpencil_boundbox_get(Object *ob)
{
  if (ELEM(NULL, ob, ob->data)) {
    return NULL;
  }

  bGPdata *gpd = (bGPdata *)ob->data;
  if ((ob->runtime.bb) && ((gpd->flag & GP_DATA_CACHE_IS_DIRTY) == 0)) {
    return ob->runtime.bb;
  }

  boundbox_gpencil(ob);

  return ob->runtime.bb;
}

/* ************************************************** */

static int stroke_march_next_point(const bGPDstroke *gps,
                                   const int index_next_pt,
                                   const float *current,
                                   const float dist,
                                   float *result,
                                   float *pressure,
                                   float *strength,
                                   float *vert_color,
                                   float *ratio_result,
                                   int *index_from,
                                   int *index_to)
{
  float remaining_till_next = 0.0f;
  float remaining_march = dist;
  float step_start[3];
  float point[3];
  int next_point_index = index_next_pt;
  bGPDspoint *pt = NULL;

  if (!(next_point_index < gps->totpoints)) {
    return -1;
  }

  copy_v3_v3(step_start, current);
  pt = &gps->points[next_point_index];
  copy_v3_v3(point, &pt->x);
  remaining_till_next = len_v3v3(point, step_start);

  while (remaining_till_next < remaining_march) {
    remaining_march -= remaining_till_next;
    pt = &gps->points[next_point_index];
    copy_v3_v3(point, &pt->x);
    copy_v3_v3(step_start, point);
    next_point_index++;
    if (!(next_point_index < gps->totpoints)) {
      next_point_index = gps->totpoints - 1;
      break;
    }
    pt = &gps->points[next_point_index];
    copy_v3_v3(point, &pt->x);
    remaining_till_next = len_v3v3(point, step_start);
  }
  if (remaining_till_next < remaining_march) {
    pt = &gps->points[next_point_index];
    copy_v3_v3(result, &pt->x);
    *pressure = gps->points[next_point_index].pressure;
    *strength = gps->points[next_point_index].strength;
    memcpy(vert_color, gps->points[next_point_index].vert_color, sizeof(float) * 4);

    *index_from = next_point_index - 1;
    *index_to = next_point_index;
    *ratio_result = 1.0f;

    return 0;
  }
  else {
    float ratio = remaining_march / remaining_till_next;
    interp_v3_v3v3(result, step_start, point, ratio);
    *pressure = interpf(
        gps->points[next_point_index].pressure, gps->points[next_point_index - 1].pressure, ratio);
    *strength = interpf(
        gps->points[next_point_index].strength, gps->points[next_point_index - 1].strength, ratio);
    interp_v4_v4v4(vert_color,
                   gps->points[next_point_index - 1].vert_color,
                   gps->points[next_point_index].vert_color,
                   ratio);

    *index_from = next_point_index - 1;
    *index_to = next_point_index;
    *ratio_result = ratio;

    return next_point_index;
  }
}

static int stroke_march_next_point_no_interp(const bGPDstroke *gps,
                                             const int index_next_pt,
                                             const float *current,
                                             const float dist,
                                             float *result)
{
  float remaining_till_next = 0.0f;
  float remaining_march = dist;
  float step_start[3];
  float point[3];
  int next_point_index = index_next_pt;
  bGPDspoint *pt = NULL;

  if (!(next_point_index < gps->totpoints)) {
    return -1;
  }

  copy_v3_v3(step_start, current);
  pt = &gps->points[next_point_index];
  copy_v3_v3(point, &pt->x);
  remaining_till_next = len_v3v3(point, step_start);

  while (remaining_till_next < remaining_march) {
    remaining_march -= remaining_till_next;
    pt = &gps->points[next_point_index];
    copy_v3_v3(point, &pt->x);
    copy_v3_v3(step_start, point);
    next_point_index++;
    if (!(next_point_index < gps->totpoints)) {
      next_point_index = gps->totpoints - 1;
      break;
    }
    pt = &gps->points[next_point_index];
    copy_v3_v3(point, &pt->x);
    remaining_till_next = len_v3v3(point, step_start);
  }
  if (remaining_till_next < remaining_march) {
    pt = &gps->points[next_point_index];
    copy_v3_v3(result, &pt->x);
    return 0;
  }
  else {
    float ratio = remaining_march / remaining_till_next;
    interp_v3_v3v3(result, step_start, point, ratio);
    return next_point_index;
  }
}

static int stroke_march_count(const bGPDstroke *gps, const float dist)
{
  int point_count = 0;
  float point[3];
  int next_point_index = 1;
  bGPDspoint *pt = NULL;

  pt = &gps->points[0];
  copy_v3_v3(point, &pt->x);
  point_count++;

  while ((next_point_index = stroke_march_next_point_no_interp(
              gps, next_point_index, point, dist, point)) > -1) {
    point_count++;
    if (next_point_index == 0) {
      break; /* last point finished */
    }
  }
  return point_count;
}

static void stroke_defvert_create_nr_list(MDeformVert *dv_list,
                                          int count,
                                          ListBase *result,
                                          int *totweight)
{
  LinkData *ld;
  MDeformVert *dv;
  MDeformWeight *dw;
  int i, j;
  int tw = 0;
  for (i = 0; i < count; i++) {
    dv = &dv_list[i];

    /* find def_nr in list, if not exist, then create one */
    for (j = 0; j < dv->totweight; j++) {
      bool found = false;
      dw = &dv->dw[j];
      for (ld = result->first; ld; ld = ld->next) {
        if (ld->data == POINTER_FROM_INT(dw->def_nr)) {
          found = true;
          break;
        }
      }
      if (!found) {
        ld = MEM_callocN(sizeof(LinkData), "def_nr_item");
        ld->data = POINTER_FROM_INT(dw->def_nr);
        BLI_addtail(result, ld);
        tw++;
      }
    }
  }

  *totweight = tw;
}

static MDeformVert *stroke_defvert_new_count(int count, int totweight, ListBase *def_nr_list)
{
  int i, j;
  LinkData *ld;
  MDeformVert *dst = MEM_mallocN(count * sizeof(MDeformVert), "new_deformVert");

  for (i = 0; i < count; i++) {
    dst[i].dw = MEM_mallocN(sizeof(MDeformWeight) * totweight, "new_deformWeight");
    dst[i].totweight = totweight;
    j = 0;
    /* re-assign deform groups */
    for (ld = def_nr_list->first; ld; ld = ld->next) {
      dst[i].dw[j].def_nr = POINTER_AS_INT(ld->data);
      j++;
    }
  }

  return dst;
}

static void stroke_interpolate_deform_weights(
    bGPDstroke *gps, int index_from, int index_to, float ratio, MDeformVert *vert)
{
  const MDeformVert *vl = &gps->dvert[index_from];
  const MDeformVert *vr = &gps->dvert[index_to];
  int i;

  for (i = 0; i < vert->totweight; i++) {
    float wl = BKE_defvert_find_weight(vl, vert->dw[i].def_nr);
    float wr = BKE_defvert_find_weight(vr, vert->dw[i].def_nr);
    vert->dw[i].weight = interpf(wr, wl, ratio);
  }
}

/**
 * Resample a stroke
 * \param gps: Stroke to sample
 * \param dist: Distance of one segment
 */
bool BKE_gpencil_stroke_sample(bGPDstroke *gps, const float dist, const bool select)
{
  bGPDspoint *pt = gps->points;
  bGPDspoint *pt1 = NULL;
  bGPDspoint *pt2 = NULL;
  int i;
  LinkData *ld;
  ListBase def_nr_list = {0};

  if (gps->totpoints < 2 || dist < FLT_EPSILON) {
    return false;
  }
  /* TODO: Implement feature point preservation. */
  int count = stroke_march_count(gps, dist);

  bGPDspoint *new_pt = MEM_callocN(sizeof(bGPDspoint) * count, "gp_stroke_points_sampled");
  MDeformVert *new_dv = NULL;

  int result_totweight;

  if (gps->dvert != NULL) {
    stroke_defvert_create_nr_list(gps->dvert, gps->totpoints, &def_nr_list, &result_totweight);
    new_dv = stroke_defvert_new_count(count, result_totweight, &def_nr_list);
  }

  int next_point_index = 1;
  i = 0;
  float pressure, strength, ratio_result;
  float vert_color[4];
  int index_from, index_to;
  float last_coord[3];

  /*  1st point is always at the start */
  pt1 = &gps->points[0];
  copy_v3_v3(last_coord, &pt1->x);
  pt2 = &new_pt[i];
  copy_v3_v3(&pt2->x, last_coord);
  new_pt[i].pressure = pt[0].pressure;
  new_pt[i].strength = pt[0].strength;
  memcpy(new_pt[i].vert_color, pt[0].vert_color, sizeof(float) * 4);
  if (select) {
    new_pt[i].flag |= GP_SPOINT_SELECT;
  }
  i++;

  if (new_dv) {
    stroke_interpolate_deform_weights(gps, 0, 0, 0, &new_dv[0]);
  }

  /* The rest. */
  while ((next_point_index = stroke_march_next_point(gps,
                                                     next_point_index,
                                                     last_coord,
                                                     dist,
                                                     last_coord,
                                                     &pressure,
                                                     &strength,
                                                     vert_color,
                                                     &ratio_result,
                                                     &index_from,
                                                     &index_to)) > -1) {
    pt2 = &new_pt[i];
    copy_v3_v3(&pt2->x, last_coord);
    new_pt[i].pressure = pressure;
    new_pt[i].strength = strength;
    memcpy(new_pt[i].vert_color, vert_color, sizeof(float) * 4);
    if (select) {
      new_pt[i].flag |= GP_SPOINT_SELECT;
    }

    if (new_dv) {
      stroke_interpolate_deform_weights(gps, index_from, index_to, ratio_result, &new_dv[i]);
    }

    i++;
    if (next_point_index == 0) {
      break; /* last point finished */
    }
  }

  gps->points = new_pt;
  /* Free original vertex list. */
  MEM_freeN(pt);

  if (new_dv) {
    /* Free original weight data. */
    BKE_gpencil_free_stroke_weights(gps);
    MEM_freeN(gps->dvert);
    while ((ld = BLI_pophead(&def_nr_list))) {
      MEM_freeN(ld);
    }

    gps->dvert = new_dv;
  }

  gps->totpoints = i;

  /* Calc geometry data. */
  BKE_gpencil_stroke_geometry_update(gps);

  return true;
}

/**
 * Backbone stretch similar to Freestyle.
 * \param gps: Stroke to sample
 * \param dist: Distance of one segment
 * \param tip_length: Ignore tip jittering, set zero to use default value.
 */
bool BKE_gpencil_stroke_stretch(bGPDstroke *gps, const float dist, const float tip_length)
{
  bGPDspoint *pt = gps->points, *last_pt, *second_last, *next_pt;
  int i;
  float threshold = (tip_length == 0 ? 0.001f : tip_length);

  if (gps->totpoints < 2 || dist < FLT_EPSILON) {
    return false;
  }

  last_pt = &pt[gps->totpoints - 1];
  second_last = &pt[gps->totpoints - 2];
  next_pt = &pt[1];

  float len1 = 0.0f;
  float len2 = 0.0f;

  i = 1;
  while (len1 < threshold && gps->totpoints > i) {
    next_pt = &pt[i];
    len1 = len_v3v3(&next_pt->x, &pt->x);
    i++;
  }

  i = 2;
  while (len2 < threshold && gps->totpoints >= i) {
    second_last = &pt[gps->totpoints - i];
    len2 = len_v3v3(&last_pt->x, &second_last->x);
    i++;
  }

  float extend1 = (len1 + dist) / len1;
  float extend2 = (len2 + dist) / len2;

  float result1[3], result2[3];

  interp_v3_v3v3(result1, &next_pt->x, &pt->x, extend1);
  interp_v3_v3v3(result2, &second_last->x, &last_pt->x, extend2);

  copy_v3_v3(&pt->x, result1);
  copy_v3_v3(&last_pt->x, result2);

  return true;
}

/**
 * Trim stroke to needed segments
 * \param gps: Target stroke
 * \param index_from: the index of the first point to be used in the trimmed result
 * \param index_to: the index of the last point to be used in the trimmed result
 */
bool BKE_gpencil_stroke_trim_points(bGPDstroke *gps, const int index_from, const int index_to)
{
  bGPDspoint *pt = gps->points, *new_pt;
  MDeformVert *dv, *new_dv;

  const int new_count = index_to - index_from + 1;

  if (new_count >= gps->totpoints) {
    return false;
  }

  if (new_count == 1) {
    BKE_gpencil_free_stroke_weights(gps);
    MEM_freeN(gps->points);
    gps->points = NULL;
    gps->dvert = NULL;
    gps->totpoints = 0;
    return false;
  }

  new_pt = MEM_callocN(sizeof(bGPDspoint) * new_count, "gp_stroke_points_trimmed");

  for (int i = 0; i < new_count; i++) {
    memcpy(&new_pt[i], &pt[i + index_from], sizeof(bGPDspoint));
  }

  if (gps->dvert) {
    new_dv = MEM_callocN(sizeof(MDeformVert) * new_count, "gp_stroke_dverts_trimmed");
    for (int i = 0; i < new_count; i++) {
      dv = &gps->dvert[i + index_from];
      new_dv[i].flag = dv->flag;
      new_dv[i].totweight = dv->totweight;
      new_dv[i].dw = MEM_callocN(sizeof(MDeformWeight) * dv->totweight,
                                 "gp_stroke_dverts_dw_trimmed");
      for (int j = 0; j < dv->totweight; j++) {
        new_dv[i].dw[j].weight = dv->dw[j].weight;
        new_dv[i].dw[j].def_nr = dv->dw[j].def_nr;
      }
    }
    MEM_freeN(gps->dvert);
    gps->dvert = new_dv;
  }

  MEM_freeN(gps->points);
  gps->points = new_pt;
  gps->totpoints = new_count;

  return true;
}

bool BKE_gpencil_stroke_split(bGPDframe *gpf,
                              bGPDstroke *gps,
                              const int before_index,
                              bGPDstroke **remaining_gps)
{
  bGPDstroke *new_gps;
  bGPDspoint *pt = gps->points, *new_pt;
  MDeformVert *dv, *new_dv;

  if (before_index >= gps->totpoints || before_index == 0) {
    return false;
  }

  const int new_count = gps->totpoints - before_index;
  const int old_count = before_index;

  /* Handle remaining segments first. */

  new_gps = BKE_gpencil_stroke_add_existing_style(
      gpf, gps, gps->mat_nr, new_count, gps->thickness);

  new_pt = new_gps->points; /* Allocated from above. */

  for (int i = 0; i < new_count; i++) {
    memcpy(&new_pt[i], &pt[i + before_index], sizeof(bGPDspoint));
  }

  if (gps->dvert) {
    new_dv = MEM_callocN(sizeof(MDeformVert) * new_count,
                         "gp_stroke_dverts_remaining(MDeformVert)");
    for (int i = 0; i < new_count; i++) {
      dv = &gps->dvert[i + before_index];
      new_dv[i].flag = dv->flag;
      new_dv[i].totweight = dv->totweight;
      new_dv[i].dw = MEM_callocN(sizeof(MDeformWeight) * dv->totweight,
                                 "gp_stroke_dverts_dw_remaining(MDeformWeight)");
      for (int j = 0; j < dv->totweight; j++) {
        new_dv[i].dw[j].weight = dv->dw[j].weight;
        new_dv[i].dw[j].def_nr = dv->dw[j].def_nr;
      }
    }
    new_gps->dvert = new_dv;
  }

  (*remaining_gps) = new_gps;

  /* Trim the original stroke into a shorter one.
   * Keep the end point. */

  BKE_gpencil_stroke_trim_points(gps, 0, old_count);
  BKE_gpencil_stroke_geometry_update(gps);
  return true;
}

/**
 * Shrink the stroke by length.
 * \param gps: Stroke to shrink
 * \param dist: delta length
 */
bool BKE_gpencil_stroke_shrink(bGPDstroke *gps, const float dist)
{
  bGPDspoint *pt = gps->points, *second_last;
  int i;

  if (gps->totpoints < 2 || dist < FLT_EPSILON) {
    return false;
  }

  second_last = &pt[gps->totpoints - 2];

  float len1, this_len1, cut_len1;
  float len2, this_len2, cut_len2;
  int index_start, index_end;

  len1 = len2 = this_len1 = this_len2 = cut_len1 = cut_len2 = 0.0f;

  i = 1;
  while (len1 < dist && gps->totpoints > i - 1) {
    this_len1 = len_v3v3(&pt[i].x, &pt[i + 1].x);
    len1 += this_len1;
    cut_len1 = len1 - dist;
    i++;
  }
  index_start = i - 2;

  i = 2;
  while (len2 < dist && gps->totpoints >= i) {
    second_last = &pt[gps->totpoints - i];
    this_len2 = len_v3v3(&second_last[1].x, &second_last->x);
    len2 += this_len2;
    cut_len2 = len2 - dist;
    i++;
  }
  index_end = gps->totpoints - i + 2;

  if (len1 < dist || len2 < dist || index_end <= index_start) {
    index_start = index_end = 0; /* empty stroke */
  }

  if ((index_end == index_start + 1) && (cut_len1 + cut_len2 > 1.0f)) {
    index_start = index_end = 0; /* no length left to cut */
  }

  BKE_gpencil_stroke_trim_points(gps, index_start, index_end);

  if (gps->totpoints == 0) {
    return false;
  }

  pt = gps->points;

  float cut1 = cut_len1 / this_len1;
  float cut2 = cut_len2 / this_len2;

  float result1[3], result2[3];

  interp_v3_v3v3(result1, &pt[1].x, &pt[0].x, cut1);
  interp_v3_v3v3(result2, &pt[gps->totpoints - 2].x, &pt[gps->totpoints - 1].x, cut2);

  copy_v3_v3(&pt[0].x, result1);
  copy_v3_v3(&pt[gps->totpoints - 1].x, result2);

  return true;
}

/**
 * Apply smooth to stroke point
 * \param gps: Stroke to smooth
 * \param i: Point index
 * \param inf: Amount of smoothing to apply
 */
bool BKE_gpencil_stroke_smooth(bGPDstroke *gps, int i, float inf)
{
  bGPDspoint *pt = &gps->points[i];
  float sco[3] = {0.0f};

  /* Do nothing if not enough points to smooth out */
  if (gps->totpoints <= 2) {
    return false;
  }

  /* Only affect endpoints by a fraction of the normal strength,
   * to prevent the stroke from shrinking too much
   */
  if ((i == 0) || (i == gps->totpoints - 1)) {
    inf *= 0.1f;
  }

  /* Compute smoothed coordinate by taking the ones nearby */
  /* XXX: This is potentially slow,
   *      and suffers from accumulation error as earlier points are handled before later ones. */
  {
    /* XXX: this is hardcoded to look at 2 points on either side of the current one
     * (i.e. 5 items total). */
    const int steps = 2;
    const float average_fac = 1.0f / (float)(steps * 2 + 1);
    int step;

    /* add the point itself */
    madd_v3_v3fl(sco, &pt->x, average_fac);

    /* n-steps before/after current point */
    /* XXX: review how the endpoints are treated by this algorithm. */
    /* XXX: falloff measures should also introduce some weighting variations,
     *      so that further-out points get less weight. */
    for (step = 1; step <= steps; step++) {
      bGPDspoint *pt1, *pt2;
      int before = i - step;
      int after = i + step;

      CLAMP_MIN(before, 0);
      CLAMP_MAX(after, gps->totpoints - 1);

      pt1 = &gps->points[before];
      pt2 = &gps->points[after];

      /* add both these points to the average-sum (s += p[i]/n) */
      madd_v3_v3fl(sco, &pt1->x, average_fac);
      madd_v3_v3fl(sco, &pt2->x, average_fac);
    }
  }

  /* Based on influence factor, blend between original and optimal smoothed coordinate */
  interp_v3_v3v3(&pt->x, &pt->x, sco, inf);

  return true;
}

/**
 * Apply smooth for strength to stroke point */
bool BKE_gpencil_stroke_smooth_strength(bGPDstroke *gps, int point_index, float influence)
{
  bGPDspoint *ptb = &gps->points[point_index];

  /* Do nothing if not enough points */
  if ((gps->totpoints <= 2) || (point_index < 1)) {
    return false;
  }
  /* Only affect endpoints by a fraction of the normal influence */
  float inf = influence;
  if ((point_index == 0) || (point_index == gps->totpoints - 1)) {
    inf *= 0.01f;
  }
  /* Limit max influence to reduce pop effect. */
  CLAMP_MAX(inf, 0.98f);

  float total = 0.0f;
  float max_strength = 0.0f;
  const int steps = 4;
  const float average_fac = 1.0f / (float)(steps * 2 + 1);
  int step;

  /* add the point itself */
  total += ptb->strength * average_fac;
  max_strength = ptb->strength;

  /* n-steps before/after current point */
  for (step = 1; step <= steps; step++) {
    bGPDspoint *pt1, *pt2;
    int before = point_index - step;
    int after = point_index + step;

    CLAMP_MIN(before, 0);
    CLAMP_MAX(after, gps->totpoints - 1);

    pt1 = &gps->points[before];
    pt2 = &gps->points[after];

    /* add both these points to the average-sum (s += p[i]/n) */
    total += pt1->strength * average_fac;
    total += pt2->strength * average_fac;
    /* Save max value. */
    if (max_strength < pt1->strength) {
      max_strength = pt1->strength;
    }
    if (max_strength < pt2->strength) {
      max_strength = pt2->strength;
    }
  }

  /* Based on influence factor, blend between original and optimal smoothed value. */
  ptb->strength = interpf(ptb->strength, total, inf);
  /* Clamp to maximum stroke strength to avoid weird results. */
  CLAMP_MAX(ptb->strength, max_strength);

  return true;
}

/**
 * Apply smooth for thickness to stroke point (use pressure) */
bool BKE_gpencil_stroke_smooth_thickness(bGPDstroke *gps, int point_index, float influence)
{
  bGPDspoint *ptb = &gps->points[point_index];

  /* Do nothing if not enough points */
  if ((gps->totpoints <= 2) || (point_index < 1)) {
    return false;
  }
  /* Only affect endpoints by a fraction of the normal influence */
  float inf = influence;
  if ((point_index == 0) || (point_index == gps->totpoints - 1)) {
    inf *= 0.01f;
  }
  /* Limit max influence to reduce pop effect. */
  CLAMP_MAX(inf, 0.98f);

  float total = 0.0f;
  float max_pressure = 0.0f;
  const int steps = 4;
  const float average_fac = 1.0f / (float)(steps * 2 + 1);
  int step;

  /* add the point itself */
  total += ptb->pressure * average_fac;
  max_pressure = ptb->pressure;

  /* n-steps before/after current point */
  for (step = 1; step <= steps; step++) {
    bGPDspoint *pt1, *pt2;
    int before = point_index - step;
    int after = point_index + step;

    CLAMP_MIN(before, 0);
    CLAMP_MAX(after, gps->totpoints - 1);

    pt1 = &gps->points[before];
    pt2 = &gps->points[after];

    /* add both these points to the average-sum (s += p[i]/n) */
    total += pt1->pressure * average_fac;
    total += pt2->pressure * average_fac;
    /* Save max value. */
    if (max_pressure < pt1->pressure) {
      max_pressure = pt1->pressure;
    }
    if (max_pressure < pt2->pressure) {
      max_pressure = pt2->pressure;
    }
  }

  /* Based on influence factor, blend between original and optimal smoothed value. */
  ptb->pressure = interpf(ptb->pressure, total, inf);
  /* Clamp to maximum stroke thickness to avoid weird results. */
  CLAMP_MAX(ptb->pressure, max_pressure);
  return true;
}

/**
 * Apply smooth for UV rotation to stroke point (use pressure).
 */
bool BKE_gpencil_stroke_smooth_uv(bGPDstroke *gps, int point_index, float influence)
{
  bGPDspoint *ptb = &gps->points[point_index];

  /* Do nothing if not enough points */
  if (gps->totpoints <= 2) {
    return false;
  }

  /* Compute theoretical optimal value */
  bGPDspoint *pta, *ptc;
  int before = point_index - 1;
  int after = point_index + 1;

  CLAMP_MIN(before, 0);
  CLAMP_MAX(after, gps->totpoints - 1);

  pta = &gps->points[before];
  ptc = &gps->points[after];

  /* the optimal value is the corresponding to the interpolation of the pressure
   * at the distance of point b
   */
  float fac = line_point_factor_v3(&ptb->x, &pta->x, &ptc->x);
  /* sometimes the factor can be wrong due stroke geometry, so use middle point */
  if ((fac < 0.0f) || (fac > 1.0f)) {
    fac = 0.5f;
  }
  float optimal = interpf(ptc->uv_rot, pta->uv_rot, fac);

  /* Based on influence factor, blend between original and optimal */
  ptb->uv_rot = interpf(optimal, ptb->uv_rot, influence);
  CLAMP(ptb->uv_rot, -M_PI_2, M_PI_2);

  return true;
}
/* Get points of stroke always flat to view not affected by camera view or view position */
void BKE_gpencil_stroke_2d_flat(const bGPDspoint *points,
                                int totpoints,
                                float (*points2d)[2],
                                int *r_direction)
{
  BLI_assert(totpoints >= 2);

  const bGPDspoint *pt0 = &points[0];
  const bGPDspoint *pt1 = &points[1];
  const bGPDspoint *pt3 = &points[(int)(totpoints * 0.75)];

  float locx[3];
  float locy[3];
  float loc3[3];
  float normal[3];

  /* local X axis (p0 -> p1) */
  sub_v3_v3v3(locx, &pt1->x, &pt0->x);

  /* point vector at 3/4 */
  float v3[3];
  if (totpoints == 2) {
    mul_v3_v3fl(v3, &pt3->x, 0.001f);
  }
  else {
    copy_v3_v3(v3, &pt3->x);
  }

  sub_v3_v3v3(loc3, v3, &pt0->x);

  /* vector orthogonal to polygon plane */
  cross_v3_v3v3(normal, locx, loc3);

  /* local Y axis (cross to normal/x axis) */
  cross_v3_v3v3(locy, normal, locx);

  /* Normalize vectors */
  normalize_v3(locx);
  normalize_v3(locy);

  /* Get all points in local space */
  for (int i = 0; i < totpoints; i++) {
    const bGPDspoint *pt = &points[i];
    float loc[3];

    /* Get local space using first point as origin */
    sub_v3_v3v3(loc, &pt->x, &pt0->x);

    points2d[i][0] = dot_v3v3(loc, locx);
    points2d[i][1] = dot_v3v3(loc, locy);
  }

  /* Concave (-1), Convex (1), or Autodetect (0)? */
  *r_direction = (int)locy[2];
}

/* Get points of stroke always flat to view not affected by camera view or view position
 * using another stroke as reference
 */
void BKE_gpencil_stroke_2d_flat_ref(const bGPDspoint *ref_points,
                                    int ref_totpoints,
                                    const bGPDspoint *points,
                                    int totpoints,
                                    float (*points2d)[2],
                                    const float scale,
                                    int *r_direction)
{
  BLI_assert(totpoints >= 2);

  const bGPDspoint *pt0 = &ref_points[0];
  const bGPDspoint *pt1 = &ref_points[1];
  const bGPDspoint *pt3 = &ref_points[(int)(ref_totpoints * 0.75)];

  float locx[3];
  float locy[3];
  float loc3[3];
  float normal[3];

  /* local X axis (p0 -> p1) */
  sub_v3_v3v3(locx, &pt1->x, &pt0->x);

  /* point vector at 3/4 */
  float v3[3];
  if (totpoints == 2) {
    mul_v3_v3fl(v3, &pt3->x, 0.001f);
  }
  else {
    copy_v3_v3(v3, &pt3->x);
  }

  sub_v3_v3v3(loc3, v3, &pt0->x);

  /* vector orthogonal to polygon plane */
  cross_v3_v3v3(normal, locx, loc3);

  /* local Y axis (cross to normal/x axis) */
  cross_v3_v3v3(locy, normal, locx);

  /* Normalize vectors */
  normalize_v3(locx);
  normalize_v3(locy);

  /* Get all points in local space */
  for (int i = 0; i < totpoints; i++) {
    const bGPDspoint *pt = &points[i];
    float loc[3];
    float v1[3];
    float vn[3] = {0.0f, 0.0f, 0.0f};

    /* apply scale to extremes of the stroke to get better collision detection
     * the scale is divided to get more control in the UI parameter
     */
    /* first point */
    if (i == 0) {
      const bGPDspoint *pt_next = &points[i + 1];
      sub_v3_v3v3(vn, &pt->x, &pt_next->x);
      normalize_v3(vn);
      mul_v3_fl(vn, scale / 10.0f);
      add_v3_v3v3(v1, &pt->x, vn);
    }
    /* last point */
    else if (i == totpoints - 1) {
      const bGPDspoint *pt_prev = &points[i - 1];
      sub_v3_v3v3(vn, &pt->x, &pt_prev->x);
      normalize_v3(vn);
      mul_v3_fl(vn, scale / 10.0f);
      add_v3_v3v3(v1, &pt->x, vn);
    }
    else {
      copy_v3_v3(v1, &pt->x);
    }

    /* Get local space using first point as origin (ref stroke) */
    sub_v3_v3v3(loc, v1, &pt0->x);

    points2d[i][0] = dot_v3v3(loc, locx);
    points2d[i][1] = dot_v3v3(loc, locy);
  }

  /* Concave (-1), Convex (1), or Autodetect (0)? */
  *r_direction = (int)locy[2];
}

/* calc texture coordinates using flat projected points */
static void gpencil_calc_stroke_fill_uv(const float (*points2d)[2],
                                        bGPDstroke *gps,
                                        const float minv[2],
                                        float maxv[2],
                                        float (*r_uv)[2])
{
  const float s = sin(gps->uv_rotation);
  const float c = cos(gps->uv_rotation);

  /* Calc center for rotation. */
  float center[2] = {0.5f, 0.5f};
  float d[2];
  d[0] = maxv[0] - minv[0];
  d[1] = maxv[1] - minv[1];
  for (int i = 0; i < gps->totpoints; i++) {
    r_uv[i][0] = (points2d[i][0] - minv[0]) / d[0];
    r_uv[i][1] = (points2d[i][1] - minv[1]) / d[1];

    /* Apply translation. */
    add_v2_v2(r_uv[i], gps->uv_translation);

    /* Apply Rotation. */
    r_uv[i][0] -= center[0];
    r_uv[i][1] -= center[1];

    float x = r_uv[i][0] * c - r_uv[i][1] * s;
    float y = r_uv[i][0] * s + r_uv[i][1] * c;

    r_uv[i][0] = x + center[0];
    r_uv[i][1] = y + center[1];

    /* Apply scale. */
    if (gps->uv_scale != 0.0f) {
      mul_v2_fl(r_uv[i], 1.0f / gps->uv_scale);
    }
  }
}

/* Triangulate stroke for high quality fill (this is done only if cache is null or stroke was
 * modified) */
void BKE_gpencil_stroke_fill_triangulate(bGPDstroke *gps)
{
  BLI_assert(gps->totpoints >= 3);

  /* allocate memory for temporary areas */
  gps->tot_triangles = gps->totpoints - 2;
  uint(*tmp_triangles)[3] = MEM_mallocN(sizeof(*tmp_triangles) * gps->tot_triangles,
                                        "GP Stroke temp triangulation");
  float(*points2d)[2] = MEM_mallocN(sizeof(*points2d) * gps->totpoints,
                                    "GP Stroke temp 2d points");
  float(*uv)[2] = MEM_mallocN(sizeof(*uv) * gps->totpoints, "GP Stroke temp 2d uv data");

  int direction = 0;

  /* convert to 2d and triangulate */
  BKE_gpencil_stroke_2d_flat(gps->points, gps->totpoints, points2d, &direction);
  BLI_polyfill_calc(points2d, (uint)gps->totpoints, direction, tmp_triangles);

  /* calc texture coordinates automatically */
  float minv[2];
  float maxv[2];
  /* first needs bounding box data */
  ARRAY_SET_ITEMS(minv, -1.0f, -1.0f);
  ARRAY_SET_ITEMS(maxv, 1.0f, 1.0f);

  /* calc uv data */
  gpencil_calc_stroke_fill_uv(points2d, gps, minv, maxv, uv);

  /* Save triangulation data. */
  if (gps->tot_triangles > 0) {
    MEM_SAFE_FREE(gps->triangles);
    gps->triangles = MEM_callocN(sizeof(*gps->triangles) * gps->tot_triangles,
                                 "GP Stroke triangulation");

    for (int i = 0; i < gps->tot_triangles; i++) {
      memcpy(gps->triangles[i].verts, tmp_triangles[i], sizeof(uint[3]));
    }

    /* Copy UVs to bGPDspoint. */
    for (int i = 0; i < gps->totpoints; i++) {
      copy_v2_v2(gps->points[i].uv_fill, uv[i]);
    }
  }
  else {
    /* No triangles needed - Free anything allocated previously */
    if (gps->triangles) {
      MEM_freeN(gps->triangles);
    }

    gps->triangles = NULL;
  }

  /* clear memory */
  MEM_SAFE_FREE(tmp_triangles);
  MEM_SAFE_FREE(points2d);
  MEM_SAFE_FREE(uv);
}

/* texture coordinate utilities */
void BKE_gpencil_stroke_uv_update(bGPDstroke *gps)
{
  if (gps == NULL || gps->totpoints == 0) {
    return;
  }

  bGPDspoint *pt = gps->points;
  float totlen = 0.0f;
  pt[0].uv_fac = totlen;
  for (int i = 1; i < gps->totpoints; i++) {
    totlen += len_v3v3(&pt[i - 1].x, &pt[i].x);
    pt[i].uv_fac = totlen;
  }
}

/* Recalc the internal geometry caches for fill and uvs. */
void BKE_gpencil_stroke_geometry_update(bGPDstroke *gps)
{
  if (gps == NULL) {
    return;
  }

  if (gps->totpoints > 2) {
    BKE_gpencil_stroke_fill_triangulate(gps);
  }
  else {
    gps->tot_triangles = 0;
    MEM_SAFE_FREE(gps->triangles);
  }

  /* calc uv data along the stroke */
  BKE_gpencil_stroke_uv_update(gps);

  /* Calc stroke bounding box. */
  BKE_gpencil_stroke_boundingbox_calc(gps);
}

float BKE_gpencil_stroke_length(const bGPDstroke *gps, bool use_3d)
{
  if (!gps->points || gps->totpoints < 2) {
    return 0.0f;
  }
  float *last_pt = &gps->points[0].x;
  int i;
  bGPDspoint *pt;
  float total_length = 0.0f;
  for (i = 1; i < gps->totpoints; i++) {
    pt = &gps->points[i];
    if (use_3d) {
      total_length += len_v3v3(&pt->x, last_pt);
    }
    else {
      total_length += len_v2v2(&pt->x, last_pt);
    }
    last_pt = &pt->x;
  }
  return total_length;
}

/**
 * Trim stroke to the first intersection or loop
 * \param gps: Stroke data
 */
bool BKE_gpencil_stroke_trim(bGPDstroke *gps)
{
  if (gps->totpoints < 4) {
    return false;
  }
  bool intersect = false;
  int start, end;
  float point[3];
  /* loop segments from start until we have an intersection */
  for (int i = 0; i < gps->totpoints - 2; i++) {
    start = i;
    bGPDspoint *a = &gps->points[start];
    bGPDspoint *b = &gps->points[start + 1];
    for (int j = start + 2; j < gps->totpoints - 1; j++) {
      end = j + 1;
      bGPDspoint *c = &gps->points[j];
      bGPDspoint *d = &gps->points[end];
      float pointb[3];
      /* get intersection */
      if (isect_line_line_v3(&a->x, &b->x, &c->x, &d->x, point, pointb)) {
        if (len_v3(point) > 0.0f) {
          float closest[3];
          /* check intersection is on both lines */
          float lambda = closest_to_line_v3(closest, point, &a->x, &b->x);
          if ((lambda <= 0.0f) || (lambda >= 1.0f)) {
            continue;
          }
          lambda = closest_to_line_v3(closest, point, &c->x, &d->x);
          if ((lambda <= 0.0f) || (lambda >= 1.0f)) {
            continue;
          }
          else {
            intersect = true;
            break;
          }
        }
      }
    }
    if (intersect) {
      break;
    }
  }

  /* trim unwanted points */
  if (intersect) {

    /* save points */
    bGPDspoint *old_points = MEM_dupallocN(gps->points);
    MDeformVert *old_dvert = NULL;
    MDeformVert *dvert_src = NULL;

    if (gps->dvert != NULL) {
      old_dvert = MEM_dupallocN(gps->dvert);
    }

    /* resize gps */
    int newtot = end - start + 1;

    gps->points = MEM_recallocN(gps->points, sizeof(*gps->points) * newtot);
    if (gps->dvert != NULL) {
      gps->dvert = MEM_recallocN(gps->dvert, sizeof(*gps->dvert) * newtot);
    }

    for (int i = 0; i < newtot; i++) {
      int idx = start + i;
      bGPDspoint *pt_src = &old_points[idx];
      bGPDspoint *pt_new = &gps->points[i];
      memcpy(pt_new, pt_src, sizeof(bGPDspoint));
      if (gps->dvert != NULL) {
        dvert_src = &old_dvert[idx];
        MDeformVert *dvert = &gps->dvert[i];
        memcpy(dvert, dvert_src, sizeof(MDeformVert));
        if (dvert_src->dw) {
          memcpy(dvert->dw, dvert_src->dw, sizeof(MDeformWeight));
        }
      }
      if (idx == start || idx == end) {
        copy_v3_v3(&pt_new->x, point);
      }
    }

    gps->totpoints = newtot;

    MEM_SAFE_FREE(old_points);
    MEM_SAFE_FREE(old_dvert);
  }

  BKE_gpencil_stroke_geometry_update(gps);

  return intersect;
}

/**
 * Close stroke
 * \param gps: Stroke to close
 */
bool BKE_gpencil_stroke_close(bGPDstroke *gps)
{
  bGPDspoint *pt1 = NULL;
  bGPDspoint *pt2 = NULL;

  /* Only can close a stroke with 3 points or more. */
  if (gps->totpoints < 3) {
    return false;
  }

  /* Calc average distance between points to get same level of sampling. */
  float dist_tot = 0.0f;
  for (int i = 0; i < gps->totpoints - 1; i++) {
    pt1 = &gps->points[i];
    pt2 = &gps->points[i + 1];
    dist_tot += len_v3v3(&pt1->x, &pt2->x);
  }
  /* Calc the average distance. */
  float dist_avg = dist_tot / (gps->totpoints - 1);

  /* Calc distance between last and first point. */
  pt1 = &gps->points[gps->totpoints - 1];
  pt2 = &gps->points[0];
  float dist_close = len_v3v3(&pt1->x, &pt2->x);

  /* if the distance to close is very small, don't need add points and just enable cyclic. */
  if (dist_close <= dist_avg) {
    gps->flag |= GP_STROKE_CYCLIC;
    return true;
  }

  /* Calc number of points required using the average distance. */
  int tot_newpoints = MAX2(dist_close / dist_avg, 1);

  /* Resize stroke array. */
  int old_tot = gps->totpoints;
  gps->totpoints += tot_newpoints;
  gps->points = MEM_recallocN(gps->points, sizeof(*gps->points) * gps->totpoints);
  if (gps->dvert != NULL) {
    gps->dvert = MEM_recallocN(gps->dvert, sizeof(*gps->dvert) * gps->totpoints);
  }

  /* Generate new points */
  pt1 = &gps->points[old_tot - 1];
  pt2 = &gps->points[0];
  bGPDspoint *pt = &gps->points[old_tot];
  for (int i = 1; i < tot_newpoints + 1; i++, pt++) {
    float step = (tot_newpoints > 1) ? ((float)i / (float)tot_newpoints) : 0.99f;
    /* Clamp last point to be near, but not on top of first point. */
    if ((tot_newpoints > 1) && (i == tot_newpoints)) {
      step *= 0.99f;
    }

    /* Average point. */
    interp_v3_v3v3(&pt->x, &pt1->x, &pt2->x, step);
    pt->pressure = interpf(pt2->pressure, pt1->pressure, step);
    pt->strength = interpf(pt2->strength, pt1->strength, step);
    pt->flag = 0;
    interp_v4_v4v4(pt->vert_color, pt1->vert_color, pt2->vert_color, step);

    /* Set weights. */
    if (gps->dvert != NULL) {
      MDeformVert *dvert1 = &gps->dvert[old_tot - 1];
      MDeformWeight *dw1 = BKE_defvert_ensure_index(dvert1, 0);
      float weight_1 = dw1 ? dw1->weight : 0.0f;

      MDeformVert *dvert2 = &gps->dvert[0];
      MDeformWeight *dw2 = BKE_defvert_ensure_index(dvert2, 0);
      float weight_2 = dw2 ? dw2->weight : 0.0f;

      MDeformVert *dvert_final = &gps->dvert[old_tot + i - 1];
      dvert_final->totweight = 0;
      MDeformWeight *dw = BKE_defvert_ensure_index(dvert_final, 0);
      if (dvert_final->dw) {
        dw->weight = interpf(weight_2, weight_1, step);
      }
    }
  }

  /* Enable cyclic flag. */
  gps->flag |= GP_STROKE_CYCLIC;

  return true;
}
/* Dissolve points in stroke */
void BKE_gpencil_dissolve_points(bGPDframe *gpf, bGPDstroke *gps, const short tag)
{
  bGPDspoint *pt;
  MDeformVert *dvert = NULL;
  int i;

  int tot = gps->totpoints; /* number of points in new buffer */
  /* first pass: count points to remove */
  /* Count how many points are selected (i.e. how many to remove) */
  for (i = 0, pt = gps->points; i < gps->totpoints; i++, pt++) {
    if (pt->flag & tag) {
      /* selected point - one of the points to remove */
      tot--;
    }
  }

  /* if no points are left, we simply delete the entire stroke */
  if (tot <= 0) {
    /* remove the entire stroke */
    if (gps->points) {
      MEM_freeN(gps->points);
    }
    if (gps->dvert) {
      BKE_gpencil_free_stroke_weights(gps);
      MEM_freeN(gps->dvert);
    }
    if (gps->triangles) {
      MEM_freeN(gps->triangles);
    }
    BLI_freelinkN(&gpf->strokes, gps);
  }
  else {
    /* just copy all points to keep into a smaller buffer */
    bGPDspoint *new_points = MEM_callocN(sizeof(bGPDspoint) * tot, "new gp stroke points copy");
    bGPDspoint *npt = new_points;

    MDeformVert *new_dvert = NULL;
    MDeformVert *ndvert = NULL;

    if (gps->dvert != NULL) {
      new_dvert = MEM_callocN(sizeof(MDeformVert) * tot, "new gp stroke weights copy");
      ndvert = new_dvert;
    }

    (gps->dvert != NULL) ? dvert = gps->dvert : NULL;
    for (i = 0, pt = gps->points; i < gps->totpoints; i++, pt++) {
      if ((pt->flag & tag) == 0) {
        *npt = *pt;
        npt++;

        if (gps->dvert != NULL) {
          *ndvert = *dvert;
          ndvert->dw = MEM_dupallocN(dvert->dw);
          ndvert++;
        }
      }
      if (gps->dvert != NULL) {
        dvert++;
      }
    }

    /* free the old buffer */
    if (gps->points) {
      MEM_freeN(gps->points);
    }
    if (gps->dvert) {
      BKE_gpencil_free_stroke_weights(gps);
      MEM_freeN(gps->dvert);
    }

    /* save the new buffer */
    gps->points = new_points;
    gps->dvert = new_dvert;
    gps->totpoints = tot;

    /* triangles cache needs to be recalculated */
    BKE_gpencil_stroke_geometry_update(gps);
  }
}

/* Merge by distance ------------------------------------- */
/* Reduce a series of points when the distance is below a threshold.
 * Special case for first and last points (both are keeped) for other points,
 * the merge point always is at first point.
 * \param gpf: Grease Pencil frame
 * \param gps: Grease Pencil stroke
 * \param threshold: Distance between points
 * \param use_unselected: Set to true to analyze all stroke and not only selected points
 */
void BKE_gpencil_stroke_merge_distance(bGPDframe *gpf,
                                       bGPDstroke *gps,
                                       const float threshold,
                                       const bool use_unselected)
{
  bGPDspoint *pt = NULL;
  bGPDspoint *pt_next = NULL;
  float tagged = false;
  /* Use square distance to speed up loop */
  const float th_square = threshold * threshold;
  /* Need to have something to merge. */
  if (gps->totpoints < 2) {
    return;
  }
  int i = 0;
  int step = 1;
  while ((i < gps->totpoints - 1) && (i + step < gps->totpoints)) {
    pt = &gps->points[i];
    if (pt->flag & GP_SPOINT_TAG) {
      i++;
      step = 1;
      continue;
    }
    pt_next = &gps->points[i + step];
    /* Do not recalc tagged points. */
    if (pt_next->flag & GP_SPOINT_TAG) {
      step++;
      continue;
    }
    /* Check if contiguous points are selected. */
    if (!use_unselected) {
      if (((pt->flag & GP_SPOINT_SELECT) == 0) || ((pt_next->flag & GP_SPOINT_SELECT) == 0)) {
        i++;
        step = 1;
        continue;
      }
    }
    float len_square = len_squared_v3v3(&pt->x, &pt_next->x);
    if (len_square <= th_square) {
      tagged = true;
      if (i != gps->totpoints - 1) {
        /* Tag second point for delete. */
        pt_next->flag |= GP_SPOINT_TAG;
      }
      else {
        pt->flag |= GP_SPOINT_TAG;
      }
      /* Jump to next pair of points, keeping first point segment equals.*/
      step++;
    }
    else {
      /* Analyze next point. */
      i++;
      step = 1;
    }
  }

  /* Always untag extremes. */
  pt = &gps->points[0];
  pt->flag &= ~GP_SPOINT_TAG;
  pt = &gps->points[gps->totpoints - 1];
  pt->flag &= ~GP_SPOINT_TAG;

  /* Dissolve tagged points */
  if (tagged) {
    BKE_gpencil_dissolve_points(gpf, gps, GP_SPOINT_TAG);
  }

  /* Calc geometry data. */
  BKE_gpencil_stroke_geometry_update(gps);
}

/* Helper: Check materials with same color. */
static int gpencil_check_same_material_color(Object *ob_gp, float color[4], Material **r_mat)
{
  Material *ma = NULL;
  float color_cu[4];
  linearrgb_to_srgb_v3_v3(color_cu, color);
  float hsv1[4];
  rgb_to_hsv_v(color_cu, hsv1);
  hsv1[3] = color[3];

  for (int i = 1; i <= ob_gp->totcol; i++) {
    ma = BKE_object_material_get(ob_gp, i);
    MaterialGPencilStyle *gp_style = ma->gp_style;
    /* Check color with small tolerance (better in HSV). */
    float hsv2[4];
    rgb_to_hsv_v(gp_style->fill_rgba, hsv2);
    hsv2[3] = gp_style->fill_rgba[3];
    if ((gp_style->fill_style == GP_MATERIAL_FILL_STYLE_SOLID) &&
        (compare_v4v4(hsv1, hsv2, 0.01f))) {
      *r_mat = ma;
      return i - 1;
    }
  }

  *r_mat = NULL;
  return -1;
}

/* Helper: Add gpencil material using material as base. */
static Material *gpencil_add_material(Main *bmain,
                                      Object *ob_gp,
                                      char *name,
                                      const float color[4],
                                      const bool use_stroke,
                                      const bool use_fill,
                                      int *r_idx)
{
  Material *mat_gp = BKE_gpencil_object_material_new(bmain, ob_gp, name, r_idx);
  MaterialGPencilStyle *gp_style = mat_gp->gp_style;

  /* Stroke color. */
  if (use_stroke) {
    ARRAY_SET_ITEMS(gp_style->stroke_rgba, 0.0f, 0.0f, 0.0f, 1.0f);
    gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
  }
  else {
    linearrgb_to_srgb_v4(gp_style->stroke_rgba, color);
    gp_style->flag &= ~GP_MATERIAL_STROKE_SHOW;
  }

  /* Fill color. */
  linearrgb_to_srgb_v4(gp_style->fill_rgba, color);
  if (use_fill) {
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
                                   float *coord_array,
                                   float pressure,
                                   int init,
                                   int totpoints,
                                   const float init_co[3],
                                   bool last)
{
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

    pt->pressure = pressure;
    pt->strength = 1.0f;
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

/* Helper: Convert one spline to grease pencil stroke. */
static void gpencil_convert_spline(Main *bmain,
                                   Object *ob_gp,
                                   Object *ob_cu,
                                   const bool gpencil_lines,
                                   const bool only_stroke,
                                   bGPDframe *gpf,
                                   Nurb *nu)
{
  Curve *cu = (Curve *)ob_cu->data;
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
  if (((nu->flagu & CU_NURB_CYCLIC) == 0) || (nu->pntsu == 2)) {
    segments--;
    cyclic = false;
  }
  totpoints = (resolu * segments) - (segments - 1);

  /* Materials
   * Notice: The color of the material is the color of viewport and not the final shader color.
   */
  Material *mat_gp = NULL;
  bool fill = true;
  /* Check if grease pencil has a material with same color.*/
  float color[4];
  if ((cu->mat) && (*cu->mat)) {
    Material *mat_cu = *cu->mat;
    copy_v4_v4(color, &mat_cu->r);
  }
  else {
    /* Gray (unassigned from SVG add-on) */
    zero_v4(color);
    add_v3_fl(color, 0.6f);
    color[3] = 1.0f;
    fill = false;
  }

  /* Special case: If the color was created by the SVG add-on and the name contains '_stroke' and
   * there is only one color, the stroke must not be closed, fill to false and use for
   * stroke the fill color.
   */
  bool do_stroke = false;
  if (ob_cu->totcol == 1) {
    Material *ma_stroke = BKE_object_material_get(ob_cu, 1);
    if ((ma_stroke) && (strstr(ma_stroke->id.name, "_stroke") != NULL)) {
      do_stroke = true;
    }
  }

  int r_idx = gpencil_check_same_material_color(ob_gp, color, &mat_gp);
  if ((ob_cu->totcol > 0) && (r_idx < 0)) {
    Material *mat_curve = BKE_object_material_get(ob_cu, 1);
    mat_gp = gpencil_add_material(bmain, ob_gp, "Material", color, gpencil_lines, fill, &r_idx);

    if ((mat_curve) && (mat_curve->gp_style != NULL)) {
      MaterialGPencilStyle *gp_style_cur = mat_curve->gp_style;
      MaterialGPencilStyle *gp_style_gp = mat_gp->gp_style;

      copy_v4_v4(gp_style_gp->mix_rgba, gp_style_cur->mix_rgba);
      gp_style_gp->fill_style = gp_style_cur->fill_style;
      gp_style_gp->mix_factor = gp_style_cur->mix_factor;
    }

    /* If object has more than 1 material, use second material for stroke color. */
    if ((!only_stroke) && (ob_cu->totcol > 1) && (BKE_object_material_get(ob_cu, 2))) {
      mat_curve = BKE_object_material_get(ob_cu, 2);
      if (mat_curve) {
        linearrgb_to_srgb_v3_v3(mat_gp->gp_style->stroke_rgba, &mat_curve->r);
        mat_gp->gp_style->stroke_rgba[3] = mat_curve->a;
      }
    }
    else if ((only_stroke) || (do_stroke)) {
      /* Also use the first color if the fill is none for stroke color. */
      if (ob_cu->totcol > 0) {
        mat_curve = BKE_object_material_get(ob_cu, 1);
        if (mat_curve) {
          copy_v3_v3(mat_gp->gp_style->stroke_rgba, &mat_curve->r);
          mat_gp->gp_style->stroke_rgba[3] = mat_curve->a;
          /* Set fill and stroke depending of curve type (3D or 2D). */
          if ((cu->flag & CU_3D) || ((cu->flag & (CU_FRONT | CU_BACK)) == 0)) {
            mat_gp->gp_style->flag |= GP_MATERIAL_STROKE_SHOW;
            mat_gp->gp_style->flag &= ~GP_MATERIAL_FILL_SHOW;
          }
          else {
            mat_gp->gp_style->flag &= ~GP_MATERIAL_STROKE_SHOW;
            mat_gp->gp_style->flag |= GP_MATERIAL_FILL_SHOW;
          }
        }
      }
    }
  }
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
      if (((nu->flagu & CU_NURB_CYCLIC) == 0) || (nu->pntsu == 2)) {
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
                                        3 * sizeof(float));
        }
        /* Save first point coordinates. */
        if (s == 0) {
          copy_v3_v3(init_co, &coord_array[0]);
        }
        /* Add points to the stroke */
        gpencil_add_new_points(gps, coord_array, bezt->radius, init, resolu, init_co, last);
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
        gps->totpoints = nurb_points - 1;
        gps->points = MEM_callocN(sizeof(bGPDspoint) * gps->totpoints, "gp_stroke_points");

        /* Add points. */
        gpencil_add_new_points(gps, coord_array, 1.0f, 0, gps->totpoints, init_co, false);

        MEM_SAFE_FREE(coord_array);
      }
      break;
    }
    default: {
      break;
    }
  }
  /* Cyclic curve, close stroke. */
  if ((cyclic) && (!do_stroke)) {
    BKE_gpencil_stroke_close(gps);
  }

  /* Recalc fill geometry. */
  BKE_gpencil_stroke_geometry_update(gps);
}

/* Convert a curve object to grease pencil stroke.
 *
 * \param bmain: Main thread pointer
 * \param scene: Original scene.
 * \param ob_gp: Grease pencil object to add strokes.
 * \param ob_cu: Curve to convert.
 * \param gpencil_lines: Use lines for strokes.
 * \param use_collections: Create layers using collection names.
 * \param only_stroke: The material must be only stroke without fill.
 */
void BKE_gpencil_convert_curve(Main *bmain,
                               Scene *scene,
                               Object *ob_gp,
                               Object *ob_cu,
                               const bool gpencil_lines,
                               const bool use_collections,
                               const bool only_stroke)
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
    gpencil_convert_spline(bmain, ob_gp, ob_cu, gpencil_lines, only_stroke, gpf, nu);
  }

  /* Tag for recalculation */
  DEG_id_tag_update(&gpd->id, ID_RECALC_GEOMETRY | ID_RECALC_COPY_ON_WRITE);
}

typedef struct GpEdge {
  uint v1, v2;
  /* Coordinates. */
  float v1_co[3], v2_co[3];
  /* Normals. */
  float n1[3], n2[3];
  /* Direction of the segment. */
  float vec[3];
  int flag;
} GpEdge;

static int gpencil_next_edge(
    GpEdge *gp_edges, int totedges, GpEdge *gped_init, const float threshold, const bool reverse)
{
  int edge = -1;
  float last_angle = 999999.0f;
  for (int i = 0; i < totedges; i++) {
    GpEdge *gped = &gp_edges[i];
    if (gped->flag != 0) {
      continue;
    }
    if (reverse) {
      if (gped_init->v1 != gped->v2) {
        continue;
      }
    }
    else {
      if (gped_init->v2 != gped->v1) {
        continue;
      }
    }
    /* Look for straight lines. */
    float angle = angle_v3v3(gped->vec, gped_init->vec);
    if ((angle < threshold) && (angle <= last_angle)) {
      edge = i;
      last_angle = angle;
    }
  }

  return edge;
}

static int gpencil_walk_edge(GHash *v_table,
                             GpEdge *gp_edges,
                             int totedges,
                             uint *stroke_array,
                             int init_idx,
                             const float angle,
                             const bool reverse)
{
  GpEdge *gped_init = &gp_edges[init_idx];
  int idx = 1;
  int edge = 0;
  while (edge > -1) {
    edge = gpencil_next_edge(gp_edges, totedges, gped_init, angle, reverse);
    if (edge > -1) {
      GpEdge *gped = &gp_edges[edge];
      stroke_array[idx] = edge;
      gped->flag = 1;
      gped_init = &gp_edges[edge];
      idx++;

      /* Avoid to follow already visited vertice. */
      if (reverse) {
        if (BLI_ghash_haskey(v_table, POINTER_FROM_INT(gped->v1))) {
          edge = -1;
        }
        else {
          BLI_ghash_insert(v_table, POINTER_FROM_INT(gped->v1), POINTER_FROM_INT(gped->v1));
        }
      }
      else {
        if (BLI_ghash_haskey(v_table, POINTER_FROM_INT(gped->v2))) {
          edge = -1;
        }
        else {
          BLI_ghash_insert(v_table, POINTER_FROM_INT(gped->v2), POINTER_FROM_INT(gped->v2));
        }
      }
    }
  }

  return idx;
}

static void gpencil_generate_edgeloops(Object *ob,
                                       bGPDframe *gpf_stroke,
                                       const float angle,
                                       const int thickness,
                                       const float offset,
                                       const float matrix[4][4],
                                       const bool use_seams)
{
  Mesh *me = (Mesh *)ob->data;
  if (me->totedge == 0) {
    return;
  }

  /* Arrays for all edge vertices (forward and backward) that form a edge loop.
   * This is reused for each edgeloop to create gpencil stroke. */
  uint *stroke = MEM_callocN(sizeof(uint) * me->totedge * 2, __func__);
  uint *stroke_fw = MEM_callocN(sizeof(uint) * me->totedge, __func__);
  uint *stroke_bw = MEM_callocN(sizeof(uint) * me->totedge, __func__);

  /* Create array with all edges. */
  GpEdge *gp_edges = MEM_callocN(sizeof(GpEdge) * me->totedge, __func__);
  GpEdge *gped = NULL;
  for (int i = 0; i < me->totedge; i++) {
    MEdge *ed = &me->medge[i];
    gped = &gp_edges[i];
    MVert *mv1 = &me->mvert[ed->v1];
    normal_short_to_float_v3(gped->n1, mv1->no);

    gped->v1 = ed->v1;
    copy_v3_v3(gped->v1_co, mv1->co);

    MVert *mv2 = &me->mvert[ed->v2];
    normal_short_to_float_v3(gped->n2, mv2->no);
    gped->v2 = ed->v2;
    copy_v3_v3(gped->v2_co, mv2->co);

    sub_v3_v3v3(gped->vec, mv1->co, mv2->co);

    /* If use seams, mark as done if not a seam. */
    if ((use_seams) && ((ed->flag & ME_SEAM) == 0)) {
      gped->flag = 1;
    }
  }

  /* Loop edges to find edgeloops */
  bool pending = true;
  int e = 0;
  while (pending) {
    /* Clear arrays of stroke. */
    memset(stroke_fw, 0, sizeof(uint) * me->totedge);
    memset(stroke_bw, 0, sizeof(uint) * me->totedge);
    memset(stroke, 0, sizeof(uint) * me->totedge * 2);

    gped = &gp_edges[e];
    /* Look first unused edge. */
    if (gped->flag != 0) {
      e++;
      if (e == me->totedge) {
        pending = false;
      }
      continue;
    }
    /* Add current edge to arrays. */
    stroke_fw[0] = e;
    stroke_bw[0] = e;
    gped->flag = 1;

    /* Hash used to avoid loop over same vertice. */
    GHash *v_table = BLI_ghash_int_new(__func__);
    /* Look forward edges. */
    int totedges = gpencil_walk_edge(v_table, gp_edges, me->totedge, stroke_fw, e, angle, false);
    /* Look backward edges. */
    int totbw = gpencil_walk_edge(v_table, gp_edges, me->totedge, stroke_bw, e, angle, false);

    BLI_ghash_free(v_table, NULL, NULL);

    /* Join both arrays. */
    int array_len = 0;
    for (int i = totbw - 1; i > 0; i--) {
      stroke[array_len] = stroke_bw[i];
      array_len++;
    }
    for (int i = 0; i < totedges; i++) {
      stroke[array_len] = stroke_fw[i];
      array_len++;
    }

    /* Create Stroke. */
    bGPDstroke *gps_stroke = BKE_gpencil_stroke_add(
        gpf_stroke, 0, array_len + 1, thickness * thickness, false);

    /* Create first segment. */
    float fpt[3];
    uint v = stroke[0];
    gped = &gp_edges[v];
    bGPDspoint *pt = &gps_stroke->points[0];
    mul_v3_v3fl(fpt, gped->n1, offset);
    add_v3_v3v3(&pt->x, gped->v1_co, fpt);
    mul_m4_v3(matrix, &pt->x);

    pt->pressure = 1.0f;
    pt->strength = 1.0f;

    pt = &gps_stroke->points[1];
    mul_v3_v3fl(fpt, gped->n2, offset);
    add_v3_v3v3(&pt->x, gped->v2_co, fpt);
    mul_m4_v3(matrix, &pt->x);

    pt->pressure = 1.0f;
    pt->strength = 1.0f;

    /* Add next segments. */
    for (int i = 1; i < array_len; i++) {
      v = stroke[i];
      gped = &gp_edges[v];

      bGPDspoint *pt = &gps_stroke->points[i + 1];
      mul_v3_v3fl(fpt, gped->n2, offset);
      add_v3_v3v3(&pt->x, gped->v2_co, fpt);
      mul_m4_v3(matrix, &pt->x);

      pt->pressure = 1.0f;
      pt->strength = 1.0f;
    }

    BKE_gpencil_stroke_geometry_update(gps_stroke);
  }

  /* Free memory. */
  MEM_SAFE_FREE(stroke);
  MEM_SAFE_FREE(stroke_fw);
  MEM_SAFE_FREE(stroke_bw);
  MEM_SAFE_FREE(gp_edges);
}

/* Convert a mesh object to grease pencil stroke.
 *
 * \param bmain: Main thread pointer
 * \param depsgraph: Original depsgraph.
 * \param scene: Original scene.
 * \param ob_gp: Grease pencil object to add strokes.
 * \param ob_mesh: Mesh to convert.
 * \param angle: Limit angle to consider a edgeloop ends.
 * \param thickness: Thickness of the strokes.
 * \param offset: Offset along the normals.
 * \param matrix: Transformation matrix.
 * \param frame_offset: Destination frame number offset.
 * \param use_seams: Only export seam edges.
 * \param use_faces: Export faces as filled strokes.
 */
void BKE_gpencil_convert_mesh(Main *bmain,
                              Depsgraph *depsgraph,
                              Scene *scene,
                              Object *ob_gp,
                              Object *ob_mesh,
                              const float angle,
                              const int thickness,
                              const float offset,
                              const float matrix[4][4],
                              const int frame_offset,
                              const bool use_seams,
                              const bool use_faces)
{
  if (ELEM(NULL, ob_gp, ob_mesh) || (ob_gp->type != OB_GPENCIL) || (ob_gp->data == NULL)) {
    return;
  }

  bGPdata *gpd = (bGPdata *)ob_gp->data;

  /* Use evaluated data to get mesh with all modifiers on top. */
  Object *ob_eval = (Object *)DEG_get_evaluated_object(depsgraph, ob_mesh);
  Mesh *me_eval = BKE_object_get_evaluated_mesh(ob_eval);
  MPoly *mp, *mpoly = me_eval->mpoly;
  MLoop *mloop = me_eval->mloop;
  int mpoly_len = me_eval->totpoly;
  int i;

  /* If the object has enough materials means it was created in a previous step. */
  const bool create_mat = (ob_gp->totcol >= ob_mesh->totcol) ? false : true;

  /* Need at least an edge. */
  if (me_eval->totvert < 2) {
    return;
  }

  int r_idx;
  const float default_colors[2][4] = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.7f, 0.7f, 0.7f, 1.0f}};
  /* Create stroke material. */
  if (create_mat) {
    gpencil_add_material(bmain, ob_gp, "Stroke", default_colors[0], true, false, &r_idx);
  }
  /* Export faces as filled strokes. */
  if (use_faces) {
    if (create_mat) {
      /* If no materials, create a simple fill. */
      if (ob_mesh->totcol == 0) {
        gpencil_add_material(bmain, ob_gp, "Fill", default_colors[1], false, true, &r_idx);
      }
      else {
        /* Create all materials for fill. */
        for (int i = 0; i < ob_mesh->totcol; i++) {
          Material *ma = BKE_object_material_get(ob_mesh, i + 1);
          float color[4];
          copy_v3_v3(color, &ma->r);
          color[3] = 1.0f;
          gpencil_add_material(bmain, ob_gp, ma->id.name + 2, color, false, true, &r_idx);
        }
      }
    }

    /* Read all polygons and create fill for each. */
    if (mpoly_len > 0) {
      bGPDlayer *gpl_fill = BKE_gpencil_layer_named_get(gpd, DATA_("Fills"));
      if (gpl_fill == NULL) {
        gpl_fill = BKE_gpencil_layer_addnew(gpd, DATA_("Fills"), true);
      }
      bGPDframe *gpf_fill = BKE_gpencil_layer_frame_get(
          gpl_fill, CFRA + frame_offset, GP_GETFRAME_ADD_NEW);
      for (i = 0, mp = mpoly; i < mpoly_len; i++, mp++) {
        MLoop *ml = &mloop[mp->loopstart];
        /* Create fill stroke. */
        bGPDstroke *gps_fill = BKE_gpencil_stroke_add(
            gpf_fill, mp->mat_nr + 1, mp->totloop, 10, false);
        gps_fill->flag |= GP_STROKE_CYCLIC;

        /* Add points to strokes. */
        int j;
        for (j = 0; j < mp->totloop; j++, ml++) {
          MVert *mv = &me_eval->mvert[ml->v];

          bGPDspoint *pt = &gps_fill->points[j];
          copy_v3_v3(&pt->x, mv->co);
          mul_m4_v3(matrix, &pt->x);
          pt->pressure = 1.0f;
          pt->strength = 1.0f;
        }

        BKE_gpencil_stroke_geometry_update(gps_fill);
      }
    }
  }

  /* Create stroke from edges. */
  bGPDlayer *gpl_stroke = BKE_gpencil_layer_named_get(gpd, DATA_("Lines"));
  if (gpl_stroke == NULL) {
    gpl_stroke = BKE_gpencil_layer_addnew(gpd, DATA_("Lines"), true);
  }
  bGPDframe *gpf_stroke = BKE_gpencil_layer_frame_get(
      gpl_stroke, CFRA + frame_offset, GP_GETFRAME_ADD_NEW);
  gpencil_generate_edgeloops(ob_eval, gpf_stroke, angle, thickness, offset, matrix, use_seams);

  /* Tag for recalculation */
  DEG_id_tag_update(&gpd->id, ID_RECALC_GEOMETRY | ID_RECALC_COPY_ON_WRITE);
}
/* Apply Transforms */
void BKE_gpencil_transform(bGPdata *gpd, float mat[4][4])
{
  if (gpd == NULL) {
    return;
  }

  const float scalef = mat4_to_scale(mat);
  LISTBASE_FOREACH (bGPDlayer *, gpl, &gpd->layers) {
    /* FIXME: For now, we just skip parented layers.
     * Otherwise, we have to update each frame to find
     * the current parent position/effects.
     */
    if (gpl->parent) {
      continue;
    }

    LISTBASE_FOREACH (bGPDframe *, gpf, &gpl->frames) {
      LISTBASE_FOREACH (bGPDstroke *, gps, &gpf->strokes) {
        bGPDspoint *pt;
        int i;

        for (pt = gps->points, i = 0; i < gps->totpoints; pt++, i++) {
          mul_m4_v3(mat, &pt->x);
          pt->pressure *= scalef;
        }

        /* Distortion may mean we need to re-triangulate. */
        BKE_gpencil_stroke_geometry_update(gps);
      }
    }
  }
}

/**
 * Stroke to view space
 * Transforms a stroke to view space. This allows for manipulations in 2D but also easy conversion
 * back to 3D.
 * Note: also takes care of parent space transform
 */
void BKE_gpencil_stroke_to_view_space(const bContext *C, const bGPDlayer *gpl, bGPDstroke *gps)
{
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  ARegion *ar = CTX_wm_region(C);
  RegionView3D *rv3d = ar->regiondata;
  Object *ob = CTX_data_active_object(C);

  float tmp[3];
  float diff_mat[4][4];
  BKE_gpencil_parent_matrix_get(depsgraph, ob, gpl, diff_mat);

  for (int i = 0; i < gps->totpoints; i++) {
    bGPDspoint *pt = &gps->points[i];
    /* point to parent space */
    mul_v3_m4v3(tmp, diff_mat, &pt->x);
    /* point to view space */
    mul_m4_v3(rv3d->viewmat, tmp);
    copy_v3_v3(&pt->x, tmp);
  }
}

/**
 * Stroke from view space
 * Transforms a stroke from view space back to world space. Inverse of
 * BKE_gpencil_stroke_to_view_space
 * Note: also takes care of parent space transform
 */
void BKE_gpencil_stroke_from_view_space(const bContext *C, const bGPDlayer *gpl, bGPDstroke *gps)
{
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
  ARegion *ar = CTX_wm_region(C);
  RegionView3D *rv3d = ar->regiondata;
  Object *ob = CTX_data_active_object(C);

  float tmp[3];
  float diff_mat[4][4];
  float inverse_diff_mat[4][4];

  BKE_gpencil_parent_matrix_get(depsgraph, ob, gpl, diff_mat);
  invert_m4_m4(inverse_diff_mat, diff_mat);

  for (int i = 0; i < gps->totpoints; i++) {
    bGPDspoint *pt = &gps->points[i];
    mul_v3_m4v3(tmp, rv3d->viewinv, &pt->x);
    mul_m4_v3(inverse_diff_mat, tmp);
    copy_v3_v3(&pt->x, tmp);
  }
}

/* ----------------------------------------------------------------------------- */
/* Stroke to perimeter */

typedef struct tPerimeterPoint {
  struct tPerimeterPoint *next, *prev;
  float x, y, z;
  bool is_left;
} tPerimeterPoint;

static tPerimeterPoint *new_perimeter_point(const float pt[3], bool is_left)
{
  tPerimeterPoint *new_pt = MEM_callocN(sizeof(tPerimeterPoint), __func__);
  copy_v3_v3(&new_pt->x, pt);
  new_pt->is_left = is_left;
  return new_pt;
}

static int generate_arc_from_point_to_point(ListBase *list,
                                            tPerimeterPoint *from,
                                            tPerimeterPoint *to,
                                            float center_pt[3],
                                            int subdivisions,
                                            bool clockwise,
                                            bool is_left)
{
  float vec_from[2];
  float vec_to[2];
  sub_v2_v2v2(vec_from, &from->x, center_pt);
  sub_v2_v2v2(vec_to, &to->x, center_pt);
  if (is_zero_v2(vec_from) || is_zero_v2(vec_to)) {
    return 0;
  }

  float dot = dot_v2v2(vec_from, vec_to);
  float det = cross_v2v2(vec_from, vec_to);
  float angle = clockwise ? M_PI - atan2f(-det, -dot) : atan2f(-det, -dot) + M_PI;

  /* number of points is 2^(n+1) + 1 on half a circle (n=subdivisions)
   * so we multiply by (angle / pi) to get the right amount of
   * points to insert */
  int num_points = (int)(((1 << (subdivisions + 1)) - 1) * (angle / M_PI));
  if (num_points > 0) {
    float angle_incr = angle / (float)num_points;

    float vec_p[3];
    float vec_t[3];
    float tmp_angle;
    tPerimeterPoint *last_point;
    if (clockwise) {
      last_point = to;
      copy_v3_v3(vec_t, vec_to);
    }
    else {
      last_point = from;
      copy_v3_v3(vec_t, vec_from);
    }

    for (int i = 0; i < num_points - 1; i++) {
      tmp_angle = (i + 1) * angle_incr;

      rotate_v2_v2fl(vec_p, vec_t, tmp_angle);
      add_v2_v2(vec_p, center_pt);
      vec_p[2] = center_pt[2];

      tPerimeterPoint *new_point = new_perimeter_point(vec_p, is_left);
      if (clockwise) {
        BLI_insertlinkbefore(list, last_point, new_point);
      }
      else {
        BLI_insertlinkafter(list, last_point, new_point);
      }

      last_point = new_point;
    }

    return num_points - 1;
  }

  return 0;
}

static int generate_semi_circle_from_point_to_point(
    ListBase *list, tPerimeterPoint *from, tPerimeterPoint *to, int subdivisions, bool is_left)
{
  int num_points = (1 << (subdivisions + 1)) + 1;
  float center_pt[3];
  interp_v3_v3v3(center_pt, &from->x, &to->x, 0.5f);

  float vec_center[2];
  sub_v2_v2v2(vec_center, &from->x, center_pt);
  if (is_zero_v2(vec_center)) {
    return 0;
  }

  float vec_p[3];  // temp vector to do the vector math
  float angle_incr = M_PI / ((float)num_points - 1);

  tPerimeterPoint *last_point = from;
  for (int i = 1; i < num_points; i++) {
    float angle = i * angle_incr;

    /* rotate vector around point to get perimeter points */
    rotate_v2_v2fl(vec_p, vec_center, angle);
    add_v2_v2(vec_p, center_pt);
    vec_p[2] = center_pt[2];

    tPerimeterPoint *new_point = new_perimeter_point(vec_p, is_left);
    BLI_insertlinkafter(list, last_point, new_point);

    last_point = new_point;
  }

  return num_points - 1;
}

static int generate_perimeter_cap(const float point[4],
                                  const float other_point[4],
                                  float radius,
                                  ListBase *list,
                                  int subdivisions,
                                  short cap_type,
                                  bool is_left)
{
  float cap_vec[2];
  sub_v2_v2v2(cap_vec, other_point, point);
  normalize_v2(cap_vec);

  float cap_nvec[2];
  if (is_zero_v2(cap_vec)) {
    cap_nvec[0] = 0;
    cap_nvec[1] = radius;
  }
  else {
    cap_nvec[0] = -cap_vec[1];
    cap_nvec[1] = cap_vec[0];
    mul_v2_fl(cap_nvec, radius);
  }
  float cap_nvec_inv[2];
  negate_v2_v2(cap_nvec_inv, cap_nvec);

  float vec_perimeter[3];
  copy_v3_v3(vec_perimeter, point);
  add_v2_v2(vec_perimeter, cap_nvec);

  float vec_perimeter_inv[3];
  copy_v3_v3(vec_perimeter_inv, point);
  add_v2_v2(vec_perimeter_inv, cap_nvec_inv);

  tPerimeterPoint *p_pt = new_perimeter_point(vec_perimeter, is_left);
  tPerimeterPoint *p_pt_inv = new_perimeter_point(vec_perimeter_inv, is_left);

  BLI_addtail(list, p_pt);
  BLI_addtail(list, p_pt_inv);

  int num_points = 0;
  if (cap_type == GP_STROKE_CAP_ROUND) {
    num_points += generate_semi_circle_from_point_to_point(
        list, p_pt, p_pt_inv, subdivisions, is_left);
  }

  return num_points + 2;
}

/**
 * Calculate the perimeter (outline) of a stroke as list of tPerimeterPoint.
 * \param subdivisions: Number of subdivions for the start and end caps
 * \return: list of tPerimeterPoint
 */
static ListBase *gpencil_stroke_perimeter_ex(const bGPdata *gpd,
                                             const bGPDlayer *gpl,
                                             const bGPDstroke *gps,
                                             int subdivisions,
                                             int *r_num_perimeter_points)
{
  /* sanity check */
  if (gps->totpoints < 1) {
    return NULL;
  }

  float defaultpixsize = 1000.0f / gpd->pixfactor;
  float stroke_radius = ((gps->thickness + gpl->line_change) / defaultpixsize) / 2.0f;

  ListBase *perimeter_right_side = MEM_callocN(sizeof(ListBase), __func__);
  ListBase *perimeter_left_side = MEM_callocN(sizeof(ListBase), __func__);
  int num_perimeter_points = 0;

  bGPDspoint *first = &gps->points[0];
  bGPDspoint *last = &gps->points[gps->totpoints - 1];

  float first_radius = stroke_radius * first->pressure;
  float last_radius = stroke_radius * last->pressure;

  bGPDspoint *first_next;
  bGPDspoint *last_prev;
  if (gps->totpoints > 1) {
    first_next = &gps->points[1];
    last_prev = &gps->points[gps->totpoints - 2];
  }
  else {
    first_next = first;
    last_prev = last;
  }

  float first_pt[3];
  float last_pt[3];
  float first_next_pt[3];
  float last_prev_pt[3];
  copy_v3_v3(first_pt, &first->x);
  copy_v3_v3(last_pt, &last->x);
  copy_v3_v3(first_next_pt, &first_next->x);
  copy_v3_v3(last_prev_pt, &last_prev->x);

  /* edgecase if single point */
  if (gps->totpoints == 1) {
    first_next_pt[0] += 1.0f;
    last_prev_pt[0] -= 1.0f;
  }

  /* generate points for start cap */
  num_perimeter_points += generate_perimeter_cap(first_pt,
                                                 first_next_pt,
                                                 first_radius,
                                                 perimeter_right_side,
                                                 subdivisions,
                                                 gps->caps[0],
                                                 false);

  /* generate perimeter points  */
  float curr_pt[3], next_pt[3], prev_pt[3];
  float vec_next[2], vec_prev[2];
  float nvec_next[2], nvec_prev[2];
  float nvec_next_pt[3], nvec_prev_pt[3];
  float vec_tangent[2];

  float vec_miter_left[2], vec_miter_right[2];
  float miter_left_pt[3], miter_right_pt[3];

  for (int i = 1; i < gps->totpoints - 1; i++) {
    bGPDspoint *curr = &gps->points[i];
    bGPDspoint *prev = &gps->points[i - 1];
    bGPDspoint *next = &gps->points[i + 1];
    float radius = stroke_radius * curr->pressure;

    copy_v3_v3(curr_pt, &curr->x);
    copy_v3_v3(next_pt, &next->x);
    copy_v3_v3(prev_pt, &prev->x);

    sub_v2_v2v2(vec_prev, curr_pt, prev_pt);
    sub_v2_v2v2(vec_next, next_pt, curr_pt);
    float prev_length = len_v2(vec_prev);
    float next_length = len_v2(vec_next);

    if (normalize_v2(vec_prev) == 0.0f) {
      vec_prev[0] = 1.0f;
      vec_prev[1] = 0.0f;
    }
    if (normalize_v2(vec_next) == 0.0f) {
      vec_next[0] = 1.0f;
      vec_next[1] = 0.0f;
    }

    nvec_prev[0] = -vec_prev[1];
    nvec_prev[1] = vec_prev[0];

    nvec_next[0] = -vec_next[1];
    nvec_next[1] = vec_next[0];

    add_v2_v2v2(vec_tangent, vec_prev, vec_next);
    if (normalize_v2(vec_tangent) == 0.0f) {
      copy_v2_v2(vec_tangent, nvec_prev);
    }

    vec_miter_left[0] = -vec_tangent[1];
    vec_miter_left[1] = vec_tangent[0];

    /* calculate miter length */
    float an1 = dot_v2v2(vec_miter_left, nvec_prev);
    if (an1 == 0.0f) {
      an1 = 1.0f;
    }
    float miter_length = radius / an1;
    if (miter_length <= 0.0f) {
      miter_length = 0.01f;
    }

    normalize_v2_length(vec_miter_left, miter_length);

    copy_v2_v2(vec_miter_right, vec_miter_left);
    negate_v2(vec_miter_right);

    float angle = dot_v2v2(vec_next, nvec_prev);
    /* add two points if angle is close to beeing straight */
    if (fabsf(angle) < 0.0001f) {
      normalize_v2_length(nvec_prev, radius);
      normalize_v2_length(nvec_next, radius);

      copy_v3_v3(nvec_prev_pt, curr_pt);
      add_v2_v2(nvec_prev_pt, nvec_prev);

      copy_v3_v3(nvec_next_pt, curr_pt);
      negate_v2(nvec_next);
      add_v2_v2(nvec_next_pt, nvec_next);

      tPerimeterPoint *normal_prev = new_perimeter_point(nvec_prev_pt, true);
      tPerimeterPoint *normal_next = new_perimeter_point(nvec_next_pt, false);

      BLI_addtail(perimeter_left_side, normal_prev);
      BLI_addtail(perimeter_right_side, normal_next);
      num_perimeter_points += 2;
    }
    else {
      /* bend to the left */
      if (angle < 0.0f) {
        normalize_v2_length(nvec_prev, radius);
        normalize_v2_length(nvec_next, radius);

        copy_v3_v3(nvec_prev_pt, curr_pt);
        add_v2_v2(nvec_prev_pt, nvec_prev);

        copy_v3_v3(nvec_next_pt, curr_pt);
        add_v2_v2(nvec_next_pt, nvec_next);

        tPerimeterPoint *normal_prev = new_perimeter_point(nvec_prev_pt, true);
        tPerimeterPoint *normal_next = new_perimeter_point(nvec_next_pt, true);

        BLI_addtail(perimeter_left_side, normal_prev);
        BLI_addtail(perimeter_left_side, normal_next);
        num_perimeter_points += 2;

        num_perimeter_points += generate_arc_from_point_to_point(
            perimeter_left_side, normal_prev, normal_next, curr_pt, subdivisions, true, true);

        if (miter_length < prev_length && miter_length < next_length) {
          copy_v3_v3(miter_right_pt, curr_pt);
          add_v2_v2(miter_right_pt, vec_miter_right);
        }
        else {
          copy_v3_v3(miter_right_pt, curr_pt);
          negate_v2(nvec_next);
          add_v2_v2(miter_right_pt, nvec_next);
        }

        tPerimeterPoint *miter_right = new_perimeter_point(miter_right_pt, false);
        BLI_addtail(perimeter_right_side, miter_right);
        num_perimeter_points++;
      }
      /* bend to the right */
      else {
        normalize_v2_length(nvec_prev, -radius);
        normalize_v2_length(nvec_next, -radius);

        copy_v3_v3(nvec_prev_pt, curr_pt);
        add_v2_v2(nvec_prev_pt, nvec_prev);

        copy_v3_v3(nvec_next_pt, curr_pt);
        add_v2_v2(nvec_next_pt, nvec_next);

        tPerimeterPoint *normal_prev = new_perimeter_point(nvec_prev_pt, false);
        tPerimeterPoint *normal_next = new_perimeter_point(nvec_next_pt, false);

        BLI_addtail(perimeter_right_side, normal_prev);
        BLI_addtail(perimeter_right_side, normal_next);
        num_perimeter_points += 2;

        num_perimeter_points += generate_arc_from_point_to_point(
            perimeter_right_side, normal_prev, normal_next, curr_pt, subdivisions, false, false);

        if (miter_length < prev_length && miter_length < next_length) {
          copy_v3_v3(miter_left_pt, curr_pt);
          add_v2_v2(miter_left_pt, vec_miter_left);
        }
        else {
          copy_v3_v3(miter_left_pt, curr_pt);
          negate_v2(nvec_prev);
          add_v2_v2(miter_left_pt, nvec_prev);
        }

        tPerimeterPoint *miter_left = new_perimeter_point(miter_left_pt, true);
        BLI_addtail(perimeter_left_side, miter_left);
        num_perimeter_points++;
      }
    }
  }

  /* generate points for end cap */
  num_perimeter_points += generate_perimeter_cap(
      last_pt, last_prev_pt, last_radius, perimeter_right_side, subdivisions, gps->caps[1], false);

  /* merge both sides to one list */
  BLI_listbase_reverse(perimeter_right_side);
  BLI_movelisttolist(perimeter_left_side,
                     perimeter_right_side);  // perimeter_left_side contains entire list
  ListBase *perimeter_list = perimeter_left_side;

  /* close by creating a point close to the first (make a small gap) */
  float close_pt[3];
  tPerimeterPoint *close_first = (tPerimeterPoint *)perimeter_list->first;
  tPerimeterPoint *close_last = (tPerimeterPoint *)perimeter_list->last;
  interp_v3_v3v3(close_pt, &close_last->x, &close_first->x, 0.99f);

  if (compare_v3v3(close_pt, &close_first->x, FLT_EPSILON) == false) {
    tPerimeterPoint *close_p_pt = new_perimeter_point(close_pt, true);
    BLI_addtail(perimeter_list, close_p_pt);
    num_perimeter_points++;
  }

  /* free temp data */
  BLI_freelistN(perimeter_right_side);
  MEM_freeN(perimeter_right_side);

  *r_num_perimeter_points = num_perimeter_points;
  return perimeter_list;
}

/**
 * Calculates the perimeter of a stroke projected from the view and
 * returns it as a new stroke.
 * \param subdivisions: Number of subdivions for the start and end caps
 * \return: bGPDstroke pointer to stroke perimeter
 */
bGPDstroke *BKE_gpencil_stroke_perimeter_from_view(
    const bContext *C, const bGPdata *gpd, const bGPDlayer *gpl, bGPDstroke *gps, int subdivisions)
{
  if (gps->totpoints == 0) {
    return NULL;
  }

  BKE_gpencil_stroke_to_view_space(C, gpl, gps);
  int num_perimeter_points = 0;
  ListBase *perimeter_points = gpencil_stroke_perimeter_ex(
      gpd, gpl, gps, subdivisions, &num_perimeter_points);

  if (num_perimeter_points == 0) {
    return NULL;
  }

  /* create new stroke */
  bGPDstroke *perimeter_stroke = BKE_gpencil_stroke_new(gps->mat_nr, num_perimeter_points, 1);

  tPerimeterPoint *curr = perimeter_points->first;
  for (int i = 0; i < num_perimeter_points; i++) {
    bGPDspoint *pt = &perimeter_stroke->points[i];

    copy_v3_v3(&pt->x, &curr->x);

    /* Set pressure to zero and strength to one */
    pt->pressure = 0.0f;
    pt->strength = 1.0f;

    pt->flag |= GP_SPOINT_SELECT;
    pt->flag |= (curr->is_left) ? 0 : GP_SPOINT_RIGHT_SIDE;

    curr = curr->next;
  }

  BKE_gpencil_stroke_from_view_space(C, gpl, perimeter_stroke);

  /* free temp data */
  BLI_freelistN(perimeter_points);
  MEM_freeN(perimeter_points);

  /* triangles cache needs to be recalculated */
  BKE_gpencil_stroke_geometry_update(perimeter_stroke);

  perimeter_stroke->flag |= GP_STROKE_SELECT | GP_STROKE_CYCLIC;

  return perimeter_stroke;
}

/** \} */
