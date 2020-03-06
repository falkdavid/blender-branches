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
 * The Original Code is Copyright (C) 2009 Blender Foundation, Joshua Leung
 * All rights reserved.
 */

#ifndef __BLI_WAVLTREE_H__
#define __BLI_WAVLTREE_H__

/** \file
 * \ingroup bli
 */

/* WAVL Tree Implementation:
 *
 * WAVL-trees (or weak AVL-trees) are a hybrid of RB-trees and AVL-trees.
 * They where first introduced in the paper "Rank-balanced trees" by 
 * Haeupler, Bernhard; Sen, Siddhartha; Tarjan, Robert E. (2015).
 * The main advantage of AVL-trees over RB-trees is that they are more
 * balanced. RB-trees however use a constant amount of tree rotations
 * when deleting a node, compared to a (worst case) logarithmic 
 * number using AVL-trees. WAVL-trees have both of the better properties.
 */

/* ********************************************** */

/* Data structs*/

/* Node data */
typedef struct WAVLT_Node {
  /* pointers for tree structure */
  struct WAVLT_Node *left, *right;
  struct WAVLT_Node *parent;
  /* pointers to successor and predecessor */
  struct WAVLT_Node *succ, *pred;
  /* Size of the subtree including the node */
  size_t size;
  /* The rank is an approximation of the distance 
   * to the farthest leaf descendant */
  int rank;
  /* pointer to the data */
  void *data;
} WAVLT_Node;

/* WAVL Tree data */
typedef struct WAVLT_Tree {
  /* root node */
  struct WAVLT_Node *root;
  /* pointers to min and max node */
  struct WAVLT_Node *min_node, *max_node;
} WAVLT_Tree;

/* ********************************************** */
/* Callbacks */

/**
 * Callback function that returns -1, 0, 1 depending on
 * whether the data in WAVLT_Node A is less than, equal,
 * or greater than the data in WAVLT_Node B.
 */
typedef short (*WAVLT_comparator_FP)(void *data_a, void *data_b);

/* ********************************************** */
/* Public API */

struct WAVLT_Tree *BLI_wavlTree_new(void);
void BLI_wavlTree_free(struct WAVLT_Tree *tree);

bool BLI_wavlTree_empty(const struct WAVLT_Tree *tree);
size_t BLI_wavlTree_size(const struct WAVLT_Tree *tree);

struct WAVLT_Node *BLI_wavlTree_search(const struct WAVLT_Tree *tree, WAVLT_comparator_FP cmp, void *search_data);

void BLI_wavlTree_insert_or_update(struct WAVLT_Tree *tree, void *data);

/* ********************************************** */

#endif /* __BLI_WAVLTREE_H__ */
