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

/* Data Types and Type Defines  */

/* Node data */
typedef struct WAVLT_Node {
  struct WAVLT_Node *left, *right;
  struct WAVLT_Node *parent;

  /* The rank is an approximation of the distance 
   * to the farthest leaf descendant */
  int rank;
} WAVLT_Node;

/* WAVL Tree data */
typedef struct WAVLT_Tree {
  /* Root Node */
  void *root;

  /* Number of nodes in the tree */
  size_t size;

  /* pointers to min and max node */
  void *min_node, *max_node;
} WAVLT_Tree;

/* ********************************************** */
/* Public API */

struct WAVLT_Tree *BLI_wavlTree_new(void);

size_t BLI_wavlTree_size(const struct WAVLT_Tree *tree);

/* ********************************************** */

#endif /* __BLI_WAVLTREE_H__ */
