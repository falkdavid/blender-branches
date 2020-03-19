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

#ifdef __cplusplus
extern "C" {
#endif

/* WAVL Tree Implementation:
 *
 * WAVL-trees (or weak AVL-trees) are a hybrid of RB-trees and AVL-trees.
 * They where first introduced in the paper "Rank-balanced trees" by
 * Haeupler, Bernhard; Sen, Siddhartha; Tarjan, Robert E. (2015)
 * http://sidsen.azurewebsites.net//papers/rb-trees-talg.pdf.
 * The main advantage of AVL-trees over RB-trees is that they are more
 * balanced. RB-trees however use a constant amount of tree rotations
 * when deleting a node, compared to a (worst case) logarithmic
 * number using AVL-trees. WAVL-trees have both of the better properties.
 */

/* ********************************************** */

/* Data structs*/

/* Node data */
typedef struct WAVL_Node {
  /* pointers for tree structure */
  struct WAVL_Node *left, *right;
  struct WAVL_Node *parent;
  /* pointers to successor and predecessor
   * for quick access */
  struct WAVL_Node *succ, *pred;
  /* Size of the subtree including the node */
  uint size;
  /* The rank is an approximation of the distance
   * to the farthest leaf descendant.
   * (used internally to balance the BST) */
  int rank;
  /* pointer to the data in the node */
  void *data;
} WAVL_Node;

/* WAVL Tree data */
typedef struct WAVL_Tree {
  /* root node */
  struct WAVL_Node *root;
  /* pointers to min and max node
   * for quick access */
  struct WAVL_Node *min_node, *max_node;
} WAVL_Tree;

/* ********************************************** */
/* Callbacks */

/**
 * Callback function that returns -1, 0, 1 depending on
 * whether the data in WAVL_Node A is less than, equal,
 * or greater than the data in WAVL_Node B.
 */
typedef short (*WAVLT_comparator_FP)(void *data_a, void *data_b);

/**
 * Callback function that frees the data of a node.
 */
typedef void (*WAVLT_free_data_FP)(void *data);

/* ********************************************** */
/* Public API */

struct WAVL_Tree *BLI_wavlTree_new(void);
void BLI_wavlTree_free(struct WAVL_Tree *tree, WAVLT_free_data_FP free_data);

bool BLI_wavlTree_empty(const struct WAVL_Tree *tree);
uint BLI_wavlTree_size(const struct WAVL_Tree *tree);

struct WAVL_Node *BLI_wavlTree_search(const struct WAVL_Tree *tree, WAVLT_comparator_FP cmp, void *search_data);
struct WAVL_Node *BLI_wavlTree_successor_ex(const struct WAVL_Node *node);
struct WAVL_Node *BLI_wavlTree_successor(const struct WAVL_Tree *tree, WAVLT_comparator_FP cmp, void *data);
struct WAVL_Node *BLI_wavlTree_predecessor_ex(const struct WAVL_Node *node);
struct WAVL_Node *BLI_wavlTree_predecessor(const struct WAVL_Tree *tree, WAVLT_comparator_FP cmp, void *data);
struct WAVL_Node *BLI_wavlTree_min(const struct WAVL_Tree *tree);
struct WAVL_Node *BLI_wavlTree_max(const struct WAVL_Tree *tree);
void *BLI_wavlTree_min_data(const struct WAVL_Tree *tree);
void *BLI_wavlTree_max_data(const struct WAVL_Tree *tree);

struct WAVL_Node *BLI_wavlTree_insert(struct WAVL_Tree *tree, WAVLT_comparator_FP cmp, void *data);
struct WAVL_Node *BLI_wavlTree_update_node(struct WAVL_Tree *tree, WAVLT_comparator_FP cmp, struct WAVL_Node *node);
void BLI_wavlTree_delete_node(struct WAVL_Tree *tree, WAVLT_free_data_FP free_data, struct WAVL_Node *node);
void BLI_wavlTree_delete(struct WAVL_Tree *tree, WAVLT_comparator_FP cmp, WAVLT_free_data_FP free_data, void *data);

/* Macro for in-order walk over the the data in the tree (uses predecessor-successor DLL).
 * The type should be that of the data in a node */
#define WAVLTREE_INORDER(type, var, tree) \
  type var = (type)((tree)->min_node->data); \
  for (WAVL_Node *_curr = (tree)->min_node; _curr != NULL; \
  _curr = _curr->succ, var = (_curr != NULL) ? (type)(_curr->data) : NULL)

#define WAVLTREE_REVERSE_INORDER(type, var, tree) \
  type var = (type)((tree)->max_node->data); \
  for (WAVL_Node *_curr = (tree)->max_node; _curr != NULL; \
  _curr = _curr->pred, var = (_curr != NULL) ? (type)(_curr->data) : NULL)

/* ********************************************** */

#ifdef __cplusplus
}
#endif

#endif /* __BLI_WAVLTREE_H__ */
