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
 *
 * A WAVL-tree (weak AVL-tree), balanced binary search tree.
 */

#include <stdlib.h>
#include <string.h>

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_wavlTree.h"
#include "BLI_strict_flags.h"

BLI_INLINE bool binary(WAVLT_Node *node) 
{
  return (node->left != NULL) && (node->right != NULL);
}

BLI_INLINE bool unary(WAVLT_Node *node) 
{
  return (node->left != NULL) ^ (node->right != NULL);
}

BLI_INLINE bool leaf(WAVLT_Node *node) 
{
  return (node->left == NULL) && (node->right == NULL);
}

BLI_INLINE void demote(WAVLT_Node *node)
{
  node->rank--;
}

BLI_INLINE void promote(WAVLT_Node *node)
{
  node->rank++;
}

/* Helper: check if the node is of type l,r */
BLI_INLINE bool check_node_type(int l, int r, WAVLT_Node *node)
{
  /* if a child is NULL (external) it's rank is -1 */
  int l_diff = (node->left != NULL) ? node->rank - node->left->rank : node->rank + 1;
  int r_diff = (node->right != NULL) ? node->rank - node->right->rank : node->rank + 1;
  //printf("l_diff: %d, r_diff: %d\n", r_diff, l_diff);
  if ((l == l_diff) && (r == r_diff)) {
    return true;
  }
  return false;
}

BLI_INLINE void update_size(WAVLT_Node *node)
{
  size_t new_size = 1;
  if (node->left != NULL) {
    new_size += node->left->size;
  }
  if (node->right != NULL) {
    new_size += node->right->size;
  }
  node->size = new_size;
}

static WAVLT_Node *new_wavl_node_with_data(void *data)
{
  WAVLT_Node *new_node = MEM_callocN(sizeof(WAVLT_Node), __func__);
  new_node->parent = new_node->left = new_node->right = NULL;
  new_node->succ = new_node->pred = NULL;
  new_node->size = 1;
  new_node->rank = 0;
  new_node->data = data;

  return new_node;
}

/* Helper: performs a right rotation on the given node */
static void right_rotate(WAVLT_Tree *tree, WAVLT_Node *node)
{
  if (node->left == NULL) {
    return;
  }

  WAVLT_Node *x = node->left;
  WAVLT_Node *b = x->right;
  WAVLT_Node *p = node->parent;

  node->left = b;
  x->right = node;
  node->parent = x;
  x->parent = p;

  if (node == tree->root) {
    tree->root = x;
  }

  update_size(node);
  update_size(x);
}

/* Helper: performs a left rotation on the given node */
static void left_rotate(WAVLT_Tree *tree, WAVLT_Node *node)
{
  if (node->right == NULL) {
    return;
  }

  WAVLT_Node *x = node->right;
  WAVLT_Node *b = x->left;
  WAVLT_Node *p = node->parent;

  node->right = b;
  x->left = node;
  node->parent = x;
  x->parent = p;

  if (node == tree->root) {
    tree->root = x;
  }

  update_size(node);
  update_size(x);
}

static void wavlTree_rebalance_insert(WAVLT_Tree *tree, WAVLT_Node *node)
{
  if (node == NULL) {
    return;
  }
  /* repeat case 1: simple promotion */
  while (check_node_type(0, 1, node) || check_node_type(1, 0, node)) {
    promote(node);
    update_size(node);
    if (node->parent == NULL) {
      break;
    }
    node = node->parent;
  }

  /* case 2: tree rotations */
  if (check_node_type(0, 2, node)) {
    /* rotate right */
    if (check_node_type(1, 2, node->left)) {
      right_rotate(tree, node);
      demote(node);
    }
    /* double rotate */
    else if (check_node_type(2, 1, node->left)) {

    }
  }
  else if (check_node_type(2, 0, node)){
    /* rotate left */
    if (check_node_type(2, 1, node->right)) {
      left_rotate(tree, node);
      demote(node);
    }
    /* double rotate */
    else if (check_node_type(1, 2, node->right)) {

    }
  }

  /* propagate the new size to the top */
  while (node != NULL) {
    update_size(node);
    node = node->parent;
  }
}

/** \name Public WAVl-tree API
 * \{ */

WAVLT_Tree *BLI_wavlTree_new()
{
  WAVLT_Tree *new_tree = MEM_mallocN(sizeof(WAVLT_Tree), __func__);
  new_tree->root = new_tree->min_node = new_tree->max_node = NULL;

  return new_tree;
}

void BLI_wavlTree_free(WAVLT_Tree *tree, WAVLT_free_data_FP free_data)
{
  if (tree == NULL) {
    return;
  }

  WAVLT_Node *succ;
  if (free_data != NULL) {
    for (WAVLT_Node *curr = tree->min_node; curr != NULL; curr = succ) {
      succ = curr->succ;
      free_data(curr->data);
      MEM_freeN(curr);
    }
  }
  else {
    for (WAVLT_Node *curr = tree->min_node; curr != NULL; curr = succ) {
      succ = curr->succ;
      MEM_freeN(curr);
    }
  }

  MEM_freeN(tree);
  tree->root = tree->min_node = tree->max_node = NULL;
}

bool BLI_wavlTree_empty(const WAVLT_Tree *tree)
{
  return (tree != NULL) ? tree->root == NULL : true;
}

size_t BLI_wavlTree_size(const WAVLT_Tree *tree)
{
  if (tree != NULL) {
    return (tree->root != NULL) ? tree->root->size : 0;
  }
  return 0;
}

WAVLT_Node *BLI_wavlTree_search(const WAVLT_Tree *tree, WAVLT_comparator_FP cmp, void *search_data)
{
  WAVLT_Node *current = tree->root;
  while (current != NULL) {
    short c = cmp(search_data, current->data);
    if (c == 0) {
      return current;
    }
    else if (c == -1) {
      current = current->left;
    }
    else {
      current = current->right;
    }
  }
  return NULL;
}

void *BLI_wavlTree_min(WAVLT_Tree *tree)
{
  return tree->min_node->data;
}

void *BLI_wavlTree_max(WAVLT_Tree *tree)
{
  return tree->max_node->data;
}

void BLI_wavlTree_insert(WAVLT_Tree *tree, WAVLT_comparator_FP cmp, void *data)
{
  /* if tree is empty, make root */
  if (tree->root == NULL) {
    WAVLT_Node *new_node = new_wavl_node_with_data(data);
    tree->root = new_node;
    tree->min_node = new_node;
    tree->max_node = new_node;
    return;
  }

  /* search for the leaf node to insert */
  bool insert_left;
  WAVLT_Node *current = tree->root;
  while (current != NULL) {
    /* compare data */
    short c = cmp(data, current->data);
    /* node already exists */
    if (c == 0) {
      return;
    }
    else if (c == -1) {
      /* insert to the left of current */
      if (current->left == NULL) {
        insert_left = true;
        break;
      }
      current = current->left;
    }
    else {
      /* insert to the right of current */
      if (current->right == NULL) {
        insert_left = false;
        break;
      }
      current = current->right;
    }
  }

  /* create a new node to insert */
  WAVLT_Node *new_node = new_wavl_node_with_data(data);
  if (insert_left) {
    /* update tree links */
    current->left = new_node;
    new_node->parent = current;

    /* update successor and predecessor */
    WAVLT_Node *curr_pred = current->pred;
    if (curr_pred != NULL) {
      curr_pred->succ = new_node;
    }
    current->pred = new_node;
    new_node->pred = curr_pred;
    new_node->succ = current;

    /* if insert before min, update min */
    if (tree->min_node == current) {
      tree->min_node = new_node;
    }
  }
  else {
    /* update tree links */
    current->right = new_node;
    new_node->parent = current;

    /* update successor and predecessor */
    WAVLT_Node *curr_succ = current->succ;
    if (curr_succ != NULL) {
      curr_succ->pred = new_node;
    }
    current->succ = new_node;
    new_node->succ = curr_succ;
    new_node->pred = current;

    /* if insert after max, update max */
    if (tree->max_node == current) {
      tree->max_node = new_node;
    }
  }

  wavlTree_rebalance_insert(tree, current);
}

/** \} */

/* Debug printing */

